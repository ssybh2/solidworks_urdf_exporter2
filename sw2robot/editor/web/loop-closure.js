import { viewer } from './dom.js';
import { THREE } from './three-setup.js';

// URDF is a tree, so a CAD linkage with a kinematic loop has one or more
// movable edges cut when the tree is built.  build() already persists those
// dropped edges in loop_closures.yaml.  This module turns that sidecar back
// into live constraints in the browser: the joint the user drags is treated as
// prescribed, while the loop's dependent joints are solved numerically so the
// two copies of every cut hinge stay coincident and coaxial.
//
// The residual is 3-D hinge-point error + 2 effective axis-alignment errors
// (represented as the 3-vector cross product).  Rotation ABOUT the common axis
// remains free, exactly as a revolute closure requires.

const ORIENT_LEN = 0.05;       // metres: converts angular error to position scale
const FD_REV = 1e-4;           // finite-difference step, radians
const FD_PRI = 1e-5;           // finite-difference step, metres
const MAX_ITERS = 14;
const POS_TOL = 2e-6;          // metres (combined residual norm target)
const DAMPING = 1e-7;

let generation = 0;
let closureState = null;
let solving = false;
let raf = 0;
let pendingJoint = null;
let warnedNoVariables = false;

function clearState() {
  generation += 1;
  closureState = null;
  pendingJoint = null;
  warnedNoVariables = false;
  if (raf) { cancelAnimationFrame(raf); raf = 0; }
}

function rootLink(robot) {
  if (robot?.links?.base_link) { return robot.links.base_link; }
  const links = Object.values(robot?.links ?? {});
  return links.find(l => !l.parent?.isURDFJoint) ?? links[0] ?? null;
}

function localPoint(worldPoint, link) {
  return worldPoint.clone().applyMatrix4(link.matrixWorld.clone().invert());
}

function localDir(worldDir, link) {
  return worldDir.clone().transformDirection(link.matrixWorld.clone().invert()).normalize();
}

function worldPoint(local, link) {
  return local.clone().applyMatrix4(link.matrixWorld);
}

function worldDir(local, link) {
  return local.clone().transformDirection(link.matrixWorld).normalize();
}

function residualVector() {
  const out = [];
  viewer.robot.updateMatrixWorld(true);
  for (const c of closureState?.closures ?? []) {
    const pa = worldPoint(c.pointA, c.linkA);
    const pb = worldPoint(c.pointB, c.linkB);
    const dp = pa.sub(pb);
    out.push(dp.x, dp.y, dp.z);

    const aa = worldDir(c.axisA, c.linkA);
    const ab = worldDir(c.axisB, c.linkB);
    // Axis direction sign is arbitrary for a hinge.  Keep the comparison on
    // the same hemisphere so +axis and -axis describe the same physical line.
    if (aa.dot(ab) < 0) { ab.negate(); }
    const cr = aa.clone().cross(ab).multiplyScalar(ORIENT_LEN);
    out.push(cr.x, cr.y, cr.z);
  }
  return out;
}

function norm2(v) {
  let s = 0;
  for (const x of v) { s += x * x; }
  return s;
}

function clampJoint(j, q) {
  const lo = Number(j?.limit?.lower), hi = Number(j?.limit?.upper);
  if (Number.isFinite(lo)) { q = Math.max(lo, q); }
  if (Number.isFinite(hi)) { q = Math.min(hi, q); }
  return q;
}

function setJoint(j, q) {
  q = clampJoint(j, q);
  // A URDF mimic follower is normally hidden from the viewer API.  For loop
  // solving it is still a real generalized coordinate, so update it directly;
  // ordinary joints go through the viewer so the existing UI gets angle-change
  // events and its numeric fields/sliders stay synchronized.
  if (j.mimicJoint && typeof j.setJointValue === 'function') {
    j.setJointValue(q);
  } else {
    viewer.setJointValue(j.name, q);
  }
  return q;
}

function solveLinear(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let k = 0; k < n; k += 1) {
    let p = k;
    for (let i = k + 1; i < n; i += 1) {
      if (Math.abs(M[i][k]) > Math.abs(M[p][k])) { p = i; }
    }
    if (Math.abs(M[p][k]) < 1e-14) { return null; }
    [M[k], M[p]] = [M[p], M[k]];
    const d = M[k][k];
    for (let j = k; j <= n; j += 1) { M[k][j] /= d; }
    for (let i = 0; i < n; i += 1) {
      if (i === k) { continue; }
      const f = M[i][k];
      if (Math.abs(f) < 1e-18) { continue; }
      for (let j = k; j <= n; j += 1) { M[i][j] -= f * M[k][j]; }
    }
  }
  return M.map(row => row[n]);
}

function dependentJoints(changedName) {
  const names = closureState?.dependent ?? [];
  const out = [];
  for (const name of names) {
    if (name === changedName) { continue; }  // the joint the user grabbed is prescribed
    const j = viewer.robot?.joints?.[name];
    if (!j || j.jointType === 'fixed') { continue; }
    out.push(j);
  }
  return out;
}

export function enforceLoopClosures(changedName = null) {
  if (solving || !closureState?.closures?.length || !viewer.robot) { return false; }
  const vars = dependentJoints(changedName);
  if (!vars.length) {
    if (!warnedNoVariables) {
      warnedNoVariables = true;
      log('closed-loop solver: no dependent joint is available to solve the loop', 'wrn');
    }
    return false;
  }

  solving = true;
  try {
    let r = residualVector();
    let e2 = norm2(r);
    if (e2 <= POS_TOL * POS_TOL) { return true; }

    for (let iter = 0; iter < MAX_ITERS; iter += 1) {
      const m = r.length, n = vars.length;
      const J = Array.from({ length: m }, () => Array(n).fill(0));
      const q0 = vars.map(j => Number(j.angle) || 0);

      // Numerical Jacobian dr/dq around the current closed-linkage pose.
      for (let col = 0; col < n; col += 1) {
        const j = vars[col];
        const eps = j.jointType === 'prismatic' ? FD_PRI : FD_REV;
        const before = Number(j.angle) || 0;
        const qp = setJoint(j, before + eps);
        const actual = qp - before;
        const rp = residualVector();
        setJoint(j, before);
        if (Math.abs(actual) < 1e-12) { continue; }
        for (let row = 0; row < m; row += 1) {
          J[row][col] = (rp[row] - r[row]) / actual;
        }
      }

      // Damped Gauss-Newton normal equation:
      // (J^T J + lambda I) dq = -J^T r
      const A = Array.from({ length: n }, () => Array(n).fill(0));
      const b = Array(n).fill(0);
      for (let i = 0; i < n; i += 1) {
        for (let row = 0; row < m; row += 1) { b[i] -= J[row][i] * r[row]; }
        for (let j = 0; j < n; j += 1) {
          let s = 0;
          for (let row = 0; row < m; row += 1) { s += J[row][i] * J[row][j]; }
          A[i][j] = s + (i === j ? DAMPING : 0);
        }
      }
      let dq = solveLinear(A, b);
      if (!dq) { break; }

      // Do not let one Newton step jump across a four-bar branch/toggle.
      dq = dq.map((v, i) => {
        const cap = vars[i].jointType === 'prismatic' ? 0.01 : 0.20;
        return Math.max(-cap, Math.min(cap, v));
      });

      // Small line search: accept only a step that actually improves closure.
      let accepted = false;
      for (const alpha of [1.0, 0.5, 0.25, 0.1]) {
        for (let i = 0; i < n; i += 1) { setJoint(vars[i], q0[i] + alpha * dq[i]); }
        const rt = residualVector();
        const et = norm2(rt);
        if (et < e2) {
          r = rt; e2 = et; accepted = true;
          break;
        }
        for (let i = 0; i < n; i += 1) { setJoint(vars[i], q0[i]); }
      }
      if (!accepted || e2 <= POS_TOL * POS_TOL) { break; }
    }
    viewer.robot.updateMatrixWorld(true);
    viewer.redraw();
    return e2 <= 2.5e-7;  // ~0.5 mm-equivalent combined residual
  } finally {
    solving = false;
  }
}

async function initForCurrentRobot() {
  const robot = viewer.robot;
  if (!robot) { return; }
  const token = ++generation;
  try {
    const resp = await fetch('/api/loop_closures?v=' + Date.now());
    const cfg = await resp.json();
    if (token !== generation || robot !== viewer.robot) { return; }
    if (!resp.ok || cfg.error || !(cfg.closures?.length)) {
      closureState = null;
      return;
    }

    robot.updateMatrixWorld(true);
    const base = rootLink(robot);
    if (!base) { return; }
    const closures = [];
    const p = new THREE.Vector3();
    const a = new THREE.Vector3();
    for (const spec of cfg.closures) {
      const linkA = robot.links?.[spec.link_a];
      const linkB = robot.links?.[spec.link_b];
      if (!linkA || !linkB || !Array.isArray(spec.point) || !Array.isArray(spec.axis)) {
        log(`closed-loop solver: missing closure link ${spec.link_a} <-> ${spec.link_b}`, 'wrn');
        continue;
      }
      p.set(...spec.point).applyMatrix4(base.matrixWorld);
      a.set(...spec.axis).transformDirection(base.matrixWorld).normalize();
      closures.push({
        spec, linkA, linkB,
        pointA: localPoint(p, linkA), pointB: localPoint(p, linkB),
        axisA: localDir(a, linkA), axisB: localDir(a, linkB),
      });
    }
    if (!closures.length) { closureState = null; return; }
    closureState = {
      closures,
      dependent: [...new Set(cfg.dependent ?? [])],
      independent: [...new Set(cfg.independent ?? [])],
    };
    log(`closed-loop solver active: ${closures.length} closure(s), `
        + `${closureState.dependent.length} dependent joint(s)`, 'ok');
    // A pose may have been restored while the sidecar was being fetched.
    enforceLoopClosures(null);
  } catch (e) {
    if (token === generation) {
      closureState = null;
      log(`closed-loop solver unavailable: ${e.message ?? e}`, 'wrn');
    }
  }
}

// The ordinary UI (tree slider, play slider, keyboard and pose drag) all cause
// angle-change.  Solve once on the next animation frame so a burst such as
// Reset Pose does not run the nonlinear solver once per joint.
viewer.addEventListener('angle-change', e => {
  if (solving || !closureState) { return; }
  pendingJoint = e.detail || null;
  if (raf) { return; }
  raf = requestAnimationFrame(() => {
    raf = 0;
    const name = pendingJoint;
    pendingJoint = null;
    enforceLoopClosures(name);
  });
});

// A rebuilt/new URDF invalidates every stored Object3D/local anchor.  Clear at
// parse time and reconstruct from the fresh zero/rest geometry once meshes load.
viewer.addEventListener('urdf-processed', clearState);
viewer.addEventListener('geometry-loaded', () => { initForCurrentRobot(); });
