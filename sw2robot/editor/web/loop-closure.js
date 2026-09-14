import { viewer } from './dom.js';
import { playRows, rows } from './joint-rows.js';
import { THREE } from './three-setup.js';

// URDF is a tree, so a CAD linkage with a kinematic loop has one or more
// movable edges cut when the tree is built. build() persists those dropped
// edges in loop_closures.yaml. This module turns that sidecar back into HARD
// browser constraints.
//
// Important policy:
//   * the joint the user moves is prescribed;
//   * the passive joints are solved by IK;
//   * a requested pose is committed ONLY if the loop closes;
//   * if it is unreachable, the prescribed joint is clipped back to the
//     nearest reachable boundary on the current assembly branch.
//
// Each cut revolute is represented by TWO material points on its axis. Keeping
// both point pairs coincident gives a rank-5 revolute closure: 3 translation +
// 2 axis-orientation constraints, leaving only rotation about the common axis.
// Using two witness points also makes the axial/face-contact offset explicit;
// the linkage cannot satisfy the closure by sliding one side along the axis.

const WITNESS_LEN = 0.05;      // metres between the two material axis points
const FD_REV = 1e-4;           // finite-difference step, radians
const FD_PRI = 1e-5;           // finite-difference step, metres
const MAX_ITERS = 20;
const SOLVE_TOL = 2e-5;        // 0.02 mm combined witness residual
const ACCEPT_TOL = 1e-4;       // hard accept gate: 0.10 mm combined residual
const DAMPING = 1e-7;
const CLIP_BISECT = 18;
const RANGE_BISECT = 16;
const RANGE_STEP_REV = Math.PI / 36; // 5 deg coarse reachability scan
const RANGE_STEP_PRI = 0.002;        // 2 mm coarse reachability scan
const RANGE_MAX_STEPS = 160;

let generation = 0;
let closureState = null;
let solving = false;
let raf = 0;
let pendingJoint = null;
let warnedNoVariables = false;
let scanningRange = false;

function clearState() {
  generation += 1;
  closureState = null;
  pendingJoint = null;
  warnedNoVariables = false;
  scanningRange = false;
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

function worldPoint(local, link) {
  return local.clone().applyMatrix4(link.matrixWorld);
}

function residualVector() {
  const out = [];
  viewer.robot.updateMatrixWorld(true);
  for (const c of closureState?.closures ?? []) {
    const a0 = worldPoint(c.pointA0, c.linkA);
    const b0 = worldPoint(c.pointB0, c.linkB);
    const a1 = worldPoint(c.pointA1, c.linkA);
    const b1 = worldPoint(c.pointB1, c.linkB);
    const d0 = a0.sub(b0);
    const d1 = a1.sub(b1);
    out.push(d0.x, d0.y, d0.z, d1.x, d1.y, d1.z);
  }
  return out;
}

function norm2(v) {
  let s = 0;
  for (const x of v) { s += x * x; }
  return s;
}

function staticLimit(j) {
  const saved = closureState?.staticLimits?.get(j?.name);
  if (saved) { return saved; }
  let lo = Number(j?.limit?.lower), hi = Number(j?.limit?.upper);
  if (j?.jointType === 'continuous' || !Number.isFinite(lo) || !Number.isFinite(hi)
      || (lo === 0 && hi === 0)) {
    lo = -Math.PI; hi = Math.PI;
  }
  return { lower: lo, upper: hi };
}

function clampJoint(j, q, useStatic = false) {
  const lim = useStatic ? staticLimit(j) : {
    lower: Number(j?.limit?.lower), upper: Number(j?.limit?.upper),
  };
  if (Number.isFinite(lim.lower)) { q = Math.max(lim.lower, q); }
  if (Number.isFinite(lim.upper)) { q = Math.min(lim.upper, q); }
  return q;
}

function setJoint(j, q, useStatic = false) {
  q = clampJoint(j, q, useStatic);
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

function loopJointNames() {
  return [...new Set([
    ...(closureState?.independent ?? []),
    ...(closureState?.dependent ?? []),
  ])];
}

function captureLoopPose() {
  const pose = new Map();
  for (const name of loopJointNames()) {
    const j = viewer.robot?.joints?.[name];
    if (j && j.jointType !== 'fixed') { pose.set(name, Number(j.angle) || 0); }
  }
  return pose;
}

function restoreLoopPose(pose, useStatic = false, skipName = null) {
  if (!pose) { return; }
  for (const [name, q] of pose) {
    if (name === skipName) { continue; }
    const j = viewer.robot?.joints?.[name];
    if (j && j.jointType !== 'fixed') { setJoint(j, q, useStatic); }
  }
  viewer.robot.updateMatrixWorld(true);
}

function solverJoints(changedName) {
  const names = [...(closureState?.dependent ?? [])];
  if ((closureState?.dependent ?? []).includes(changedName)) {
    names.push(...(closureState?.independent ?? []));
  }
  const out = [];
  for (const name of [...new Set(names)]) {
    if (name === changedName) { continue; }
    const j = viewer.robot?.joints?.[name];
    if (!j || j.jointType === 'fixed') { continue; }
    out.push(j);
  }
  return out;
}

function solveCurrent(vars, { useStatic = false } = {}) {
  let r = residualVector();
  let e2 = norm2(r);
  if (e2 <= SOLVE_TOL * SOLVE_TOL) { return { ok: true, e2 }; }
  if (!vars.length) { return { ok: false, e2 }; }

  for (let iter = 0; iter < MAX_ITERS; iter += 1) {
    const m = r.length, n = vars.length;
    const J = Array.from({ length: m }, () => Array(n).fill(0));
    const q0 = vars.map(j => Number(j.angle) || 0);

    for (let col = 0; col < n; col += 1) {
      const j = vars[col];
      const eps = j.jointType === 'prismatic' ? FD_PRI : FD_REV;
      const before = Number(j.angle) || 0;
      const qp = setJoint(j, before + eps, useStatic);
      const actual = qp - before;
      const rp = residualVector();
      setJoint(j, before, useStatic);
      if (Math.abs(actual) < 1e-12) { continue; }
      for (let row = 0; row < m; row += 1) {
        J[row][col] = (rp[row] - r[row]) / actual;
      }
    }

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

    dq = dq.map((v, i) => {
      const cap = vars[i].jointType === 'prismatic' ? 0.006 : 0.12;
      return Math.max(-cap, Math.min(cap, v));
    });

    let accepted = false;
    for (const alpha of [1.0, 0.5, 0.25, 0.1, 0.05]) {
      for (let i = 0; i < n; i += 1) {
        setJoint(vars[i], q0[i] + alpha * dq[i], useStatic);
      }
      const rt = residualVector();
      const et = norm2(rt);
      if (et < e2) {
        r = rt; e2 = et; accepted = true;
        break;
      }
      for (let i = 0; i < n; i += 1) { setJoint(vars[i], q0[i], useStatic); }
    }
    if (!accepted || e2 <= SOLVE_TOL * SOLVE_TOL) { break; }
  }
  return { ok: e2 <= ACCEPT_TOL * ACCEPT_TOL, e2 };
}

function solveAtPrescribed(name, q, warmPose, { useStatic = false } = {}) {
  const j = viewer.robot?.joints?.[name];
  if (!j || j.jointType === 'fixed') { return { ok: false, pose: warmPose }; }
  restoreLoopPose(warmPose, useStatic);
  const actual = setJoint(j, q, useStatic);
  const result = solveCurrent(solverJoints(name), { useStatic });
  return { ...result, q: actual, pose: result.ok ? captureLoopPose() : null };
}

function syncLimitUI(name, lo, hi) {
  const rec = rows.get(name);
  if (rec?.slider) {
    rec.slider.min = String(lo);
    rec.slider.max = String(hi);
    const q = Math.max(lo, Math.min(hi, Number(rec.joint?.angle) || 0));
    rec.slider.value = String(q);
  }
  const pr = playRows.get(name);
  if (pr?.slider) {
    pr.lo = lo; pr.hi = hi;
    pr.slider.min = String(lo);
    pr.slider.max = String(hi);
    const q = Math.max(lo, Math.min(hi, Number(pr.joint?.angle) || 0));
    pr.slider.value = String(q);
  }
}

function fmtLimit(j, q) {
  if (j?.jointType === 'prismatic') { return `${(q * 1000).toFixed(1)} mm`; }
  return `${(q * 180 / Math.PI).toFixed(1)} deg`;
}

function clipToReachable(changedName, target, acceptedPose) {
  const j = viewer.robot?.joints?.[changedName];
  const q0 = acceptedPose?.get(changedName);
  if (!j || !Number.isFinite(q0)) { return acceptedPose; }

  let low = 0.0, high = 1.0;
  let bestPose = new Map(acceptedPose);
  let bestQ = q0;
  for (let i = 0; i < CLIP_BISECT; i += 1) {
    const a = 0.5 * (low + high);
    const q = q0 + a * (target - q0);
    const trial = solveAtPrescribed(changedName, q, acceptedPose);
    if (trial.ok) {
      low = a; bestPose = trial.pose; bestQ = trial.q;
    } else {
      high = a;
    }
  }
  restoreLoopPose(bestPose);
  setJoint(j, bestQ);
  viewer.robot.updateMatrixWorld(true);
  return bestPose;
}

export function enforceLoopClosures(changedName = null) {
  if (solving || scanningRange || !closureState?.closures?.length || !viewer.robot) {
    return false;
  }

  const vars = solverJoints(changedName);
  if (!vars.length && changedName) {
    if (!warnedNoVariables) {
      warnedNoVariables = true;
      log('closed-loop solver: no dependent joint is available to solve the loop', 'wrn');
    }
    return false;
  }

  solving = true;
  try {
    if (!closureState.acceptedPose) {
      const res = solveCurrent(vars.length ? vars : solverJoints(null));
      if (!res.ok) {
        log('closed-loop solver: initial CAD pose does not satisfy the closure', 'wrn');
        return false;
      }
      closureState.acceptedPose = captureLoopPose();
      return true;
    }

    if (!changedName || !closureState.acceptedPose.has(changedName)) {
      const res = solveCurrent(vars.length ? vars : solverJoints(null));
      if (res.ok) { closureState.acceptedPose = captureLoopPose(); }
      else { restoreLoopPose(closureState.acceptedPose); }
      viewer.redraw();
      return res.ok;
    }

    const target = Number(viewer.robot.joints[changedName]?.angle) || 0;
    const baseline = new Map(closureState.acceptedPose);
    restoreLoopPose(baseline, false, changedName);
    setJoint(viewer.robot.joints[changedName], target);
    const res = solveCurrent(vars);
    if (res.ok) {
      closureState.acceptedPose = captureLoopPose();
      viewer.robot.updateMatrixWorld(true);
      viewer.redraw();
      return true;
    }

    closureState.acceptedPose = clipToReachable(changedName, target, baseline);
    const q = closureState.acceptedPose?.get(changedName);
    if (Number.isFinite(q)) {
      log(`closed-loop limit: ${changedName} clipped to ${fmtLimit(
        viewer.robot.joints[changedName], q)} (requested pose is unreachable)`, 'wrn');
    }
    viewer.redraw();
    return false;
  } finally {
    solving = false;
  }
}

function reachableBoundary(name, direction, basePose) {
  const j = viewer.robot?.joints?.[name];
  if (!j) { return null; }
  const lim = staticLimit(j);
  const hard = direction < 0 ? lim.lower : lim.upper;
  if (!Number.isFinite(hard)) { return null; }
  const step = j.jointType === 'prismatic' ? RANGE_STEP_PRI : RANGE_STEP_REV;
  let qGood = basePose.get(name);
  let poseGood = new Map(basePose);
  let qBad = null;

  for (let k = 0; k < RANGE_MAX_STEPS; k += 1) {
    let q = qGood + direction * step;
    if ((direction < 0 && q <= hard) || (direction > 0 && q >= hard)) { q = hard; }
    const trial = solveAtPrescribed(name, q, poseGood, { useStatic: true });
    if (trial.ok) {
      qGood = trial.q; poseGood = trial.pose;
      if (Math.abs(qGood - hard) < 1e-10) { return qGood; }
    } else {
      qBad = q;
      break;
    }
  }
  if (qBad === null) { return qGood; }

  let good = qGood, bad = qBad;
  let warm = poseGood;
  for (let i = 0; i < RANGE_BISECT; i += 1) {
    const q = 0.5 * (good + bad);
    const trial = solveAtPrescribed(name, q, warm, { useStatic: true });
    if (trial.ok) {
      good = trial.q; warm = trial.pose;
    } else {
      bad = q;
    }
  }
  return good;
}

async function inferReachableRanges(token) {
  if (!closureState?.acceptedPose || token !== generation || !viewer.robot) { return; }
  const drivers = [...new Set(closureState.independent ?? [])]
    .filter(name => viewer.robot?.joints?.[name]
      && viewer.robot.joints[name].jointType !== 'fixed');
  if (!drivers.length) { return; }

  scanningRange = true;
  solving = true;
  const basePose = new Map(closureState.acceptedPose);
  try {
    const inferred = [];
    for (const name of drivers) {
      if (token !== generation) { return; }
      const j = viewer.robot.joints[name];
      const s = staticLimit(j);
      const lo = reachableBoundary(name, -1, basePose);
      restoreLoopPose(basePose, true);
      const hi = reachableBoundary(name, +1, basePose);
      restoreLoopPose(basePose, true);
      if (!Number.isFinite(lo) || !Number.isFinite(hi) || hi <= lo) { continue; }
      inferred.push({ name, j, lo: Math.max(s.lower, lo), hi: Math.min(s.upper, hi), s });
      await new Promise(resolve => setTimeout(resolve, 0));
    }

    restoreLoopPose(basePose, true);
    for (const r of inferred) {
      r.j.limit.lower = r.lo;
      r.j.limit.upper = r.hi;
      syncLimitUI(r.name, r.lo, r.hi);
      const shrunk = r.lo > r.s.lower + 1e-5 || r.hi < r.s.upper - 1e-5;
      if (shrunk) {
        log(`closed-loop reachable range: ${r.name} ${fmtLimit(r.j, r.lo)} .. `
            + `${fmtLimit(r.j, r.hi)} (CAD range was ${fmtLimit(r.j, r.s.lower)} .. `
            + `${fmtLimit(r.j, r.s.upper)})`, 'ok');
      }
    }
    closureState.acceptedPose = captureLoopPose();
    viewer.redraw();
  } finally {
    restoreLoopPose(basePose);
    closureState.acceptedPose = captureLoopPose();
    solving = false;
    scanningRange = false;
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
      const p1 = p.clone().addScaledVector(a, WITNESS_LEN);
      closures.push({
        spec, linkA, linkB,
        pointA0: localPoint(p, linkA), pointB0: localPoint(p, linkB),
        pointA1: localPoint(p1, linkA), pointB1: localPoint(p1, linkB),
      });
    }
    if (!closures.length) { closureState = null; return; }

    const staticLimits = new Map();
    const allNames = [...new Set([...(cfg.dependent ?? []), ...(cfg.independent ?? [])])];
    for (const name of allNames) {
      const j = robot.joints?.[name];
      if (!j) { continue; }
      let lo = Number(j.limit?.lower), hi = Number(j.limit?.upper);
      if (j.jointType === 'continuous' || !Number.isFinite(lo) || !Number.isFinite(hi)
          || (lo === 0 && hi === 0)) {
        lo = -Math.PI; hi = Math.PI;
      }
      staticLimits.set(name, { lower: lo, upper: hi });
    }

    closureState = {
      closures,
      dependent: [...new Set(cfg.dependent ?? [])],
      independent: [...new Set(cfg.independent ?? [])],
      staticLimits,
      acceptedPose: null,
    };
    log(`closed-loop HARD solver active: ${closures.length} closure(s), `
        + `${closureState.dependent.length} passive joint(s); unreachable poses are rejected`, 'ok');

    if (enforceLoopClosures(null)) {
      setTimeout(() => { inferReachableRanges(token); }, 0);
    }
  } catch (e) {
    if (token === generation) {
      closureState = null;
      log(`closed-loop solver unavailable: ${e.message ?? e}`, 'wrn');
    }
  }
}

viewer.addEventListener('angle-change', e => {
  if (solving || scanningRange || !closureState) { return; }
  pendingJoint = e.detail || null;
  if (raf) { return; }
  raf = requestAnimationFrame(() => {
    raf = 0;
    const name = pendingJoint;
    pendingJoint = null;
    enforceLoopClosures(name);
  });
});

viewer.addEventListener('urdf-processed', clearState);
viewer.addEventListener('geometry-loaded', () => { initForCurrentRobot(); });
