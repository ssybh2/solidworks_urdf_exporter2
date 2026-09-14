import { viewer } from './dom.js';
import { playRows, rows } from './joint-rows.js';
import { op } from './session-log.js';
import { packageState, selectionState } from './state.js';

// Exact "as imported" pose snapshot.
//
// The historical Reset pose button simply wrote 0 into every movable joint.
// That is not equivalent to the CAD import pose once a closed-loop solver has
// solved passive joints (and it is not guaranteed for imported URDFs either).
// Capture the first fully-loaded pose of each opened package and restore those
// generalized coordinates verbatim.
let importPose = null;
let importKey = null;
let capturedRobot = null;

function packageKey() {
  const i = packageState.currentInfo;
  if (!i) { return '(drop-or-preview)'; }
  return `${i.name ?? ''}|${i.urdf ?? ''}`;
}

function capturePose(robot) {
  const out = {};
  for (const [name, j] of Object.entries(robot?.joints ?? {})) {
    if (j.jointType !== 'fixed') { out[name] = Number(j.angle) || 0; }
  }
  return out;
}

function setJointDirect(j, q) {
  // Deliberately bypass viewer.setJointValue(): that wrapper emits an
  // angle-change for every joint. Reset is an atomic restore, and the closed
  // loop solver must see the completed CAD pose rather than N half-restored
  // intermediate poses.
  if (typeof j?.setJointValue === 'function') {
    j.setJointValue(Number(q) || 0);
    return true;
  }
  return false;
}

function refreshVisibleJointControls() {
  for (const [name, rec] of rows) {
    const j = viewer.robot?.joints?.[name];
    if (!j) { continue; }
    const q = Number(j.angle) || 0;
    if (rec.slider) { rec.slider.value = String(q); }
    if (rec.val) {
      rec.val.textContent = rec.fmtDisp ? rec.fmtDisp(q) : String(q);
    }
  }
  for (const [name, rec] of playRows) {
    const j = viewer.robot?.joints?.[name];
    if (!j) { continue; }
    const q = Number(j.angle) || 0;
    if (rec.slider) { rec.slider.value = String(q); }
    if (rec.val) { rec.val.value = rec.fmt ? rec.fmt(q) : String(q); }
  }
  const sync = selectionState.jpSync;
  if (sync?.name && typeof sync.set === 'function') {
    const j = viewer.robot?.joints?.[sync.name];
    if (j) { sync.set(Number(j.angle) || 0); }
  }
}

function captureImportPose() {
  const robot = viewer.robot;
  if (!robot) { return; }
  const key = packageKey();

  // geometry-loaded also fires after edit/re-root rebuilds. Do not overwrite
  // the CAD reference pose during those rebuilds. A different package (or a
  // full page reload) gets a new snapshot.
  if (importPose && key === importKey) { return; }
  importPose = capturePose(robot);
  importKey = key;
  capturedRobot = robot;
  const n = Object.keys(importPose).length;
  log(`import pose captured: ${n} movable joint coordinate(s)`, 'ok');
}

export function restoreImportPose() {
  const robot = viewer.robot;
  if (!robot) { return false; }

  // If this is a robot we have not seen (e.g. a dropped URDF), make its current
  // pose the reference rather than falling back to the old package snapshot.
  if (!importPose || (capturedRobot !== robot && packageKey() !== importKey)) {
    importPose = capturePose(robot);
    importKey = packageKey();
    capturedRobot = robot;
  }

  let n = 0;
  for (const [name, q] of Object.entries(importPose)) {
    const j = robot.joints?.[name];
    if (!j || j.jointType === 'fixed') { continue; }
    if (setJointDirect(j, q)) { n += 1; }
  }
  robot.updateMatrixWorld(true);
  refreshVisibleJointControls();
  viewer.redraw();

  // This is an atomic restore. The dedicated event is useful to other tools;
  // one synthetic angle-change with null detail then lets the closed-loop
  // solver validate/synchronize its accepted branch ONCE, after every joint is
  // already back at the imported pose. No intermediate half-restored pose is
  // ever presented to IK.
  viewer.dispatchEvent(new CustomEvent('import-pose-restored', {
    detail: { angles: { ...importPose }, count: n },
  }));
  viewer.dispatchEvent(new CustomEvent('angle-change', { detail: null }));

  op('resetPose', { n, mode: 'import' });
  log(`pose restored to CAD import snapshot (${n} joints)`, 'ok');
  return n > 0;
}

viewer.addEventListener('geometry-loaded', captureImportPose);

const resetBtn = document.getElementById('reset');
if (resetBtn) {
  resetBtn.textContent = 'Reset import pose';
  resetBtn.title = 'Restore the exact joint pose captured when this package was first loaded';
  // Capture phase + stopImmediatePropagation replaces load.js's legacy
  // "set every joint to zero" click handler without perturbing that module's
  // resetView/camera logic.
  resetBtn.addEventListener('click', ev => {
    ev.preventDefault();
    ev.stopImmediatePropagation();
    restoreImportPose();
  }, { capture: true });
}

// If the viewer is explicitly cleared, the next package must establish a fresh
// reference even when it happens to reuse the same package name.
document.getElementById('clearview')?.addEventListener('click', () => {
  importPose = null;
  importKey = null;
  capturedRobot = null;
}, { capture: true });
