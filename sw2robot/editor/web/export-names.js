import {
  collPreviewAbort, collPreviewClearOverlay, collPreviewPoll,
  setCollisionShown,
} from './coacd-preview.js';
import {
  collModeSel, cqualitySel, expmeshdir, exppkg, exprobot, expurdf,
} from './dom.js';
import { collisionState, packageState } from './state.js';
// ---- export names: thread the chosen package + URDF + robot names into the
// ZIP links.  The ROS1/ROS2/glb downloads are plain <a download> links; on click
// we rewrite their ?name=&urdf=&robotname= from these fields so the exported
// package (and zip) carry them.  The names cascade when left empty: URDF stem ->
// package name, <robot name> -> URDF stem.
function exportDefaultName() {
  // mirror the server: sanitise the assembly name to a valid ROS package name
  // ('Assem1' -> 'assem1_description') so the placeholder matches what ships
  const n = packageState.currentInfo?.name;
  if (!n) { return 'robot_description'; }
  let base = n.toLowerCase().replace(/[^a-z0-9_]/g, '_').replace(/^_+|_+$/g, '');
  if (!base || !/[a-z]/.test(base[0])) { base = base ? 'robot_' + base : 'robot'; }
  return `${base}_description`;
}
function effectivePkgName() {
  return (exppkg?.value || '').trim() || exportDefaultName();
}
// the URDF stem defaults to the package name, and the <robot name> inside the
// URDF to that stem -- so renaming the package renames all three
function effectiveUrdfName() {
  return (expurdf?.value || '').trim() || effectivePkgName();
}
function refreshNamePlaceholders() {
  if (expurdf) { expurdf.placeholder = effectivePkgName(); }  // urdf = pkg name
  if (exprobot) { exprobot.placeholder = effectiveUrdfName(); }  // robot = urdf
}
export function refreshExportName() {
  if (exppkg) { exppkg.placeholder = exportDefaultName(); }
  refreshNamePlaceholders();
}
// keep the package field a valid ROS package name as the user types (lowercase,
// digits, underscore; must start with a letter) -- matches the server check
exppkg?.addEventListener('input', () => {
  const clean = exppkg.value.toLowerCase()
    .replace(/[^a-z0-9_]/g, '_').replace(/^[^a-z]+/, '');
  if (exppkg.value !== clean) { exppkg.value = clean; }
  refreshNamePlaceholders();
});
// the URDF stem is a filename: letters/digits/_/-/. , starting alphanumeric
expurdf?.addEventListener('input', () => {
  const clean = expurdf.value
    .replace(/[^A-Za-z0-9_.-]/g, '_').replace(/^[^A-Za-z0-9]+/, '');
  if (expurdf.value !== clean) { expurdf.value = clean; }
  refreshNamePlaceholders();
});
// the <robot name> takes the same character set as the URDF stem
exprobot?.addEventListener('input', () => {
  const clean = exprobot.value
    .replace(/[^A-Za-z0-9_.-]/g, '_').replace(/^[^A-Za-z0-9]+/, '');
  if (exprobot.value !== clean) { exprobot.value = clean; }
});
// the mesh dir is a package-relative path: letters/digits/_/-/. segments joined
// by '/'; drop other chars and any leading slash (must stay inside the package)
expmeshdir?.addEventListener('input', () => {
  const clean = expmeshdir.value
    .replace(/[^A-Za-z0-9_./-]/g, '_').replace(/^\/+/, '');
  if (expmeshdir.value !== clean) { expmeshdir.value = clean; }
});
// building a ROS package converts every mesh (3dxml -> dae/stl) and can take a
// few seconds, during which a plain <a download> gives NO feedback.  Drive it
// over fetch instead: show an "exporting ..." loadbar, then hand the finished
// blob to a throwaway download link.
// Collision geometry for the ROS packages: 'copy' (visual mesh), 'hull'  or '
// coacd' (convex decomposition into many parts).
const collGenRow = document.getElementById('collgenrow');

// quality only applies to CoACD; the Generate/preview row only to hull+coacd
export function updateCollUI() {
  const mode = collModeSel?.value || 'copy';
  if (cqualitySel) { cqualitySel.style.display = mode === 'coacd' ? '' : 'none'; }
  if (collGenRow) { collGenRow.style.display = mode === 'copy' ? 'none' : 'flex'; }
}
collModeSel?.addEventListener('change', () => {
  updateCollUI();
  // a generate job still in flight belongs to the PREVIOUS mode -- abandon it,
  // else its completion would finalize against the old mode and silently revert
  // this deliberate switch (re-applying the old parts to live + export).
  if (collPreviewPoll) { collPreviewAbort(); }
  // any generated preview belongs to the PREVIOUS mode, so switching type drops
  // it: nothing is shown until a preview is made for the newly-selected type
  // (copy has none; hull/coacd must be (re)generated).
  collPreviewClearOverlay();
  collisionState.collPreviewFinalized = false;
  setCollisionShown(false);
});
updateCollUI();

// ---- MuJoCo spawn height -------------------------------------------------
// Empty = Auto: the exporter copies its mesh-derived safe home height into the
// floating base's DEFAULT pose.  A number is an absolute world-Z in metres and
// also replaces the home keyframe Z, so launch_from_path and Reset/Home agree.
let spawnRefreshSeq = 0;

function ensureSpawnStyle() {
  if (document.getElementById('sw2robot-spawn-height-style')) { return; }
  const st = document.createElement('style');
  st.id = 'sw2robot-spawn-height-style';
  st.textContent = `
    .mjcf-spawn-height {
      display:flex; align-items:center; gap:4px; color:#8a93a3;
      font-size:11px; padding:1px 3px; border-radius:4px;
    }
    .mjcf-spawn-height input {
      width:74px; box-sizing:border-box; background:#15171a; color:#d7dde6;
      border:1px solid #414852; border-radius:4px; padding:2px 5px;
      font-size:11px;
    }
    .mjcf-spawn-height input:focus { border-color:#6b7b90; outline:none; }
    .mjcf-spawn-height button {
      border:1px solid #414852; background:#1b2027; color:#aab3c0;
      border-radius:4px; padding:2px 6px; cursor:pointer; font-size:10px;
    }
    .mjcf-spawn-height button:hover:not(:disabled) { color:#fff; background:#262d36; }
    .mjcf-spawn-height .spawn-state { color:#6f7a88; font-size:10px; }
    .mjcf-spawn-height .spawn-state.custom { color:#e8c468; }
    .mjcf-spawn-height .spawn-state.err { color:#ff7777; }
    .mjcf-spawn-height.disabled { opacity:.55; }
  `;
  document.head.appendChild(st);
}

async function spawnApi(url, options = {}) {
  const resp = await fetch(url, options);
  const text = await resp.text();
  let payload = {};
  try { payload = text ? JSON.parse(text) : {}; }
  catch { throw new Error(`invalid API response (${resp.status})`); }
  if (!resp.ok || payload.error) {
    throw new Error(payload.error ?? `HTTP ${resp.status}`);
  }
  return payload;
}

function syncSpawnFixedBase() {
  const row = document.getElementById('mjcf-spawn-height');
  const input = document.getElementById('expspawnheight');
  const auto = document.getElementById('expspawnauto');
  const state = row?.querySelector('.spawn-state');
  const fixed = !!document.getElementById('expfixedbase')?.checked;
  if (!row || !input || !auto) { return; }
  input.disabled = fixed;
  auto.disabled = fixed;
  row.classList.toggle('disabled', fixed);
  if (fixed && state) { state.textContent = 'fixed base'; }
}

function applySpawnPayload(payload) {
  const row = document.getElementById('mjcf-spawn-height');
  const input = document.getElementById('expspawnheight');
  const state = row?.querySelector('.spawn-state');
  if (!row || !input || !state) { return; }
  if (!payload?.supported) {
    row.style.display = 'none';
    return;
  }
  row.style.display = 'flex';
  const h = payload.mujoco_spawn_height;
  if (h == null) {
    input.value = '';
    state.textContent = 'Auto (mesh clearance)';
    state.className = 'spawn-state';
  } else {
    input.value = String(h);
    state.textContent = `${Number(h).toFixed(3)} m`;
    state.className = 'spawn-state custom';
  }
  syncSpawnFixedBase();
}

async function refreshSpawnHeight() {
  const seq = ++spawnRefreshSeq;
  const row = document.getElementById('mjcf-spawn-height');
  if (!row) { return; }
  try {
    const payload = await spawnApi('/api/actuation?v=' + Date.now());
    if (seq !== spawnRefreshSeq) { return; }
    applySpawnPayload(payload);
  } catch (e) {
    if (seq !== spawnRefreshSeq) { return; }
    const state = row.querySelector('.spawn-state');
    if (state) {
      state.textContent = 'API error';
      state.className = 'spawn-state err';
    }
    row.title = e.message ?? String(e);
  }
}

async function saveSpawnHeight(height) {
  const row = document.getElementById('mjcf-spawn-height');
  const input = document.getElementById('expspawnheight');
  const auto = document.getElementById('expspawnauto');
  const state = row?.querySelector('.spawn-state');
  if (!row || !input || !auto || !state) { return; }
  input.disabled = true;
  auto.disabled = true;
  state.textContent = 'saving…';
  state.className = 'spawn-state';
  try {
    const payload = await spawnApi('/api/set_spawn_height', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ height }),
    });
    applySpawnPayload(payload);
    if (typeof log === 'function') {
      log(`MuJoCo spawn height: ${height == null ? 'Auto' : height + ' m'} ✓`, 'ok');
    }
  } catch (e) {
    state.textContent = 'save failed';
    state.className = 'spawn-state err';
    row.title = e.message ?? String(e);
    if (typeof log === 'function') {
      log(`MuJoCo spawn height update failed: ${e.message ?? e}`, 'err');
    }
  } finally {
    syncSpawnFixedBase();
  }
}

function ensureSpawnHeightUi() {
  if (document.getElementById('mjcf-spawn-height')) { return; }
  const fixedBase = document.getElementById('expfixedbase');
  const fixedLabel = fixedBase?.closest('label');
  if (!fixedLabel) { return; }
  ensureSpawnStyle();

  const row = document.createElement('div');
  row.id = 'mjcf-spawn-height';
  row.className = 'mjcf-spawn-height';
  row.title = 'Initial floating-base world Z in metres. Leave blank for Auto: use the mesh-derived safe clearance height.';
  row.innerHTML =
    '<span>Initial spawn Z</span>' +
    '<input id="expspawnheight" type="number" min="0" step="0.01" placeholder="Auto">' +
    '<span>m</span>' +
    '<button id="expspawnauto" type="button">Auto</button>' +
    '<span class="spawn-state">checking</span>';
  fixedLabel.insertAdjacentElement('afterend', row);

  const input = row.querySelector('#expspawnheight');
  const auto = row.querySelector('#expspawnauto');
  input.addEventListener('keydown', e => {
    if (e.key === 'Enter') { e.preventDefault(); input.blur(); }
  });
  input.addEventListener('change', () => {
    const raw = input.value.trim();
    if (!raw) { saveSpawnHeight(null); return; }
    const value = Number(raw);
    if (!Number.isFinite(value) || value < 0) {
      const state = row.querySelector('.spawn-state');
      state.textContent = '>= 0 m required';
      state.className = 'spawn-state err';
      return;
    }
    saveSpawnHeight(value);
  });
  auto.addEventListener('click', () => {
    input.value = '';
    saveSpawnHeight(null);
  });
  fixedBase.addEventListener('change', syncSpawnFixedBase);

  refreshSpawnHeight();
}

ensureSpawnHeightUi();
const titleForSpawnHeight = document.getElementById('title');
if (titleForSpawnHeight) {
  new MutationObserver(() => refreshSpawnHeight()).observe(
    titleForSpawnHeight, { childList: true, characterData: true, subtree: true });
}
