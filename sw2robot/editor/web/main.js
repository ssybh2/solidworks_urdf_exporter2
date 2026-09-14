import { extractFlow } from './export-box.js';
import { loadRobot } from './load.js';
import './state.js';
import './dom.js';
import './three-setup.js';
import './bootstrap.js';
import './session-log.js';
import './capture-progress.js';
import './joint-rows.js';
import './subassembly-choices.js';
import './subassembly-preview.js';
import './lists.js';
import './mass-editor.js';
import './tree.js';
import './play-mode.js';
import './axis-markers.js';
import './frames.js';
import './link-look.js';
import './mass-heatmap.js';
import './selection.js';
import './link-info.js';
import './bulk-edit.js';
import './joint-type.js';
import './export-names.js';
import './coacd-preview.js';
import './camera-reroot.js';
import './face-pick.js';
import './endcoords-gizmo.js';
import './box-select.js';
import './mimic.js';
import './batch-look.js';
import { refreshHistory } from './root-frame.js';
import './file-browser.js';
import './diagnostics.js';
import './keycast.js';
import './pose-reset.js';
import './loop-closure.js';
import './unicode-labels.js';

// ---- Motor / Passive -----------------------------------------------------
// The exporter already understands joints.yaml:actuated_joints. Surface that
// setting directly in the focused joint panel without changing joint type or
// closed-loop dependent/independent solver classification.
const ACT_STYLE_ID = 'sw2robot-actuation-style';

function ensureActuationStyle() {
  if (document.getElementById(ACT_STYLE_ID)) { return; }
  const st = document.createElement('style');
  st.id = ACT_STYLE_ID;
  st.textContent = `
    .jp-actuation-ui { align-items:center; gap:8px; margin-top:5px; }
    .jp-actuation-ui .act-seg {
      display:inline-flex; border:1px solid #414852; border-radius:6px;
      overflow:hidden; background:#15171a; min-width:150px;
    }
    .jp-actuation-ui .act-choice {
      appearance:none; border:0; border-right:1px solid #414852;
      background:#15171a; color:#9ca3ad; padding:4px 11px; cursor:pointer;
      font-size:11px; line-height:1.3; flex:1;
    }
    .jp-actuation-ui .act-choice:last-child { border-right:0; }
    .jp-actuation-ui .act-choice:hover:not(:disabled) { color:#fff; background:#242932; }
    .jp-actuation-ui .act-choice.active.motor {
      color:#ecfff7; background:#176245; box-shadow:inset 0 0 0 1px #2f9b70;
    }
    .jp-actuation-ui .act-choice.active.passive {
      color:#f0f2f4; background:#4a5059; box-shadow:inset 0 0 0 1px #777f8a;
    }
    .jp-actuation-ui .act-choice:disabled { cursor:default; opacity:.62; }
    .jp-actuation-ui .act-mode {
      color:#89919c; font-size:10px; white-space:nowrap;
    }
    .jp-actuation-ui .act-mode.custom { color:#e8c468; }
    .jp-actuation-ui .act-mode.error { color:#ff7777; }
    .mjcf-startup-actuation {
      display:flex; align-items:center; gap:4px; color:#8a93a3; cursor:pointer;
      font-size:11px; padding:1px 3px; border-radius:4px;
    }
    .mjcf-startup-actuation:hover { color:#cdd7e6; background:#1b2027; }
    .mjcf-startup-actuation .act-start-state {
      color:#6f7a88; font-size:10px; margin-left:2px;
    }
    .mjcf-startup-actuation .act-start-state.on { color:#e8c468; }
    .mjcf-startup-actuation .act-start-state.err { color:#ff7777; }
  `;
  document.head.appendChild(st);
}

function actuationJointName(panel) {
  // There are two .jp-name elements (link then joint). Pick the joint one.
  const name = panel.querySelector('.jp-name[data-kind="joint"]');
  return (name?.dataset?.old || name?.textContent || '').trim();
}

function actuationMovable(panel) {
  const type = panel.querySelector('.jp-type')?.value || '';
  return !['fixed', 'mass_only', 'floating', 'planar'].includes(type);
}

function actuationModeLabel(mode) {
  if (mode === 'configured') { return 'Custom'; }
  if (mode === 'ACT_*') { return 'ACT_*'; }
  return 'Auto';
}

function applyActuationState(row, payload, joint) {
  const isMotor = (payload.actuated || []).includes(joint);
  const motor = row.querySelector('[data-act="motor"]');
  const passive = row.querySelector('[data-act="passive"]');
  const mode = row.querySelector('.act-mode');
  motor.classList.toggle('active', isMotor);
  passive.classList.toggle('active', !isMotor);
  mode.textContent = actuationModeLabel(payload.mode);
  mode.className = 'act-mode' + (payload.mode === 'configured' ? ' custom' : '');
  row.dataset.motor = isMotor ? '1' : '0';
}

async function fetchActuation(url, options = {}, timeoutMs = 5000) {
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), timeoutMs);
  try {
    const resp = await fetch(url, { ...options, signal: ctl.signal });
    const text = await resp.text();
    let payload = {};
    try { payload = text ? JSON.parse(text) : {}; }
    catch { throw new Error(`invalid API response (${resp.status})`); }
    if (!resp.ok || payload.error) {
      throw new Error(payload.error ?? `HTTP ${resp.status}`);
    }
    return payload;
  } catch (e) {
    if (e?.name === 'AbortError') {
      throw new Error('actuation API timeout');
    }
    throw e;
  } finally {
    clearTimeout(timer);
  }
}

function wireActuationRow(panel) {
  if (!panel || panel.querySelector('.jp-actuation-ui') || !actuationMovable(panel)) {
    return;
  }
  const joint = actuationJointName(panel);
  if (!joint) { return; }
  ensureActuationStyle();

  const row = document.createElement('div');
  row.className = 'jp-row jp-actuation-ui';
  row.innerHTML =
    '<span class="jp-lbl" title="MuJoCo actuator selection">Drive</span>' +
    '<span class="act-seg">' +
      '<button type="button" class="act-choice motor" data-act="motor" ' +
        'title="Motor: export this joint with a MuJoCo actuator">Motor</button>' +
      '<button type="button" class="act-choice passive" data-act="passive" ' +
        'title="Passive: keep the movable joint but export no actuator">Passive</button>' +
    '</span>' +
    '<span class="act-mode">checking</span>';
  const typeRow = panel.querySelector('.jp-type')?.closest('.jp-row');
  if (typeRow) { typeRow.insertAdjacentElement('afterend', row); }
  else { panel.appendChild(row); }

  const buttons = [...row.querySelectorAll('.act-choice')];

  // IMPORTANT: attach click handlers BEFORE asking the backend for status and
  // leave the buttons enabled. Status discovery is advisory; it must never make
  // the user wait or trap the UI in a disabled/cursor-wait state.
  for (const button of buttons) {
    button.addEventListener('click', async () => {
      const wantMotor = button.dataset.act === 'motor';
      if (row.dataset.motor != null
          && (row.dataset.motor === '1') === wantMotor) { return; }
      buttons.forEach(b => { b.disabled = true; });
      const mode = row.querySelector('.act-mode');
      mode.textContent = 'saving…';
      mode.className = 'act-mode';
      try {
        const payload = await fetchActuation('/api/set_actuated', {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ joint, motor: wantMotor }),
        });
        if (!row.isConnected) { return; }
        applyActuationState(row, payload, joint);
        refreshHistory();
        if (typeof log === 'function') {
          log(`${joint}: ${wantMotor ? 'Motor' : 'Passive'} ✓`, 'ok');
        }
      } catch (e) {
        if (!row.isConnected) { return; }
        mode.textContent = 'save failed';
        mode.className = 'act-mode error';
        row.title = e.message ?? String(e);
        if (typeof log === 'function') {
          log(`Motor/Passive update failed: ${e.message ?? e}`, 'err');
        }
      } finally {
        if (row.isConnected) { buttons.forEach(b => { b.disabled = false; }); }
      }
    });
  }

  // Discover the effective state in the background. Never disable the controls
  // while this GET is pending: a slow/missing backend must not block editing.
  fetchActuation('/api/actuation?v=' + Date.now())
    .then(payload => {
      if (!row.isConnected || actuationJointName(panel) !== joint) { return; }
      if (!payload.supported || !(payload.movable || []).includes(joint)) {
        row.remove();
        return;
      }
      applyActuationState(row, payload, joint);
    })
    .catch(e => {
      if (!row.isConnected) { return; }
      const mode = row.querySelector('.act-mode');
      mode.textContent = 'API error';
      mode.className = 'act-mode error';
      row.title = e.message ?? String(e);
      if (typeof log === 'function') {
        log(`Motor/Passive status failed: ${e.message ?? e}`, 'err');
      }
    });
}

function scanActuationPanel() {
  const panel = document.querySelector('#linkinfo .jpanel');
  if (panel) { wireActuationRow(panel); }
}

const linkInfoForActuation = document.getElementById('linkinfo');
if (linkInfoForActuation) {
  new MutationObserver(() => queueMicrotask(scanActuationPanel)).observe(
    linkInfoForActuation, { childList: true, subtree: true });
  queueMicrotask(scanActuationPanel);
}

// ---- MuJoCo startup actuation -------------------------------------------
// Motor/Passive describes the robot hardware interface. This separate export
// option describes whether those already-present actuators are energised as soon
// as MuJoCo loads the model. OFF is the safe validation default: motors remain in
// <actuator> (nu is unchanged), but MuJoCo globally disables actuator forces.
let startupActuationRefresh = 0;

function setStartupActuationUi(payload) {
  const row = document.getElementById('mjcf-startup-actuation');
  const box = document.getElementById('expactuationstartup');
  const state = row?.querySelector('.act-start-state');
  if (!row || !box || !state) { return; }
  if (!payload?.supported) {
    row.style.display = 'none';
    return;
  }
  row.style.display = 'flex';
  const enabled = !!payload.mujoco_actuation_enabled;
  box.checked = enabled;
  state.textContent = enabled ? 'ON at startup' : 'OFF at startup';
  state.className = 'act-start-state' + (enabled ? ' on' : '');
  row.title = enabled
    ? 'Motor actuators will apply force immediately when MuJoCo starts.'
    : 'Motor interfaces stay in the MJCF, but actuator forces start disabled. The robot can fall freely under gravity.';
}

async function refreshStartupActuationUi() {
  const seq = ++startupActuationRefresh;
  const row = document.getElementById('mjcf-startup-actuation');
  if (!row) { return; }
  try {
    const payload = await fetchActuation('/api/actuation?v=' + Date.now());
    if (seq !== startupActuationRefresh) { return; }
    setStartupActuationUi(payload);
  } catch (e) {
    if (seq !== startupActuationRefresh) { return; }
    const state = row.querySelector('.act-start-state');
    if (state) {
      state.textContent = 'API error';
      state.className = 'act-start-state err';
    }
    row.title = e.message ?? String(e);
  }
}

function ensureStartupActuationUi() {
  if (document.getElementById('mjcf-startup-actuation')) { return; }
  const fixedBase = document.getElementById('expfixedbase');
  const fixedLabel = fixedBase?.closest('label');
  if (!fixedLabel) { return; }
  ensureActuationStyle();

  const row = document.createElement('label');
  row.id = 'mjcf-startup-actuation';
  row.className = 'mjcf-startup-actuation';
  row.innerHTML =
    '<input id="expactuationstartup" type="checkbox">' +
    '<span>Enable actuation on startup</span>' +
    '<span class="act-start-state">checking</span>';
  fixedLabel.insertAdjacentElement('afterend', row);

  const box = row.querySelector('#expactuationstartup');
  box.addEventListener('change', async () => {
    const wanted = box.checked;
    const state = row.querySelector('.act-start-state');
    box.disabled = true;
    state.textContent = 'saving…';
    state.className = 'act-start-state';
    try {
      const payload = await fetchActuation('/api/set_actuation_startup', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: wanted }),
      });
      setStartupActuationUi(payload);
      refreshHistory();
      if (typeof log === 'function') {
        log(`MuJoCo actuation on startup: ${wanted ? 'enabled' : 'disabled'} ✓`, 'ok');
      }
    } catch (e) {
      box.checked = !wanted;
      state.textContent = 'save failed';
      state.className = 'act-start-state err';
      row.title = e.message ?? String(e);
      if (typeof log === 'function') {
        log(`MuJoCo startup actuation update failed: ${e.message ?? e}`, 'err');
      }
    } finally {
      box.disabled = false;
    }
  });

  refreshStartupActuationUi();
}

ensureStartupActuationUi();
const titleForActuation = document.getElementById('title');
if (titleForActuation) {
  new MutationObserver(() => refreshStartupActuationUi()).observe(
    titleForActuation, { childList: true, characterData: true, subtree: true });
}

// ---- kick off -----------------------------------------------------------
// live SolidWorks session: only attachable when THIS SERVER was started
// from the user's own terminal (same login session); otherwise we show why
let _swStatus = null;
export function renderSwStatus() {
  const st = _swStatus;
  if (!st) { return; }
  const el = document.getElementById('swstat');
  const btn = document.getElementById('useactive');
  if (st.active_assembly) {
    btn.style.display = '';
    btn.title = st.active_assembly;
    // the path/filename is a real on-disk identifier -- never translated
    el.textContent = (st.dirty ? t('sw.unsaved') : '') +
      t('sw.open', { name: st.active_assembly.split(/[\\/]/).pop() });
  } else if (st.running && !st.attachable) {
    el.textContent = t('sw.runningNotVisible');
  } else if (!st.running) {
    el.textContent = t('sw.notRunning');
  }
}
(async () => {
  try {
    _swStatus = await (await fetch('/api/swstatus')).json();
    renderSwStatus();
    if (_swStatus.active_assembly) {
      document.getElementById('useactive').addEventListener('click', () => {
        if (_swStatus.dirty) { log(t('sw.unsavedLog'), 'wrn'); }
        extractFlow(_swStatus.active_assembly);
      });
    }
  } catch { /* status is best-effort */ }
})();

const info = await (await fetch('/api/info')).json();
if (info.urdf) { loadRobot(info); }
else {
  statusEl.textContent = t('start.pick');
  document.getElementById('emptyprompt').style.display = 'block';
  log(t('start.noPkg'), 'wrn');
}