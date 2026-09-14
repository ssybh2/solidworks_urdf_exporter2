import { viewer } from './dom.js';
import { playRows, rows } from './joint-rows.js';
import { packageState, selectionState } from './state.js';

// Display CAD/Inventor Unicode names without sacrificing conservative ASCII
// identifiers in URDF/MJCF/ROS.  Inventor extraction already keeps the original
// occurrence name in /api/components -> compMeta[link].name; historically the
// editor ignored it and rendered only link_name (c_1_<hash> for Chinese names).
//
// Keep the two concepts separate:
//   internal/export id : c_1_3b63b90a   (stable, parser-friendly)
//   display label      : 大臂大孔:1       (what the CAD user authored)
//
// Rename mode still edits the internal/export id.  On double-click the existing
// inline-renamer temporarily replaces our display label with that real id, so
// this module deliberately never touches contentEditable elements.

export function displayLinkName(linkName) {
  const raw = packageState.compMeta?.[linkName]?.name;
  const text = typeof raw === 'string' ? raw.trim() : '';
  return text || linkName || '';
}

function jointLinks(j) {
  const get = tag => [...(j?.urdfNode?.children ?? [])]
    .find(el => el.tagName === tag)?.getAttribute('link') ?? '';
  return { parent: get('parent'), child: get('child') };
}

export function displayJointName(jointName) {
  const j = viewer.robot?.joints?.[jointName];
  if (!j) { return jointName || ''; }
  const { parent, child } = jointLinks(j);
  const p = displayLinkName(parent);
  const c = displayLinkName(child);
  // If CAD metadata exists, a relationship label is much more useful than a
  // generated c_* joint id.  Otherwise preserve the real URDF joint name.
  if ((p && p !== parent) || (c && c !== child)) {
    return p && c ? `${p} ↔ ${c}` : (c || p || jointName);
  }
  return jointName || '';
}

function editable(el) {
  return !el || el.isContentEditable || el.getAttribute('contenteditable') === 'true';
}

function setText(el, text) {
  if (!el || editable(el) || !text || el.textContent === text) { return; }
  el.textContent = text;
}

function relabelTreeRows() {
  for (const [jointName, rec] of rows) {
    if (!rec?.row) { continue; }
    const child = rec.child || '';
    const label = displayLinkName(child);
    const el = rec.row.querySelector('.jname');
    if (el && label) {
      setText(el, label);
      if (!editable(el)) {
        el.title = label === child
          ? (el.title || child)
          : `${label}\ninternal link: ${child}\ninternal joint: ${jointName}`;
      }
    }
  }

  // Play mode uses a separate map/panel.
  for (const [jointName, rec] of playRows) {
    const child = rec?.child || (() => {
      const j = viewer.robot?.joints?.[jointName];
      return jointLinks(j).child;
    })();
    const label = displayLinkName(child);
    const el = rec?.row?.querySelector?.('.jname');
    if (el && label) {
      setText(el, label);
      if (!editable(el) && label !== child) {
        el.title = `${label}\ninternal link: ${child}\ninternal joint: ${jointName}`;
      }
    }
  }
}

function relabelSelectionPanel() {
  const selected = selectionState.selectedLink;
  if (selected) {
    const label = displayLinkName(selected);
    const el = document.getElementById('selname');
    if (el) {
      setText(el, `🔎 ${label}`);
      el.title = label === selected ? selected : `${label}\ninternal link: ${selected}`;
    }
  }

  const panel = document.querySelector('#linkinfo .jpanel');
  if (!panel) { return; }
  for (const el of panel.querySelectorAll('.jp-rename')) {
    if (editable(el)) { continue; }
    const internal = el.dataset.old || '';
    const kind = el.dataset.kind;
    const label = kind === 'link'
      ? displayLinkName(internal)
      : kind === 'joint' ? displayJointName(internal) : internal;
    if (label) {
      setText(el, label);
      el.title = label === internal
        ? (el.title || internal)
        : `${label}\ninternal ${kind}: ${internal}\n双击可编辑导出 ID`;
    }
  }
}

function relabelHoverTip() {
  const tip = document.getElementById('hovertip');
  if (!tip || !tip.textContent || !packageState.compMeta) { return; }
  let text = tip.textContent;
  let changed = false;
  // Hover text is short; replacing exact internal IDs here is cheap and also
  // covers labels produced by modules that do not expose a row record.
  for (const [internal, meta] of Object.entries(packageState.compMeta)) {
    const raw = typeof meta?.name === 'string' ? meta.name.trim() : '';
    if (!raw || raw === internal || !text.includes(internal)) { continue; }
    text = text.split(internal).join(raw);
    changed = true;
  }
  if (changed) { tip.textContent = text; }
}

function relabelGenericExactNames() {
  // A few read-only subassembly/diagnostic rows are not registered in `rows`.
  // Only replace elements whose ENTIRE visible text equals an internal link id;
  // this avoids touching form values or data attributes used by the exporter.
  const root = document.getElementById('panel');
  if (!root || !packageState.compMeta) { return; }
  const map = packageState.compMeta;
  for (const el of root.querySelectorAll('.jname, .subasmrow .jname')) {
    if (editable(el)) { continue; }
    const internal = (el.textContent || '').trim();
    const label = displayLinkName(internal);
    if (map[internal]?.name && label !== internal) {
      setText(el, label);
      el.title = `${label}\ninternal link: ${internal}`;
    }
  }
}

let queued = false;
export function refreshUnicodeLabels() {
  if (queued) { return; }
  queued = true;
  requestAnimationFrame(() => {
    queued = false;
    relabelTreeRows();
    relabelSelectionPanel();
    relabelHoverTip();
    relabelGenericExactNames();
  });
}

// Tree/panel content is rebuilt frequently (joint type edits, selection,
// play-mode, closed-loop updates).  MutationObserver keeps Unicode labels on
// every rebuild without coupling the rendering modules to Inventor specifics.
const target = document.getElementById('app') || document.body;
new MutationObserver(refreshUnicodeLabels).observe(target, {
  childList: true, subtree: true, characterData: true,
});

// compMeta arrives asynchronously from /api/components and does not itself
// mutate a predictable DOM node.  Poll only until metadata is available, then
// leave the observer to handle subsequent UI rebuilds.
let tries = 0;
const waitMeta = setInterval(() => {
  tries += 1;
  if (Object.keys(packageState.compMeta ?? {}).length) {
    refreshUnicodeLabels();
    clearInterval(waitMeta);
  } else if (tries > 80) {
    clearInterval(waitMeta); // ~20 s: plain URDF packages legitimately have none
  }
}, 250);

viewer.addEventListener('urdf-processed', refreshUnicodeLabels);
viewer.addEventListener('geometry-loaded', refreshUnicodeLabels);
