"""Low-level Autodesk Inventor COM helpers."""
from __future__ import annotations

import hashlib
import math
import os

try:
    import pythoncom
    import win32com.client
    HAVE_WIN32 = True
except ImportError:  # non-Windows / editor-only installs
    pythoncom = None
    win32com = None
    HAVE_WIN32 = False

from ..model import safe_name
from ..state import CoordinateSystemState

CM_TO_M = 0.01
KG_CM2_TO_KG_M2 = 1e-4
FILE_BROWSE_IO = 13059
STL_TRANSLATOR = "{533E9A98-FC3B-11D4-8E7E-0010B541CD80}"


class InventorUnavailable(RuntimeError):
    """Inventor could not be reached through its COM automation API."""


def safe_prop(obj, name, default=None):
    try:
        return getattr(obj, name)
    except Exception:
        return default


def iter_collection(coll):
    if coll is None:
        return
    try:
        n = int(coll.Count)
    except Exception:
        return
    for i in range(1, n + 1):
        try:
            yield coll.Item(i)
        except Exception:
            pass


def point_m(p):
    try:
        return [float(p.X) * CM_TO_M, float(p.Y) * CM_TO_M, float(p.Z) * CM_TO_M]
    except Exception:
        return None


def vec3(v, normalize=False):
    try:
        out = [float(v.X), float(v.Y), float(v.Z)]
    except Exception:
        return None
    if normalize:
        n = math.sqrt(sum(x * x for x in out))
        if n < 1e-12:
            return None
        out = [x / n for x in out]
    return out


def matrix_si(m):
    """Inventor Matrix -> row-major local-to-parent transform in SI metres."""
    vals = []
    for r in range(1, 5):
        for c in range(1, 5):
            try:
                vals.append(float(m.Cell(r, c)))
            except Exception:
                vals.append(1.0 if r == c else 0.0)
    for i in (3, 7, 11):
        vals[i] *= CM_TO_M
    return vals


def occ_name(occ):
    return str(safe_prop(occ, "Name", safe_prop(occ, "Name2", "")) or "")


def occ_doc(occ):
    doc = safe_prop(safe_prop(occ, "Definition"), "Document")
    if doc is not None:
        return doc
    return safe_prop(safe_prop(occ, "ReferencedDocumentDescriptor"),
                     "ReferencedDocument")


def doc_path(doc):
    p = safe_prop(doc, "FullFileName")
    return os.path.abspath(str(p)) if p else None


def _byref_double():
    return win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_R8, 0.0)


def xyz_inertia(mp):
    """(ixx,ixy,ixz,iyy,iyz,izz) about COM in kg*m^2."""
    try:
        refs = [_byref_double() for _ in range(6)]
        mp.XYZMomentsOfInertia(*refs)
        ixx, iyy, izz, ixy, iyz, ixz = [float(x.value) for x in refs]
    except Exception:
        try:
            out = mp.XYZMomentsOfInertia(0., 0., 0., 0., 0., 0.)
            ixx, iyy, izz, ixy, iyz, ixz = [float(x) for x in out[:6]]
        except Exception:
            return None
    s = KG_CM2_TO_KG_M2
    return [ixx*s, ixy*s, ixz*s, iyy*s, iyz*s, izz*s]


def mass_props(definition):
    mp = safe_prop(definition, "MassProperties")
    if mp is None:
        return None, None, None, False
    try:
        mp.CacheResultsOnCompute = False
    except Exception:
        pass
    try:
        mass = float(mp.Mass)
        if not math.isfinite(mass) or mass <= 0:
            mass = None
    except Exception:
        mass = None
    return (mass, point_m(safe_prop(mp, "CenterOfMass")), xyz_inertia(mp),
            bool(safe_prop(mp, "MassOverridden", False)))


def ucs_states(definition):
    out = []
    for ucs in iter_collection(safe_prop(definition, "UserCoordinateSystems")):
        try:
            out.append(CoordinateSystemState(
                name=str(ucs.Name), document_from_frame=matrix_si(ucs.Transformation)))
        except Exception:
            pass
    return out


def geometry_direction(obj):
    seen = set()
    for _ in range(4):
        if obj is None or id(obj) in seen:
            break
        seen.add(id(obj))
        for attr in ("Direction", "AxisVector", "Normal"):
            d = vec3(safe_prop(obj, attr), normalize=True)
            if d is not None:
                return d
        nxt = safe_prop(obj, "Geometry")
        if nxt is None or nxt is obj:
            break
        obj = nxt
    return None


def geometry_point(obj):
    seen = set()
    for _ in range(4):
        if obj is None or id(obj) in seen:
            break
        seen.add(id(obj))
        for attr in ("Center", "BasePoint", "RootPoint", "Point"):
            p = point_m(safe_prop(obj, attr))
            if p is not None:
                return p
        nxt = safe_prop(obj, "Geometry")
        if nxt is None or nxt is obj:
            break
        obj = nxt
    return None


def intent_axis(intent, flip=False):
    if intent is None:
        return None, None
    g = safe_prop(intent, "Geometry")
    p = point_m(safe_prop(intent, "Point")) or geometry_point(g)
    d = geometry_direction(g)
    if d and flip:
        d = [-x for x in d]
    return p, d


def mesh_name(path):
    stem = safe_name(os.path.splitext(os.path.basename(path))[0])
    digest = hashlib.sha1(os.path.normcase(path).encode("utf-8", "replace")).hexdigest()[:8]
    return f"{stem}_{digest}.stl"


def _map_set(options, name, value):
    try:
        options.Remove(name)
    except Exception:
        pass
    options.Add(name, value)


def export_stl(app, doc, out_path):
    """Export a document as a high-resolution, metre-scaled binary STL."""
    tr = app.ApplicationAddIns.ItemById(STL_TRANSLATOR)
    ctx = app.TransientObjects.CreateTranslationContext()
    ctx.Type = FILE_BROWSE_IO
    opts = app.TransientObjects.CreateNameValueMap()
    data = app.TransientObjects.CreateDataMedium()
    data.FileName = os.path.abspath(out_path)
    if tr.HasSaveCopyAsOptions(doc, ctx, opts):
        _map_set(opts, "ExportUnits", 6)          # metre
        _map_set(opts, "Resolution", 0)           # high
        _map_set(opts, "OutputFileType", 0)       # binary
        _map_set(opts, "ExportFileStructure", 0)  # one file
        _map_set(opts, "ExportColor", True)
    tr.SaveCopyAs(doc, ctx, opts, data)
    if not os.path.exists(out_path):
        raise RuntimeError(f"Inventor STL translator did not create {out_path}")


class Inventor:
    """Own a private Inventor session or attach to the user's running session."""
    def __init__(self, visible=False, attach=False):
        if not HAVE_WIN32:
            raise InventorUnavailable(
                "Inventor extraction requires Windows + pywin32 + Autodesk Inventor")
        try:
            pythoncom.CoInitialize()
        except Exception:
            pass
        self.owned, self.docs = not attach, []
        try:
            self.app = (win32com.client.GetActiveObject("Inventor.Application")
                        if attach else win32com.client.DispatchEx("Inventor.Application"))
        except Exception as e:
            raise InventorUnavailable(f"could not connect to Autodesk Inventor: {e}") from e
        try:
            self.app.Visible = bool(visible or attach)
        except Exception:
            pass

    def open(self, path):
        path = os.path.abspath(path)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        doc = self.app.Documents.Open(path, bool(safe_prop(self.app, "Visible", False)))
        self.docs.append(doc)
        return doc

    def close(self, doc):
        try:
            doc.Close(True)  # SkipSave=True
        except Exception:
            pass
        if doc in self.docs:
            self.docs.remove(doc)

    def shutdown(self):
        for doc in list(reversed(self.docs)):
            self.close(doc)
        if self.owned:
            try:
                self.app.Quit()
            except Exception:
                pass
        self.app = None
        try:
            pythoncom.CoUninitialize()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()
