"""Inventor-aware wrapper around the existing sw2robot browser server.

The upstream web server is deliberately left untouched: this module subclasses
its request handler only for the three CAD-entry routes that need to know about
Autodesk Inventor (.iam/.ipt).  Once extraction has produced graph.json + meshes,
all editing/build/export routes are the original shared sw2robot implementation.
"""
from __future__ import annotations

import os
import string
import threading
import urllib.parse

from . import webserver as _ws

_INVENTOR_EXTS = (".iam", ".ipt")
_CAD_EXTS = (".sldasm", ".sldprt", ".iam", ".ipt", ".urdf")
_INVENTOR_STAGES = ["connect Inventor", "extract assembly",
                    "export meshes", "build package"]


def _is_inventor(path):
    return str(path or "").lower().endswith(_INVENTOR_EXTS)


def _inventor_progress(msg):
    """Bridge Inventor extraction progress into upstream's shared progress UI."""
    if _ws._job.get("cancel"):
        raise _ws._CancelExtract()
    msg = str(msg)
    _ws._job["log"].append(msg)
    _ws._prog_log(msg)

    low = msg.lower()
    if low.startswith("opening ") or "inventor" in low and "connect" in low:
        _ws._prog_stage("connect Inventor")
    elif low.startswith("exporting mesh "):
        # "exporting mesh i/n: name"
        try:
            count = low.split("exporting mesh ", 1)[1].split(":", 1)[0]
            i_s, n_s = count.split("/", 1)
            i, n = int(i_s), int(n_s)
            _ws._prog_stage("export meshes", frac=(i / n) if n else None)
            _ws._prog_update(sub=msg.split(": ", 1)[-1])
        except Exception:
            _ws._prog_stage("export meshes")
            _ws._prog_update(sub=msg)
    elif (low.startswith("reading ") or low.startswith("components:")
          or low.startswith("relationships:") or low.startswith("warn:")):
        _ws._prog_stage("extract assembly")
        _ws._prog_update(sub=msg)
    elif low.startswith("graph:"):
        _ws._prog_stage("extract assembly")
        _ws._prog_update(sub=msg)
    else:
        _ws._prog_update(sub=msg)


def _run_inventor_extract(cad_path):
    """Background .iam/.ipt extraction feeding the existing web job state."""
    try:
        from sw2robot.exporter.export import build
        from sw2robot.exporter.inventor_backend.com import InventorUnavailable
        from sw2robot.exporter.inventor_backend.extract import extract_inventor

        _inventor_progress(
            f"connecting to Autodesk Inventor for {os.path.basename(cad_path)} ...")
        root = _InventorHandler.root_dir
        try:
            # Prefer the user's already-running Inventor so opening a large model
            # does not pay a second application startup.  If none is running,
            # start a private automation instance instead.
            pkg = extract_inventor(cad_path, out_dir=root, attach=True,
                                   progress=_inventor_progress)
        except InventorUnavailable:
            _inventor_progress(
                "no attachable Inventor session; starting a private instance ...")
            pkg = extract_inventor(cad_path, out_dir=root, visible=False,
                                   attach=False, progress=_inventor_progress)

        if _ws._job.get("cancel"):
            raise _ws._CancelExtract()
        _ws._prog_stage("build package")
        _ws._prog_update(sub="building URDF from Inventor graph ...")
        _ws._prog_log("building URDF from Inventor graph ...")
        build(pkg)
        _ws._preconvert_meshes(str(pkg))
        _ws._job["package"] = str(pkg)
        _ws._job["running"] = False
        _ws._prog_finish(result={"package": str(pkg)})
        print(f"[sw2robot.web] Inventor extract done -> {pkg}")
    except _ws._CancelExtract:
        _ws._job["cancelled"] = True
        _ws._job["log"].append("Inventor extraction cancelled by user.")
        _ws._prog_log("Inventor extraction cancelled by user.")
        _ws._job["running"] = False
        _ws._prog_finish(cancelled=True)
        print("[sw2robot.web] Inventor extract CANCELLED")
    except Exception as e:
        import traceback

        _ws._job["error"] = f"{type(e).__name__}: {e}"
        _ws._job["running"] = False
        _ws._prog_finish(error=f"{type(e).__name__}: {e}")
        print(f"[sw2robot.web] Inventor extract FAILED: {e!r}")
        traceback.print_exc()


def _send_fs(handler, target):
    """Upstream /api/fs plus Inventor IAM/IPT file visibility."""
    if not target:
        roots = []
        for drive in string.ascii_uppercase:
            if os.path.exists(f"{drive}:\\"):
                roots.append(f"{drive}:\\")
        root = type(handler).root_dir
        if root and os.path.isdir(root) and root not in roots:
            roots.append(root)
        return handler._send_json(
            {"path": "", "parent": None,
             "dirs": [{"name": r, "path": r, "package": False}
                      for r in roots],
             "files": []})

    p = os.path.abspath(target)
    if not os.path.isdir(p):
        return handler._send_json({"error": f"not a dir: {p}"}, 400)
    dirs, files = [], []
    try:
        for name in sorted(os.listdir(p), key=str.lower):
            full = os.path.join(p, name)
            if name.startswith(("~$", ".")):
                continue
            if os.path.isdir(full):
                dirs.append({"name": name, "path": full,
                             "package": _ws._dir_is_package(full)})
            elif name.lower().endswith(_CAD_EXTS):
                files.append({"name": name, "path": full})
    except OSError as e:
        return handler._send_json({"error": str(e)}, 400)
    parent = os.path.dirname(p.rstrip("\\/"))
    return handler._send_json(
        {"path": p, "parent": parent if parent != p else None,
         "dirs": dirs[:400], "files": files[:400]})


class _InventorHandler(_ws._Handler):
    """Only intercept CAD discovery/extraction; delegate everything else."""

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if path == "/api/fs":
            target = (query.get("path") or [""])[0]
            return _send_fs(self, target)

        if path == "/api/configurations":
            target = (query.get("path") or [""])[0]
            if _is_inventor(target):
                # Model State / iAssembly selection is a later feature.  Returning
                # no alternatives makes the current UI proceed with the active model.
                return self._send_json(
                    {"configurations": [], "source": "inventor"})

        if path == "/api/extract":
            target = (query.get("path") or [""])[0]
            target = os.path.abspath(os.path.expanduser(
                target.strip().strip('"')))
            if _is_inventor(target):
                if not os.path.isfile(target):
                    return self._send_json(
                        {"error": f"not an .iam/.ipt file: {target}"}, 400)
                if not _ws._prog_start("extract", _INVENTOR_STAGES):
                    return self._send_json(
                        {"error": "a job is already running"}, 409)
                with _ws._job_lock:
                    _ws._job.update(running=True, log=[], error=None,
                                   package=None, cancel=False, cancelled=False)
                try:
                    threading.Thread(target=_run_inventor_extract,
                                     args=(target,), daemon=True).start()
                except Exception as e:
                    with _ws._job_lock:
                        _ws._job.update(running=False)
                    _ws._prog_finish(error=f"{type(e).__name__}: {e}")
                    return self._send_json(
                        {"error": f"failed to start Inventor extract: {e}"}, 500)
                return self._send_json({"started": True})

        return super().do_GET()


def main():
    # webserver.main()/serve() refer to the module-global _Handler class.  Swap
    # it before entering upstream so all root/package state and helper functions
    # consistently see our subclass.  The original implementation remains the
    # source of truth for every non-Inventor route.
    _ws._Handler = _InventorHandler
    return _ws.main()


if __name__ == "__main__":
    main()
