from __future__ import annotations

import os
import re
import shlex
import subprocess

_CM = os.path.expanduser("~/.ssh/cm-viewer-%r@%h:%p")
_SSH = [
    "ssh", "-o", "ControlMaster=auto", "-o", f"ControlPath={_CM}",
    "-o", "ControlPersist=30", "-o", "ConnectTimeout=6",
    "-o", "ConnectionAttempts=1",
    "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
    "-o", "StrictHostKeyChecking=accept-new", "-o", "BatchMode=yes",
]

_MODE = {0: "OTHER", 1: "IDLE", 2: "MOVE", 3: "GUIDING", 4: "REFLEX",
         5: "USER_STOPPED", 6: "ERROR_RECOVERY"}


def _bracket(pat: str) -> str:
    out = []
    for p in pat.split("|"):
        out.append(f"[{p[0]}]{p[1:]}" if p and p[0].isalnum() else p)
    return "|".join(out)


def _run(user_addr: str, remote: str, timeout: float = 8.0):
    argv = _SSH + [user_addr, f"bash -lc {shlex.quote(remote)}"]
    try:
        p = subprocess.run(argv, capture_output=True, text=True,
                           timeout=timeout)
        return p.returncode, p.stdout, p.stderr
    except Exception as e:
        return 124, "", str(e)


class SshStatus:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.hosts = cfg["hosts"]
        self.sources = cfg.get("sources", {})

    def _ua(self, host_key: str) -> str:
        h = self.hosts[host_key]
        return f"{h['user']}@{h['addr']}"

    def _src(self, host_key: str) -> str:
        return self.sources.get(host_key, "")

    def reachable(self, host_key: str) -> bool:
        rc, out, _ = _run(self._ua(host_key), "echo ok", timeout=6)
        return rc == 0 and "ok" in out

    def service_running(self, host_key: str, pattern: str) -> bool:
        rc, out, _ = _run(
            self._ua(host_key),
            f"pgrep -af {shlex.quote(_bracket(pattern))} 2>/dev/null | head -1")
        return bool(out.strip())

    def controllers(self, host_key: str, cm: str) -> dict:
        src = self._src(host_key)
        cmd = (f"{src} && ros2 control list_controllers -c {cm} 2>/dev/null"
               if src else
               f"ros2 control list_controllers -c {cm} 2>/dev/null")
        rc, out, _ = _run(self._ua(host_key), cmd, timeout=15)
        active, inactive = [], []
        for ln in out.splitlines():
            parts = ln.split()
            if len(parts) >= 3:
                name, state = parts[0], parts[-1]
                (active if "active" in state else inactive).append(name)
        return {"active": active, "inactive": inactive,
                "ok": bool(active) or bool(inactive)}

    def arm_state(self, host_key: str, topic: str) -> dict:
        import yaml
        src = self._src(host_key)
        cmd = (f"{src} && timeout 5 ros2 topic echo {topic} --once 2>/dev/null")
        rc, out, _ = _run(self._ua(host_key), cmd, timeout=9)
        if not out.strip():
            return {"ok": False, "mode": None, "mode_name": "?",
                    "errors": [], "locked": False}
        try:
            docs = [d for d in yaml.safe_load_all(out) if d]
            d = docs[0]
        except Exception:
            return {"ok": False, "mode": None, "mode_name": "?",
                    "errors": [], "locked": False}
        mode = d.get("robot_mode")
        errs = []
        ce = d.get("current_errors")
        if isinstance(ce, dict):
            errs = [k for k, v in ce.items() if v is True]
        locked = (mode in (4, 5)) or bool(errs)
        return {"ok": True, "mode": mode,
                "mode_name": _MODE.get(mode, str(mode)),
                "errors": errs, "locked": locked}

    def spine_position(self) -> dict:
        sp = self.cfg.get("spine_service")
        if not sp:
            return {"ok": False, "position": None}
        src = self._src(sp["host"])
        call = (f"timeout 6 ros2 service call {sp['service']} "
                f"{sp['type']} '{{}}' 2>/dev/null")
        cmd = f"{src} && {call}" if src else call
        rc, out, _ = _run(self._ua(sp["host"]), cmd, timeout=10)
        m = re.search(r"position=([-\d.eE+]+)", out)
        if not m:
            return {"ok": False, "position": None}
        return {"ok": True, "position": float(m.group(1))}

    def poll(self) -> dict:
        from concurrent.futures import ThreadPoolExecutor

        out: dict = {"hosts": {}, "services": {}, "controllers": {},
                     "arm": {}}
        with ThreadPoolExecutor(max_workers=8) as ex:
            fh = {hk: ex.submit(self.reachable, hk) for hk in self.hosts}
            for hk, f in fh.items():
                out["hosts"][hk] = {"reachable": f.result(),
                                    "addr": self.hosts[hk].get("addr")}
            up = {hk for hk, h in out["hosts"].items() if h["reachable"]}

            fs = {name: ex.submit(self.service_running, s["host"],
                                  s["pattern"])
                  for name, s in self.cfg.get("services", {}).items()
                  if s["host"] in up}
            fc = {cm["name"]: ex.submit(self.controllers, cm["host"],
                                        cm["cm"])
                  for cm in self.cfg.get("controller_managers", [])
                  if cm["host"] in up}
            fa = {a["name"]: ex.submit(self.arm_state, a["host"],
                                       a["topic"])
                  for a in self.cfg.get("arm_state", [])
                  if a["host"] in up}
            for name, f in fc.items():
                out["controllers"][name] = f.result()
            for name, f in fa.items():
                out["arm"][name] = f.result()
            arm_ok = bool(out["arm"]) and all(
                a.get("ok") for a in out["arm"].values())
            ctrl_ok = bool(out["controllers"]) and all(
                c.get("active") for c in out["controllers"].values())
            for name, f in fs.items():
                running = f.result()
                health = self.cfg["services"][name].get("health")
                if health == "controllers":
                    healthy = running and ctrl_ok
                elif health == "arm_state":
                    healthy = running and arm_ok
                else:
                    healthy = None
                out["services"][name] = {
                    "host": self.cfg["services"][name]["host"],
                    "running": running, "healthy": healthy}
            for name, s in self.cfg.get("services", {}).items():
                if name not in out["services"]:
                    out["services"][name] = {"host": s["host"],
                                             "running": False,
                                             "healthy": None}
        return out
