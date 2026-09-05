from __future__ import annotations

import os
import subprocess

_HERE = os.path.dirname(os.path.abspath(__file__))
_OFFLINE_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
_PIXI = os.path.expanduser("~/.pixi/bin/pixi")


def _args(action: str, params: dict):
    if action == "enable_all":
        return ["--enable", "all"]
    if action == "recover_left":
        return ["--recover", "left"]
    if action == "recover_right":
        return ["--recover", "right"]
    if action == "setup_collision":
        return ["--setup_collision"]
    if action == "hold_both":
        return ["--hold", "both"]
    if action == "unhold_both":
        return ["--unhold", "both"]
    if action == "spine":
        try:
            v = float(params.get("value", 0.5))
        except (TypeError, ValueError):
            return None
        v = max(0.0, min(1.0, v))
        return ["--spine", f"{v:.3f}"]
    if action in ("arm_left", "arm_right"):
        js = params.get("joints")
        if not isinstance(js, list) or len(js) != 7:
            return None
        flag = "--left_arm_joint" if action == "arm_left" else "--right_arm_joint"
        return [flag] + [f"{float(x):.4f}" for x in js]
    if action == "down_arms":
        return ["--down", "--only", "arms"]
    return None


def run_action(action: str, params: dict | None = None) -> dict:
    args = _args(action, params or {})
    if args is None:
        return {"ok": False, "output": f"未知/不支持的动作:{action}"}
    cmd = [_PIXI, "run", "tmrctl"] + args
    try:
        p = subprocess.run(cmd, cwd=_OFFLINE_ROOT, capture_output=True,
                           text=True, timeout=90)
        raw = (p.stdout + "\n" + p.stderr)
        lines = [ln for ln in raw.splitlines()
                 if ln.strip() and "✨" not in ln
                 and "rmw_cyclonedds" not in ln and "type hash" not in ln]
        return {"ok": p.returncode == 0,
                "output": "\n".join(lines[-6:])[:500] or "(无输出)"}
    except subprocess.TimeoutExpired:
        return {"ok": False, "output": "执行超时"}
    except Exception as e:
        return {"ok": False, "output": str(e)}
