from __future__ import annotations

import math
import time


def demo_snapshot():
    t = time.time()
    base = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    def joints(phase):
        pos = [round(b + 0.35 * math.sin(t * 0.6 + i + phase), 4)
               for i, b in enumerate(base)]
        return {"pos": pos, "rate": 90.0, "ok": True}

    left_locked = (int(t) % 20) < 4
    wrist_r_ok = (int(t) % 15) > 3
    moving = (int(t) % 8) < 4

    ros = {
        "cameras": {
            "head": {"rate": 14.3, "res": "1280x720", "connected": True},
            "wrist_left": {"rate": 30.1, "res": "640x480", "connected": True},
            "wrist_right": {"rate": 29.8 if wrist_r_ok else 0.0,
                            "res": "640x480", "connected": wrist_r_ok},
        },
        "joints": {"left": joints(0.0), "right": joints(1.5)},
        "gello": {"left": {"rate": 30.0, "ok": True},
                  "right": {"rate": 0.0, "ok": False}},
        "odom": {"x": 1.23, "y": -0.41, "yaw": 0.15,
                 "vx": 0.12 if moving else 0.0, "vy": 0.0, "wz": 0.0,
                 "speed": 0.12 if moving else 0.0, "moving": moving,
                 "rate": 50.0, "ok": True},
    }
    ssh = {
        "hosts": {"base": {"reachable": True, "addr": "172.16.0.50"},
                  "arms": {"reachable": True, "addr": "172.16.0.100"},
                  "teleop": {"reachable": True, "addr": "172.16.0.101"}},
        "services": {
            "base": {"host": "base", "running": True, "healthy": None},
            "zed": {"host": "base", "running": True, "healthy": None},
            "lidar": {"host": "base", "running": True, "healthy": None},
            "spine": {"host": "arms", "running": True, "healthy": None},
            "grippers": {"host": "arms", "running": True, "healthy": None},
            "arms": {"host": "arms", "running": True,
                     "healthy": not left_locked},
            "wrist_cams": {"host": "arms", "running": True, "healthy": None},
            "gello": {"host": "teleop", "running": False, "healthy": None},
            "pedal": {"host": "teleop", "running": False, "healthy": None},
        },
        "controllers": {
            "left": {"active": ["joint_impedance_controller",
                                "joint_state_broadcaster"],
                     "inactive": [], "ok": True},
            "right": {"active": ["joint_impedance_controller"],
                      "inactive": [], "ok": True},
        },
        "arm": {
            "left": {"ok": True, "mode": 4 if left_locked else 2,
                     "mode_name": "REFLEX" if left_locked else "MOVE",
                     "errors": (["joint_position_limits_violation"]
                                if left_locked else []),
                     "locked": left_locked},
            "right": {"ok": True, "mode": 1, "mode_name": "IDLE",
                      "errors": [], "locked": False},
        },
    }
    return ros, ssh


def demo_frame(cam: str):
    try:
        import cv2
        import numpy as np
    except Exception:
        return None
    t = time.time()
    h, w = 360, 480
    xs = np.linspace(0, 255, w, dtype=np.uint8)
    row = np.tile(xs, (h, 1))
    shift = int((math.sin(t * 0.8) * 0.5 + 0.5) * 255)
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :, 0] = (row + shift) % 255
    img[:, :, 1] = (row.T[:h, :w] if False else row) // 2
    img[:, :, 2] = 255 - ((row + shift) % 255)
    cv2.putText(img, f"{cam} (demo)", (16, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(img, time.strftime("%H:%M:%S"), (16, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 220, 255), 2)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes() if ok else None
