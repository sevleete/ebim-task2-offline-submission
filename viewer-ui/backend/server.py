from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import yaml

_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _DIR)
_FRONTEND = os.path.normpath(os.path.join(_DIR, "..", "frontend"))
_ASSETS = os.path.normpath(os.path.join(_DIR, "..", "assets"))

from ssh_status import SshStatus
from ros_state import RosState

_ssh_state: dict = {}
_ssh_lock = threading.Lock()
_spine_state: dict = {"ok": False, "position": None}
_spine_lock = threading.Lock()
ROS: RosState | None = None
CFG: dict = {}
_FPS = 15
DEMO = False


def _poller(ssh: SshStatus, interval: float):
    while True:
        try:
            s = ssh.poll()
        except Exception as e:
            s = {"error": str(e)}
        with _ssh_lock:
            _ssh_state.clear()
            _ssh_state.update(s)
        time.sleep(interval)


def _spine_poller(ssh: SshStatus, interval: float):
    while True:
        try:
            s = ssh.spine_position()
        except Exception:
            s = {"ok": False, "position": None}
        with _spine_lock:
            _spine_state.clear()
            _spine_state.update(s)
        time.sleep(interval)


def _derive_alerts(ros_snap: dict, ssh: dict) -> list[dict]:
    a = []
    for hk, h in ssh.get("hosts", {}).items():
        if not h.get("reachable"):
            a.append({"level": "error", "msg": f"主机 {hk} 不可达"})
    for arm, s in ssh.get("arm", {}).items():
        if s.get("locked"):
            errs = ",".join(s.get("errors", [])) or s.get("mode_name", "")
            a.append({"level": "error",
                      "msg": f"{arm}臂 抱死/报错({s.get('mode_name')}) {errs}"})
    for name, s in ssh.get("services", {}).items():
        if not s.get("running"):
            a.append({"level": "warn", "msg": f"服务 {name} 未运行"})
        elif s.get("healthy") is False:
            a.append({"level": "warn",
                      "msg": f"服务 {name} 进程在但不可用(硬件未连/控制器未激活)"})
    for cam, c in ros_snap.get("cameras", {}).items():
        if not c.get("connected"):
            a.append({"level": "warn", "msg": f"相机 {cam} 无数据"})
    return a


def _status() -> dict:
    if DEMO:
        from demo import demo_snapshot
        ros_snap, ssh = demo_snapshot()
    else:
        ros_snap = ROS.snapshot() if ROS else {}
        with _ssh_lock:
            ssh = json.loads(json.dumps(_ssh_state))
        with _spine_lock:
            ssh["spine"] = dict(_spine_state)
    return {"ts": time.time(), "demo": DEMO, "ros": ros_snap, "ssh": ssh,
            "alerts": _derive_alerts(ros_snap, ssh)}


def _joints() -> dict:
    if DEMO:
        from demo import demo_snapshot
        ros_snap, ssh = demo_snapshot()
        j = {k: v.get("pos") for k, v in ros_snap.get("joints", {}).items()}
        sp = ssh.get("spine", {})
    else:
        j = ROS.joints_snapshot() if ROS else {}
        with _spine_lock:
            sp = dict(_spine_state)
    return {"ts": time.time(), "joints": j,
            "spine": sp.get("position") if sp.get("ok") else None}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _static(self, name, ctype):
        path = os.path.join(_FRONTEND, name)
        if not os.path.isfile(path):
            self.send_error(404)
            return
        with open(path, "rb") as f:
            self._send(200, ctype, f.read())

    def do_GET(self):
        p = urlparse(self.path)
        if p.path == "/":
            self._static("index.html", "text/html; charset=utf-8")
        elif p.path == "/style.css":
            self._static("style.css", "text/css")
        elif p.path == "/app.js":
            self._static("app.js", "application/javascript")
        elif p.path == "/twin.js":
            self._static("twin.js", "application/javascript")
        elif p.path.startswith("/assets/"):
            rel = p.path[len("/assets/"):]
            safe = os.path.normpath(os.path.join(_ASSETS, rel))
            if not safe.startswith(_ASSETS) or not os.path.isfile(safe):
                self.send_error(404)
                return
            ext = os.path.splitext(safe)[1].lower()
            ctype = {".urdf": "application/xml",
                     ".dae": "model/vnd.collada+xml", ".stl": "model/stl",
                     ".glb": "model/gltf-binary",
                     ".js": "application/javascript",
                     ".png": "image/png", ".jpg": "image/jpeg"}.get(
                         ext, "application/octet-stream")
            with open(safe, "rb") as f:
                self._send(200, ctype, f.read())
        elif p.path == "/api/status":
            self._send(200, "application/json",
                       json.dumps(_status()).encode("utf-8"))
        elif p.path == "/api/joints":
            self._send(200, "application/json",
                       json.dumps(_joints()).encode("utf-8"))
        elif p.path == "/stream":
            self._stream(parse_qs(p.query).get("cam", [""])[0])
        else:
            self.send_error(404)

    def do_POST(self):
        p = urlparse(self.path)
        if p.path != "/api/control":
            self.send_error(404)
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            body = {}
        action = body.get("action", "")
        if DEMO:
            res = {"ok": True, "output": f"[DEMO] 已模拟:{action}(未真实执行)"}
        else:
            from control_exec import run_action
            res = run_action(action, body)
        self._send(200, "application/json", json.dumps(res).encode("utf-8"))

    def _stream(self, cam):
        if DEMO:
            from demo import demo_frame
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    jpg = demo_frame(cam)
                    if jpg:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n"
                                         .encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(1.0 / _FPS)
            except (BrokenPipeError, ConnectionResetError):
                return
            return
        if ROS is None:
            self.send_error(503)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                jpg = ROS.jpeg(cam)
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n"
                                     .encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                time.sleep(1.0 / _FPS)
        except (BrokenPipeError, ConnectionResetError):
            return


def main() -> int:
    global ROS, CFG, _FPS, DEMO
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=os.path.join(_DIR, "..",
                                                     "config.yaml"))
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--no-ros", action="store_true",
                    help="仅 SSH 状态,不起 ROS 订阅(调试用)")
    ap.add_argument("--demo", action="store_true",
                    help="不连 ROS/机器人,造合成数据看界面")
    args = ap.parse_args()
    _FPS = args.fps
    DEMO = args.demo
    with open(args.config, encoding="utf-8") as f:
        CFG = yaml.safe_load(f)

    if not DEMO:
        ssh = SshStatus(CFG)
        threading.Thread(target=_poller,
                         args=(ssh, float(CFG.get("poll_interval", 2.0))),
                         daemon=True).start()
        threading.Thread(
            target=_spine_poller,
            args=(ssh, float(CFG.get("spine_poll_interval", 1.0))),
            daemon=True).start()
        if not args.no_ros:
            import rclpy
            rclpy.init()
            ROS = RosState(CFG)
            threading.Thread(target=rclpy.spin, args=(ROS.node,),
                             daemon=True).start()

    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    mode = "DEMO(合成数据)" if DEMO else ("无ROS" if args.no_ros else "实机")
    print(f"viewer-ui 后端监听 {args.bind}:{args.port}  模式:{mode}")
    print(f"看界面:在 det/Mac 上 `ssh -N -L {args.port}:localhost:{args.port} "
          f"<det>`,再开 http://localhost:{args.port}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
