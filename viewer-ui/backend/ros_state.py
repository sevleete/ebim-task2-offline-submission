from __future__ import annotations

import math
import threading
import time
from collections import deque


class _Rate:

    def __init__(self, n: int = 30):
        self.ts = deque(maxlen=n)

    def tick(self):
        self.ts.append(time.monotonic())

    def hz(self) -> float:
        if len(self.ts) < 2 or not self.fresh():
            return 0.0
        dt = self.ts[-1] - self.ts[0]
        return (len(self.ts) - 1) / dt if dt > 0 else 0.0

    def fresh(self, within: float = 2.0) -> bool:
        return bool(self.ts) and (time.monotonic() - self.ts[-1]) < within


class RosState:
    def __init__(self, cfg: dict):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image, JointState
        from nav_msgs.msg import Odometry

        self.cfg = cfg
        self.lock = threading.Lock()
        self._bridge = CvBridge()
        self.node = Node("viewer_ros_state")

        self.cam_frame: dict[str, object] = {}
        self.cam_res: dict[str, str] = {}
        self.cam_rate: dict[str, _Rate] = {}
        self.joints: dict[str, dict] = {}
        self.joint_rate: dict[str, _Rate] = {}
        self.gello: dict[str, dict] = {}
        self.gello_rate: dict[str, _Rate] = {}
        self.odom = {}
        self.odom_rate = _Rate()

        for c in cfg.get("cameras", []):
            k = c["key"]
            self.cam_rate[k] = _Rate()
            if c["topic"].rstrip("/").endswith("compressed"):
                from sensor_msgs.msg import CompressedImage
                self.node.create_subscription(
                    CompressedImage, c["topic"], self._cam_jpeg_cb(k),
                    qos_profile_sensor_data)
            else:
                self.node.create_subscription(
                    Image, c["topic"], self._cam_cb(k),
                    qos_profile_sensor_data)
        for j in cfg.get("arms_joint_states", []):
            k = j["key"]
            self.joint_rate[k] = _Rate()
            self.node.create_subscription(
                JointState, j["topic"], self._joint_cb(k), 10)
        for g in cfg.get("gello", []):
            k = g["key"]
            self.gello_rate[k] = _Rate()
            self.node.create_subscription(
                JointState, g["topic"], self._gello_cb(k), 10)
        if cfg.get("odom"):
            self.node.create_subscription(
                Odometry, cfg["odom"], self._odom_cb, 10)

    def _cam_cb(self, key):
        def cb(msg):
            img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.cam_frame[key] = img
                self.cam_res[key] = f"{msg.width}x{msg.height}"
                self.cam_rate[key].tick()
        return cb

    def _cam_jpeg_cb(self, key):
        def cb(msg):
            buf = bytes(msg.data)
            res = None
            if key not in self.cam_res:
                import cv2
                import numpy as np
                img = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8),
                                   cv2.IMREAD_COLOR)
                if img is not None:
                    res = f"{img.shape[1]}x{img.shape[0]}"
            with self.lock:
                self.cam_frame[key] = buf
                if res:
                    self.cam_res[key] = res
                self.cam_rate[key].tick()
        return cb

    def _joint_cb(self, key):
        def cb(msg):
            with self.lock:
                self.joints[key] = dict(zip(msg.name, msg.position))
                self.joint_rate[key].tick()
        return cb

    def _gello_cb(self, key):
        def cb(msg):
            with self.lock:
                self.gello[key] = dict(zip(msg.name, msg.position))
                self.gello_rate[key].tick()
        return cb

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y),
                         1 - 2 * (o.y * o.y + o.z * o.z))
        speed = math.hypot(t.linear.x, t.linear.y)
        with self.lock:
            self.odom = {"x": p.x, "y": p.y, "yaw": yaw,
                         "vx": t.linear.x, "vy": t.linear.y,
                         "wz": t.angular.z, "speed": speed,
                         "moving": speed > 0.02 or abs(t.angular.z) > 0.02}
            self.odom_rate.tick()

    def joints_snapshot(self) -> dict:
        names = self.cfg.get("joint_names", {})
        with self.lock:
            out = {}
            for k in self.joint_rate:
                names_k = names.get(k, [])
                if not self.joint_rate[k].fresh():
                    out[k] = [None] * len(names_k)
                    continue
                m = self.joints.get(k, {})
                out[k] = [round(float(m[n]), 4) if n in m else None
                          for n in names_k]
        return out

    def snapshot(self) -> dict:
        with self.lock:
            cams = {k: {"rate": round(self.cam_rate[k].hz(), 1),
                        "res": self.cam_res.get(k, "?"),
                        "connected": self.cam_rate[k].fresh()}
                    for k in self.cam_rate}
            names = self.cfg.get("joint_names", {})

            def vec(store, key):
                m = store.get(key, {})
                return [round(float(m[n]), 4) if n in m else None
                        for n in names.get(key, [])]

            joints = {k: {"pos": vec(self.joints, k),
                          "rate": round(self.joint_rate[k].hz(), 1),
                          "ok": self.joint_rate[k].fresh()}
                      for k in self.joint_rate}
            gello = {k: {"rate": round(self.gello_rate[k].hz(), 1),
                         "ok": self.gello_rate[k].fresh()}
                     for k in self.gello_rate}
            odom = dict(self.odom)
            odom["rate"] = round(self.odom_rate.hz(), 1)
            odom["ok"] = self.odom_rate.fresh()
        return {"cameras": cams, "joints": joints, "gello": gello,
                "odom": odom}

    def jpeg(self, key: str, quality: int = 80):
        import cv2
        with self.lock:
            img = self.cam_frame.get(key)
        if img is None:
            return None
        if isinstance(img, (bytes, bytearray)):
            return bytes(img)
        ok, buf = cv2.imencode(".jpg", img,
                               [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else None
