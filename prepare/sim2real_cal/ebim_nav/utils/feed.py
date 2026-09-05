from __future__ import annotations

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from ebim_nav.common import load_config


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CloudOdomFeed(Node):

    def __init__(self, name: str = "ebim_feed"):
        if not rclpy.ok():
            rclpy.init()
        super().__init__(name)
        self.cfg = load_config()
        self.lock = threading.Lock()
        self.pts = None
        self.odom = None
        self.stamp = 0.0
        self.create_subscription(PointCloud2, self.cfg["merger"]["out_cloud_topic"],
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.cfg["base"]["odom_topic"],
                                 self._on_odom, qos_profile_sensor_data)
        self.pub = self.create_publisher(TwistStamped, self.cfg["base"]["cmd_vel_topic"], 10)

    def _on_cloud(self, msg):
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y"))
        with self.lock:
            self.pts = pts.astype(np.float64)
            self.stamp = time.time()

    def _on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self.odom = (p.x, p.y, quat_yaw(q))

    def start(self):
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        return self

    def latest(self):
        with self.lock:
            if self.pts is None or self.odom is None:
                return None
            return self.stamp, self.pts, self.odom

    def wait_ready(self, timeout: float = 10.0):
        t0 = time.time()
        while self.latest() is None:
            if time.time() - t0 > timeout:
                print("✗ 等不到 merged_cloud/odom——scan_merger 和底盘(teleop)在跑吗？\n"
                      "  雷达+merger: 终端起 ./run.sh scan_merger（前置 tmrctl --up ctl --only lidar）\n"
                      "  底盘 odom : .50 的 mobile_teleop 要在跑", flush=True)
                import os
                os._exit(1)
            time.sleep(0.1)
        return self

    def send(self, vx: float, vy: float, wz: float, max_xy: float, max_yaw: float):
        vx = max(-max_xy, min(max_xy, vx))
        vy = max(-max_xy, min(max_xy, vy))
        wz = max(-max_yaw, min(max_yaw, wz))
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = vx
        m.twist.linear.y = vy
        m.twist.angular.z = wz
        self.pub.publish(m)
        return vx, vy, wz

    def stop_cmd(self):
        self.send(0.0, 0.0, 0.0, 1.0, 1.0)


def save_frames(path, frames):
    np.savez_compressed(
        path,
        t=np.array([f[0] for f in frames]),
        counts=np.array([len(f[1]) for f in frames]),
        pts=np.vstack([f[1] for f in frames]),
        odom=np.array([f[2] for f in frames]))


def load_frames(path):
    d = np.load(path)
    frames, ofs = [], 0
    for t, n, od in zip(d["t"], d["counts"], d["odom"]):
        frames.append((float(t), d["pts"][ofs:ofs + int(n)], tuple(od)))
        ofs += int(n)
    return frames
