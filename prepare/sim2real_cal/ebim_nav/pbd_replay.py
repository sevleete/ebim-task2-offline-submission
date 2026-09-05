#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ebim_nav.common import PKG_ROOT, load_config


def quat_yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PBDReplay(Node):
    def __init__(self, name, mode, dry):
        super().__init__("ebim_pbd_replay")
        self.cfg = load_config()
        d = np.load(PKG_ROOT / "paths" / f"{name}_pbd.npz")
        self.cmds = d["cmds"]
        self.path = d["odom"]
        self.mode = mode
        self.dry = dry
        b = self.cfg["base"]
        self.max_xy = min(b["max_vel_xy"], float(np.abs(self.cmds[:, 1:3]).max()) + 0.02) if len(self.cmds) else 0.1
        self.max_yaw = min(b["max_vel_yaw"], float(np.abs(self.cmds[:, 3]).max()) + 0.05) if len(self.cmds) else 0.2
        self.kp_xy, self.kp_yaw = 1.2, 1.5
        self.lookahead = 0.25

        self.pose = None
        self.origin = None
        self.lock = threading.Lock()
        self.pub = self.create_publisher(TwistStamped, b["cmd_vel_topic"], 10)
        self.create_subscription(Odometry, b["odom_topic"], self.on_odom, qos_profile_sensor_data)
        self.start_t = None
        self.done = False
        self.create_timer(0.05, self.tick)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self.pose = (p.x, p.y, quat_yaw(q))
            if self.origin is None:
                self.origin = self.pose
                self.get_logger().info(f"回放起点对齐: ({p.x:.2f},{p.y:.2f})")

    def rel_path_target(self, t):
        px = np.interp(t, self.path[:, 0], self.path[:, 1])
        py = np.interp(t, self.path[:, 0], self.path[:, 2])
        pyaw = np.interp(t, self.path[:, 0], np.unwrap(self.path[:, 3]))
        p0 = self.path[0]
        c, s = math.cos(-p0[3]), math.sin(-p0[3])
        dx = c*(px-p0[1]) - s*(py-p0[2])
        dy = s*(px-p0[1]) + c*(py-p0[2])
        return dx, dy, wrap(pyaw - p0[3])

    def send(self, vx, vy, wz):
        vx = max(-self.max_xy, min(self.max_xy, vx))
        vy = max(-self.max_xy, min(self.max_xy, vy))
        wz = max(-self.max_yaw, min(self.max_yaw, wz))
        if self.dry:
            return vx, vy, wz
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = vx; m.twist.linear.y = vy; m.twist.angular.z = wz
        self.pub.publish(m)
        return vx, vy, wz

    def tick(self):
        if self.done or self.pose is None or self.origin is None:
            return
        if self.start_t is None:
            self.start_t = time.time()
        t = time.time() - self.start_t
        T_end = self.path[-1, 0]

        if self.mode == "open":
            if t > T_end:
                self.finish(); return
            vx = np.interp(t, self.cmds[:, 0], self.cmds[:, 1])
            vy = np.interp(t, self.cmds[:, 0], self.cmds[:, 2])
            wz = np.interp(t, self.cmds[:, 0], self.cmds[:, 3])
            v = self.send(vx, vy, wz)
            self.get_logger().info(f"[开环] t={t:4.1f}/{T_end:.1f}s cmd=({v[0]:+.2f},{v[1]:+.2f},{v[2]:+.2f})")
            return

        if t > T_end + 1.0:
            self.finish(); return
        dx, dy, dyaw = self.rel_path_target(min(t + self.lookahead, T_end))
        ox, oy, oyaw = self.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        tx = ox + c*dx - s*dy
        ty = oy + s*dx + c*dy
        tyaw = wrap(oyaw + dyaw)
        with self.lock:
            x, y, yaw = self.pose
        ex_w, ey_w = tx - x, ty - y
        cb, sb = math.cos(-yaw), math.sin(-yaw)
        ex = cb*ex_w - sb*ey_w
        ey = sb*ex_w + cb*ey_w
        eyaw = wrap(tyaw - yaw)
        vx, vy, wz = self.send(self.kp_xy*ex, self.kp_xy*ey, self.kp_yaw*eyaw)
        dist_end = math.hypot(tx - x, ty - y)
        self.get_logger().info(f"[闭环] t={t:4.1f}/{T_end:.1f}s err=({ex*100:+.0f},{ey*100:+.0f})cm "
                               f"yaw={math.degrees(eyaw):+.0f}° cmd=({vx:+.2f},{vy:+.2f},{wz:+.2f})")
        if t >= T_end and dist_end < 0.05:
            self.finish()

    def finish(self):
        self.send(0.0, 0.0, 0.0)
        self.get_logger().info("✅ 回放结束，停车")
        self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--open", action="store_true", help="纯开环重放 cmd_vel")
    ap.add_argument("--dry", action="store_true", help="只打印不发 cmd（准备段也只打印）")
    ap.add_argument("--no-prepare", action="store_true",
                    help="跳过准备段（spine/双臂已就位时）")
    a = ap.parse_args()
    if not a.no_prepare:
        from ebim_nav.prepare_pose import run_prepare
        from ebim_nav.common import load_config
        if not run_prepare(load_config()["approach"], dry=a.dry):
            raise SystemExit("准备段失败，车不动（就位后可 --no-prepare 跳过重试）")
    rclpy.init()
    node = PBDReplay(a.name, "open" if a.open else "closed", a.dry)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send(0.0, 0.0, 0.0)


if __name__ == "__main__":
    main()
