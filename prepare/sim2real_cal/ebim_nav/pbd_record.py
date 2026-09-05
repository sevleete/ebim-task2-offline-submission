#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
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


class PBDRecord(Node):
    def __init__(self, cfg):
        super().__init__("ebim_pbd_record")
        self.lock = threading.Lock()
        self.t0 = None
        self.cmds = []
        self.odom = []
        self.create_subscription(TwistStamped, cfg["base"]["cmd_vel_topic"],
                                 self.on_cmd, qos_profile_sensor_data)
        self.create_subscription(Odometry, cfg["base"]["odom_topic"],
                                 self.on_odom, qos_profile_sensor_data)

    def _t(self):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def on_cmd(self, m):
        v = m.twist
        with self.lock:
            self.cmds.append((self._t(), v.linear.x, v.linear.y, v.angular.z))

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self.odom.append((self._t(), p.x, p.y, quat_yaw(q)))


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "pbd_task"
    cfg = load_config()
    rclpy.init()
    node = PBDRecord(cfg)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f"== PBD 录制 [{name}] ==  手柄开一遍，Ctrl+C 结束保存")
    try:
        while True:
            with node.lock:
                nc, no = len(node.cmds), len(node.odom)
                last = node.odom[-1] if node.odom else None
            if last:
                print(f"\r  t={last[0]:5.1f}s  cmd帧{nc} odom帧{no}  "
                      f"pos=({last[1]:+.2f},{last[2]:+.2f}) yaw={math.degrees(last[3]):+.0f}°  ", end="")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    with node.lock:
        cmds = np.array(node.cmds) if node.cmds else np.empty((0, 4))
        odom = np.array(node.odom) if node.odom else np.empty((0, 4))
    if len(odom) < 2:
        sys.exit("\n没录到 odom，检查底盘是否在跑")
    out = PKG_ROOT / "paths" / f"{name}_pbd.npz"
    out.parent.mkdir(exist_ok=True)
    np.savez(out, cmds=cmds, odom=odom)
    dur = odom[-1, 0]
    dist = np.sum(np.hypot(np.diff(odom[:, 1]), np.diff(odom[:, 2])))
    vmax = np.abs(cmds[:, 1:]).max() if len(cmds) else 0
    print(f"\n已保存 {out.name}: 时长{dur:.1f}s 行程{dist:.2f}m 峰值速度{vmax:.3f} "
          f"cmd{len(cmds)} odom{len(odom)}")
    print(f"看轨迹: ./run.sh pbd_viz {name}   回放对比: ./run.sh pbd_viz {name} --live")


if __name__ == "__main__":
    main()
