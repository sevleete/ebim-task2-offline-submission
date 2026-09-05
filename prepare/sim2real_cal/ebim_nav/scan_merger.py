#!/usr/bin/env python3
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import StaticTransformBroadcaster

from ebim_nav.common import (EXTR_FILE, load_config, load_extrinsics, planar_apply,
                             points_to_virtual_scan, scan_to_points)


def tf_to_mat4(tf_msg) -> np.ndarray:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [t.x, t.y, t.z]
    return M


class ScanMerger(Node):
    def __init__(self):
        super().__init__("ebim_scan_merger")
        self.cfg = load_config()
        mcfg = self.cfg["merger"]
        self.base_frame = self.cfg["frames"]["base"]
        self.pair_max_dt = mcfg["pair_max_dt"]
        self.n_bins = mcfg["virtual_scan_bins"]
        self.range_max = mcfg["virtual_scan_range_max"]
        self.clip_min = mcfg["range_clip_min"]

        self.tf_buffer = None

        self.static_bc = StaticTransformBroadcaster(self)
        static_tfs = []
        for lid in self.cfg["lidars"].values():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = lid["mount_frame"]
            t.child_frame_id = lid["frame"]
            t.transform.rotation.w = 1.0
            static_tfs.append(t)
        self.static_bc.sendTransform(static_tfs)

        self.extrinsics = load_extrinsics()
        self.get_logger().info(f"外参来源: {'标定文件' if EXTR_FILE.exists() else 'URDF标称'}")

        self.latest: dict[str, LaserScan] = {}
        for name, lid in self.cfg["lidars"].items():
            self.create_subscription(
                LaserScan, lid["scan_topic"],
                lambda msg, n=name: self.on_scan(n, msg),
                qos_profile_sensor_data)

        self.pub_cloud = self.create_publisher(PointCloud2, mcfg["out_cloud_topic"], 5)
        self.pub_scan = self.create_publisher(LaserScan, mcfg["out_scan_topic"], 5)
        self.get_logger().info("scan_merger up: 等雷达数据 + TF ...")

    def on_scan(self, name: str, msg: LaserScan):
        self.latest[name] = msg
        if len(self.latest) < len(self.cfg["lidars"]):
            return
        stamps = {n: Time.from_msg(m.header.stamp) for n, m in self.latest.items()}
        ts = list(stamps.values())
        if abs((ts[0] - ts[1]).nanoseconds) * 1e-9 > self.pair_max_dt:
            return
        if stamps[name] != max(ts):
            return
        self.merge_and_publish()

    def merge_and_publish(self):
        merged = []
        for name, msg in self.latest.items():
            pts2 = scan_to_points(msg, self.clip_min)
            merged.append(planar_apply(pts2, self.extrinsics[name]))
        pts = np.vstack(merged)

        newest = max(self.latest.values(), key=lambda m: Time.from_msg(m.header.stamp).nanoseconds)
        header = newest.header
        header.frame_id = self.base_frame

        cloud_pts = np.column_stack([pts, np.zeros(len(pts))]).astype(np.float32)
        self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(header, cloud_pts))

        ranges, ang_min, ang_inc = points_to_virtual_scan(pts, self.n_bins, self.range_max)
        out = LaserScan()
        out.header = header
        out.angle_min = ang_min
        out.angle_max = ang_min + ang_inc * (self.n_bins - 1)
        out.angle_increment = ang_inc
        out.range_min = 0.05
        out.range_max = self.range_max
        out.ranges = ranges.astype(np.float32).tolist()
        self.pub_scan.publish(out)


def main():
    rclpy.init()
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
