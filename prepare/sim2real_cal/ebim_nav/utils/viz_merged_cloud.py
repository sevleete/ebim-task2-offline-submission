#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from ebim_nav.common import load_config
from ebim_nav.leg_detect import detect_legs


class CloudViz(Node):
    def __init__(self):
        super().__init__("ebim_cloud_viz")
        cfg = load_config()
        self.topic = cfg["merger"]["out_cloud_topic"]
        self.base = cfg["frames"]["base"]
        self.rear_len = cfg.get("approach", {}).get("rear_len", 0.40)
        self.wall_crop = cfg.get("approach", {}).get("wall_crop", None)
        self.lock = threading.Lock()
        self.pts = None
        self.stamp = 0.0
        self.create_subscription(PointCloud2, self.topic, self._on_cloud,
                                 qos_profile_sensor_data)

    def _on_cloud(self, msg):
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y"))
        with self.lock:
            self.pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
            self.stamp = time.time()

    def latest(self):
        with self.lock:
            return (self.stamp, self.pts) if self.pts is not None else None


def main():
    ap = argparse.ArgumentParser(description="")
    ap.add_argument("--range", type=float, default=3.0, help="视窗半宽 [m]（默认 3）")
    ap.add_argument("--rate", type=float, default=10.0, help="刷新率 [Hz]（默认 10）")
    ap.add_argument("--size", type=float, default=6.0, help="点大小（桌腿细，可调大）")
    ap.add_argument("--detect", action="store_true", help="叠加桌腿检测（聚类+矩形拟合）")
    args = ap.parse_args()

    rclpy.init()
    node = CloudViz()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(f"等 {node.topic} …（scan_merger 和雷达在跑吗？）")
    t0 = time.time()
    while node.latest() is None:
        if time.time() - t0 > 15:
            print("✗ 15s 没收到点云：确认 ./run.sh scan_merger 在跑、雷达已起")
            return
        time.sleep(0.1)
    print("✓ 收到点云，开窗。窗口里按 s 存图；Ctrl+C 退出。")

    saved = {"n": 0}

    def on_key(ev):
        if ev.key == "s":
            fn = f"/tmp/merged_cloud_{saved['n']}.png"
            fig.savefig(fn, dpi=140)
            saved["n"] += 1
            print(f"  已存 {fn}")

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 9))
    fig.canvas.mpl_connect("key_press_event", on_key)
    R = args.range
    det_cfg = load_config()["dock"]["detect"] if args.detect else None
    lock = {"rect": None, "miss": 0}

    try:
        while rclpy.ok():
            got = node.latest()
            if got:
                _, pts = got
                ax.clear()
                if pts is not None and len(pts):
                    x, y = pts[:, 0], pts[:, 1]
                    rng = np.hypot(x, y)
                    ax.scatter(y, x, s=args.size, c=rng, cmap="viridis",
                               vmin=0, vmax=R * 1.2, linewidths=0)
                ax.plot(0, 0, "r+", ms=14, mew=2)
                ax.annotate("", xy=(0, 0.4), xytext=(0, 0),
                            arrowprops=dict(arrowstyle="->", color="red", lw=2))
                ax.plot([0.3, -0.3], [-node.rear_len, -node.rear_len],
                        "r--", lw=1, alpha=0.6)
                if node.wall_crop:
                    wc = node.wall_crop
                    ax.add_patch(plt.Rectangle(
                        (-wc["y_half"], wc["x_min"]),
                        2 * wc["y_half"], wc["x_max"] - wc["x_min"],
                        fill=False, ec="orange", ls=":", lw=1.2, alpha=0.7))

                if args.detect:
                    res = detect_legs(pts, det_cfg)
                    rect, hold = res.rect, False
                    if rect is None and lock["rect"] is not None and lock["miss"] < det_cfg["hold_frames"]:
                        rect, hold = lock["rect"], True
                        lock["miss"] += 1
                    elif rect is not None:
                        lock["rect"], lock["miss"] = rect, 0
                    else:
                        lock["rect"] = None
                    if rect is not None:
                        col = "yellow" if hold else "lime"
                        for c in rect.clusters:
                            ax.scatter(c.pts[:, 1], c.pts[:, 0], s=args.size * 2,
                                       c="red", linewidths=0, zorder=5)
                            ax.add_patch(plt.Circle((c.centroid[1], c.centroid[0]),
                                        det_cfg["leg_max_extent"], fill=False,
                                        ec="red", lw=1.5, zorder=5))
                        poly = rect.corners
                        ax.plot(np.append(poly[:, 1], poly[0, 1]),
                                np.append(poly[:, 0], poly[0, 0]),
                                "-", color=col, lw=2.0, zorder=6)
                        ax.plot(rect.center[1], rect.center[0], "x",
                                color=col, ms=10, mew=2, zorder=6)
                        txt = (f"legs {rect.short_side*100:.1f}×{rect.long_side*100:.1f} cm"
                               f"  ctr=({rect.center[0]:+.2f},{rect.center[1]:+.2f})"
                               f"  yaw={math.degrees(rect.yaw):+.1f}°  score={rect.score:.2f}"
                               + ("  [HOLD]" if hold else ""))
                    else:
                        txt = f"no lock  (候选腿 {len(res.candidates)} / 簇 {res.n_clusters})"
                    ax.text(0.02, 0.98, txt, transform=ax.transAxes, va="top", ha="left",
                            fontsize=10, color="white",
                            bbox=dict(fc="black", alpha=0.6, pad=3), zorder=7)

                ax.set_xlim(R, -R)
                ax.set_ylim(-R, R)
                ax.set_aspect("equal")
                ax.set_xlabel("← 左  y [m]  右 →")
                ax.set_ylabel("← 后  x [m]  前(车头) →")
                ax.set_title(f"merged_cloud（{node.base} 系）  "
                             f"{0 if pts is None else len(pts)} 点   红+=车  橙:=后墙窗")
                ax.grid(which="major", lw=0.6, alpha=0.5)
                ax.grid(which="minor", lw=0.3, alpha=0.25)
                ax.set_xticks(np.arange(-R, R + 0.01, 0.5))
                ax.set_yticks(np.arange(-R, R + 0.01, 0.5))
                ax.set_xticks(np.arange(-R, R + 0.01, 0.1), minor=True)
                ax.set_yticks(np.arange(-R, R + 0.01, 0.1), minor=True)
            plt.pause(max(0.001, 1.0 / args.rate))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
