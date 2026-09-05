#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import threading
import time

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from ebim_nav.common import PKG_ROOT, load_config

for _f in ("Noto Sans CJK SC", "WenQuanYi Micro Hei", "AR PL UKai CN", "AR PL UMing CN"):
    if any(x.name == _f for x in matplotlib.font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.sans-serif"] = [_f, "DejaVu Sans"]
        break
matplotlib.rcParams["axes.unicode_minus"] = False


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def to_rel(path: np.ndarray) -> np.ndarray:
    _, x0, y0, yaw0 = path[0]
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    dx, dy = path[:, 1] - x0, path[:, 2] - y0
    return np.column_stack([path[:, 0],
                            c * dx - s * dy,
                            s * dx + c * dy,
                            np.unwrap(path[:, 3]) - yaw0])


def draw_recorded(ax, rel: np.ndarray):
    dist = float(np.sum(np.hypot(np.diff(rel[:, 1]), np.diff(rel[:, 2]))))
    ax.plot(rel[:, 1], rel[:, 2], "-", color="0.55", lw=2,
            label=f"录制 {dist:.2f}m/{rel[-1,0]:.0f}s")
    q = rel[:: max(1, len(rel) // 18)]
    ax.quiver(q[:, 1], q[:, 2], np.cos(q[:, 3]), np.sin(q[:, 3]),
              color="0.35", angles="xy", scale_units="xy", scale=12, width=0.004)
    ax.plot(rel[0, 1], rel[0, 2], "o", color="tab:blue", ms=9, label="起点")
    ax.plot(rel[-1, 1], rel[-1, 2], "*", color="tab:red", ms=14, label="录制终点")
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_xlabel("x [m]（起点系）")
    ax.set_ylabel("y [m]")


def pose_arrow(ax, x, y, yaw, color):
    return ax.arrow(x, y, 0.15 * math.cos(yaw), 0.15 * math.sin(yaw),
                    head_width=0.05, color=color, zorder=5)


def main():
    ap = argparse.ArgumentParser(description="PbD 轨迹可视化（静态/回放对比）")
    ap.add_argument("name")
    ap.add_argument("--live", action="store_true", help="订阅 odom 实时叠加执行轨迹")
    ap.add_argument("--rate", type=float, default=8.0, help="live 刷新频率 [Hz]")
    a = ap.parse_args()

    f = PKG_ROOT / "paths" / f"{a.name}_pbd.npz"
    if not f.exists():
        raise SystemExit(f"找不到 {f}（先 pbd_record {a.name}）")
    rel = to_rel(np.load(f)["odom"])

    if not a.live:
        fig, ax = plt.subplots(figsize=(8, 8))
        draw_recorded(ax, rel)
        ax.set_title(f"PbD 录制轨迹 [{a.name}]")
        ax.legend(loc="best")
        plt.show()
        return

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    cfg = load_config()
    lock = threading.Lock()
    state = {"origin": None, "trail": [], "cur": None}

    class OdomSub(Node):
        def __init__(self):
            super().__init__("ebim_pbd_viz")
            self.create_subscription(Odometry, cfg["base"]["odom_topic"],
                                     self.cb, qos_profile_sensor_data)

        def cb(self, m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = quat_yaw(q)
            with lock:
                if state["origin"] is None:
                    state["origin"] = (p.x, p.y, yaw)
                    print(f"live 原点锁定 ({p.x:.2f},{p.y:.2f})——现在可以起 pbd_replay 了")
                ox, oy, oyaw = state["origin"]
                c, s = math.cos(-oyaw), math.sin(-oyaw)
                xr = c * (p.x - ox) - s * (p.y - oy)
                yr = s * (p.x - ox) + c * (p.y - oy)
                state["cur"] = (xr, yr, yaw - oyaw)
                tr = state["trail"]
                if not tr or math.hypot(xr - tr[-1][0], yr - tr[-1][1]) > 0.005:
                    tr.append((xr, yr))

    rclpy.init()
    node = OdomSub()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    try:
        while plt.fignum_exists(fig.number):
            with lock:
                trail = np.array(state["trail"]) if state["trail"] else None
                cur = state["cur"]
            ax.cla()
            draw_recorded(ax, rel)
            title = f"PbD 回放对比 [{a.name}] — 等 odom..."
            if trail is not None and cur is not None:
                ax.plot(trail[:, 0], trail[:, 1], "-", color="tab:green", lw=2,
                        label=f"执行 {len(trail)}pt")
                pose_arrow(ax, cur[0], cur[1], cur[2], "tab:green")
                dmin = float(np.min(np.hypot(rel[:, 1] - cur[0], rel[:, 2] - cur[1])))
                dend = float(math.hypot(rel[-1, 1] - cur[0], rel[-1, 2] - cur[1]))
                title = (f"PbD 回放对比 [{a.name}]   偏离录制线 {dmin*100:.1f}cm   "
                         f"距录制终点 {dend*100:.2f}m")
            ax.set_title(title)
            ax.legend(loc="best", fontsize=9)
            plt.pause(max(0.02, 1.0 / a.rate))
    except KeyboardInterrupt:
        pass

    with lock:
        cur = state["cur"]
    if cur is not None:
        dend = math.hypot(rel[-1, 1] - cur[0], rel[-1, 2] - cur[1])
        dyaw = math.degrees((cur[2] - rel[-1, 3] + math.pi) % (2 * math.pi) - math.pi)
        print(f"\n终点误差: {dend*100:.1f}cm / {dyaw:+.1f}°（相对录制终点）")
        out = PKG_ROOT / "data"
        out.mkdir(exist_ok=True)
        png = out / f"pbd_viz_{a.name}_{time.strftime('%m%d_%H%M%S')}.png"
        fig.savefig(png, dpi=110)
        print(f"对比图已存 {png}")


if __name__ == "__main__":
    main()
