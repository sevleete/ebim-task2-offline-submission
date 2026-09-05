#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time

import matplotlib.pyplot as plt
import numpy as np

from ebim_nav.common import load_config
from ebim_nav.icp2d import se2_matrix, se2_params, transform_points
from ebim_nav.local_map import LocalMap, rear_wall, se2_inv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", help="离线回放 record_frames 录的帧序列")
    ap.add_argument("--rate", type=float, default=8.0, help="刷新频率 [Hz]")
    a = ap.parse_args()

    cfg = load_config()
    lm = LocalMap(cfg["local_map"])
    ac = cfg["approach"]

    if a.npz:
        from ebim_nav.utils.feed import load_frames
        frames = iter(load_frames(a.npz))
        get = lambda: next(frames, None)
        print(f"离线回放 {a.npz}")
    else:
        from ebim_nav.utils.feed import CloudOdomFeed
        feed = CloudOdomFeed("ebim_lmap_viz").start().wait_ready()
        state = {"last": 0.0}

        def get():
            while True:
                got = feed.latest()
                if got and got[0] > state["last"]:
                    state["last"] = got[0]
                    return got
                time.sleep(0.02)

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 9))
    trail_icp, trail_odom = [], []
    odom0_inv = None

    while True:
        got = get()
        if got is None:
            print("回放结束（窗口保留，Ctrl+C 退出）")
            plt.ioff()
            plt.show()
            return
        _, pts, odom = got
        p = lm.update(pts, odom)

        if odom0_inv is None:
            odom0_inv = se2_inv(se2_matrix(*odom))
        ox, oy, _ = se2_params(odom0_inv @ se2_matrix(*odom))
        trail_icp.append((p.x, p.y))
        trail_odom.append((ox, oy))

        rw = rear_wall(pts, ac["wall_crop"], ac["rear_len"])

        ax.cla()
        ax.set_aspect("equal")
        ax.grid(True, lw=0.3)
        if lm.map_pts is not None:
            ax.plot(*lm.map_pts.T, ".", ms=1.5, color="0.6", label=f"map {len(lm.map_pts)}pt")
        cur_map = transform_points(lm._prep(pts), p.T)
        ax.plot(*cur_map.T, ".", ms=2, color="tab:blue", alpha=0.6, label="当前帧")
        ti, to = np.array(trail_icp), np.array(trail_odom)
        ax.plot(*ti.T, "-", color="tab:green", lw=1.5, label="ICP 轨迹")
        ax.plot(*to.T, "--", color="tab:orange", lw=1.2, label="odom 轨迹")
        ax.arrow(p.x, p.y, 0.4 * math.cos(p.yaw), 0.4 * math.sin(p.yaw),
                 head_width=0.09, color="tab:green")
        if rw is not None:
            gx = -(rw[0] + ac["rear_len"])
            seg = np.array([[gx, -2.0], [gx, 2.0]])
            c, s = math.cos(-rw[1]), math.sin(-rw[1])
            seg = seg @ np.array([[c, -s], [s, c]]).T
            ax.plot(*transform_points(seg, p.T).T, "r-", lw=2, label="后墙拟合")
        drift = math.hypot(p.x - ox, p.y - oy)
        ax.set_title(
            f"pose=({p.x:+.2f},{p.y:+.2f},{math.degrees(p.yaw):+.1f}°) "
            f"fit={p.fitness:.2f}{'' if p.ok else ' ⚠ICP失效→odom'}   odom漂移={drift*100:.0f}cm\n"
            + (f"后墙 gap={rw[0]*100:.1f}cm  平行角={math.degrees(rw[1]):+.1f}°"
               if rw else "后墙: 未拟合到"))
        ax.legend(loc="upper right", fontsize=8)
        plt.pause(0.001 if a.npz else max(0.001, 1.0 / a.rate))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
