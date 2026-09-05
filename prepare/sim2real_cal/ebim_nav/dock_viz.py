#!/usr/bin/env python3
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from ebim_nav.common import PKG_ROOT, load_config
from ebim_nav.dock_lib import crop_box, front_leg_pair
from ebim_nav.local_map import voxel_down
from ebim_nav.utils.feed import CloudOdomFeed


def main():
    cfg = load_config()
    d = cfg["dock"]
    fc = d["front_crop"]
    legs = d["legs"]
    tol = d["lat_tol"]
    try:
        ref_mid_y = float(np.load(PKG_ROOT / d["ref_scan_file"])["mid_y"])
    except (FileNotFoundError, KeyError):
        ref_mid_y = None

    feed = CloudOdomFeed("ebim_dock_viz").start().wait_ready()
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    print("实时 dock 可视化（关闭窗口退出）。绿=前2腿/当前中点，红=目标中点。")

    while plt.fignum_exists(fig.number):
        got = feed.latest()
        if got is None:
            plt.pause(0.05)
            continue
        front = voxel_down(crop_box(got[1], fc["x_min"], fc["x_max"], fc["y_min"], fc["y_max"]),
                           d["match_voxel"])
        r = front_leg_pair(front, legs)

        ax.clear()
        ax.add_patch(plt.Rectangle((fc["y_min"], fc["x_min"]),
                                   fc["y_max"] - fc["y_min"], fc["x_max"] - fc["x_min"],
                                   fill=False, ec="gray", ls="--", lw=1))
        if len(front):
            ax.scatter(front[:, 1], front[:, 0], s=8, c="lightgray")
        ax.scatter([0], [0], c="r", marker="^", s=160, label="car", zorder=5)

        if ref_mid_y is not None:
            ax.axvline(ref_mid_y, c="red", lw=2, label=f"target {ref_mid_y:+.3f}")
            ax.axvspan(ref_mid_y - tol, ref_mid_y + tol, color="red", alpha=0.10)

        if r is not None:
            mid_y, la, lb = r
            ax.scatter([la[1], lb[1]], [la[0], lb[0]], s=90, c="green",
                       edgecolors="k", zorder=6, label="front legs")
            ax.plot([la[1], lb[1]], [la[0], lb[0]], "g-", lw=1.5)
            ax.axvline(mid_y, c="green", lw=2, label=f"mid_y {mid_y:+.3f}")
            if ref_mid_y is not None:
                dy = ref_mid_y - mid_y
                arrow = "→右" if dy < 0 else "←左"
                ok = "✓到位" if abs(dy) < tol else f"需{arrow} {abs(dy)*100:.1f}cm"
                ax.set_title(f"前2腿✓  当前 mid_y={mid_y:+.3f}  目标={ref_mid_y:+.3f}  Δy={dy:+.3f}m  {ok}")
            else:
                ax.set_title(f"前2腿✓ mid_y={mid_y:+.3f}（无参考，先 ./run.sh dock_ref）")
        else:
            ax.set_title("未识别到前2腿（车没对着桌 / 出框 / 间距不符 1.225m）")

        ax.set_xlabel("y  (左 +  /  右 -)  [m]")
        ax.set_ylabel("x  (前方) [m]")
        ax.invert_xaxis()
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
        plt.pause(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    import os
    os._exit(0)
