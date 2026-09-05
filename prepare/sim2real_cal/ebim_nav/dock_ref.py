#!/usr/bin/env python3
from __future__ import annotations

import sys
import time

import numpy as np

from ebim_nav.common import PKG_ROOT, load_config
from ebim_nav.dock_lib import crop_box, front_leg_pair
from ebim_nav.local_map import voxel_down
from ebim_nav.utils.feed import CloudOdomFeed


def main():
    cfg = load_config()
    d = cfg["dock"]
    fc = d["front_crop"]
    legs = d["legs"]
    need = int(sys.argv[1]) if len(sys.argv) > 1 else 30

    feed = CloudOdomFeed("ebim_dock_ref").start().wait_ready()
    print(f"采集前2腿 y 中点：前区 x∈[{fc['x_min']},{fc['x_max']}] y∈[{fc['y_min']},{fc['y_max']}]，"
          f"目标 {need} 个有效帧（Ctrl+C 提前结束）")

    vals, last = [], 0.0
    try:
        while len(vals) < need:
            got = feed.latest()
            if got is None or got[0] <= last:
                time.sleep(0.02)
                continue
            last = got[0]
            front = voxel_down(crop_box(got[1], fc["x_min"], fc["x_max"], fc["y_min"], fc["y_max"]),
                               d["match_voxel"])
            r = front_leg_pair(front, legs)
            if r is not None:
                vals.append(r[0])
            print(f"\r  有效 {len(vals)}/{need}  本帧前2腿={'有 y中点%+.3f' % r[0] if r else '无'}   ", end="")
    except KeyboardInterrupt:
        print("\n手动结束采集")

    if len(vals) < 3:
        sys.exit(f"\n✗ 只稳定测到 {len(vals)} 帧前2腿——车正对桌了吗？front_crop 框住 2 条近腿了吗？"
                 f" 先 ./run.sh cloud_viz 看一眼。")
    mid_y = float(np.median(vals))
    std = float(np.std(vals))
    print(f"\n前2腿 y 中点 mid_y={mid_y:+.4f}m（{len(vals)}帧，帧间std={std * 100:.1f}cm）")
    if std > d["confirm_std"]:
        print(f"⚠️ 帧间抖动 {std*100:.1f}cm > confirm_std {d['confirm_std']*100:.0f}cm——"
              f"车没停稳或检测不稳，参考可能不准。")

    out = PKG_ROOT / d["ref_scan_file"]
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, mid_y=mid_y)
    print(f"已存 {out}")
    print("   独立验证: ./run.sh dock --dry   （挪开车再看它算出的 Δy 对不对）")


if __name__ == "__main__":
    import os
    try:
        main(); code = 0
    except SystemExit as e:
        code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
    sys.stdout.flush()
    os._exit(code)
