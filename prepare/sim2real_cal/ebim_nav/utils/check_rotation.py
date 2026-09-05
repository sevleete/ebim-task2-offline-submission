#!/usr/bin/env python3
from __future__ import annotations

import math
import time

from ebim_nav.common import load_config
from ebim_nav.local_map import LocalMap, wrap
from ebim_nav.utils.feed import CloudOdomFeed


def main():
    cfg = load_config()
    lm = LocalMap(cfg["local_map"])
    feed = CloudOdomFeed("ebim_check_rot").start().wait_ready()
    yaw_odom0 = None
    last = 0.0
    print("== 转角验证 ==  手柄原地转车（Ctrl+C 退出）")
    while True:
        got = feed.latest()
        if got is None or got[0] <= last:
            time.sleep(0.02)
            continue
        last, pts, odom = got
        p = lm.update(pts, odom)
        if yaw_odom0 is None:
            yaw_odom0 = odom[2]
        d_odom = math.degrees(wrap(odom[2] - yaw_odom0))
        print(f"\r  ICP={math.degrees(p.yaw):+7.2f}°   odom={d_odom:+7.2f}°   "
              f"差={math.degrees(p.yaw)-d_odom:+5.2f}°   fit={p.fitness:.2f}"
              f"{' ⚠ICP失效' if not p.ok else '        '}", end="")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
