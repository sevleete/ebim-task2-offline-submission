#!/usr/bin/env python3
from __future__ import annotations

import math
import time

from ebim_nav.common import load_config
from ebim_nav.local_map import rear_wall
from ebim_nav.utils.feed import CloudOdomFeed


def main():
    cfg = load_config()
    ac = cfg["approach"]
    feed = CloudOdomFeed("ebim_check_wall").start().wait_ready()
    last = 0.0
    print(f"== 后墙验证 ==  rear_len={ac['rear_len']}m  裁剪={ac['wall_crop']}（Ctrl+C 退出）")
    while True:
        got = feed.latest()
        if got is None or got[0] <= last:
            time.sleep(0.05)
            continue
        last, pts, _ = got
        rw = rear_wall(pts, ac["wall_crop"], ac["rear_len"])
        if rw is None:
            print("\r  未拟合到后墙（车尾窗口内点太少？）              ", end="")
        else:
            gap, eyaw = rw
            band = "✅在15~20cm带内" if 0.15 <= gap <= 0.20 else "              "
            print(f"\r  车尾净距 gap={gap*100:6.1f}cm   平行角={math.degrees(eyaw):+6.2f}°  {band}",
                  end="")
        time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
