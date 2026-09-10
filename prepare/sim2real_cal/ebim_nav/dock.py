#!/usr/bin/env python3
from __future__ import annotations

import argparse

from ebim_nav.common import load_config
from ebim_nav.dock_lib import DockLateral
from ebim_nav.utils.feed import CloudOdomFeed


def main():
    ap = argparse.ArgumentParser(description="")
    ap.add_argument("--dry", action="store_true", help="只测不横移")
    args = ap.parse_args()

    cfg = load_config()
    try:
        dock = DockLateral(cfg)
    except FileNotFoundError as e:
        raise SystemExit(f"✗ {e}")
    feed = CloudOdomFeed("ebim_dock").start().wait_ready()
    print(f"dock（只调左右）：目标前2腿 y 中点={dock.ref_mid_y:+.3f}，容差 {dock.tol*100:.0f}cm，"
          f"速度 {dock.speed} m/s{' [DRY]' if args.dry else ''}")
    ok = dock.run(feed, dry=args.dry)
    print("完成" if ok else "未收敛/失败")


if __name__ == "__main__":
    import os
    import sys
    try:
        main(); code = 0
    except SystemExit as e:
        code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
    sys.stdout.flush()
    os._exit(code)
