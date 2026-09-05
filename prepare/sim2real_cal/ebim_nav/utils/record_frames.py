#!/usr/bin/env python3
from __future__ import annotations

import sys
import time

from ebim_nav.common import PKG_ROOT
from ebim_nav.utils.feed import CloudOdomFeed, save_frames

RATE = 10.0


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "lmap"
    out_dir = PKG_ROOT / "data"
    out_dir.mkdir(exist_ok=True)
    out = out_dir / f"{name}_frames.npz"

    feed = CloudOdomFeed("ebim_lmap_record").start().wait_ready()
    frames = []
    last_stamp = 0.0
    print(f"== 录制 local_map 帧 [{name}] ==  手柄开车，Ctrl+C 结束保存")
    try:
        while True:
            got = feed.latest()
            if got and got[0] > last_stamp:
                last_stamp = got[0]
                frames.append(got)
                od = got[2]
                print(f"\r  帧{len(frames):4d}  odom=({od[0]:+.2f},{od[1]:+.2f})  ", end="")
            time.sleep(1.0 / RATE)
    except KeyboardInterrupt:
        pass
    if len(frames) < 5:
        sys.exit("\n帧太少，没保存")
    save_frames(out, frames)
    print(f"\n已保存 {out}（{len(frames)} 帧）→ 离线回放: scripts/run.sh lmap_viz --npz {out}")


if __name__ == "__main__":
    main()
