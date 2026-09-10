#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time

from ebim_nav.common import load_config
from ebim_nav.dock_lib import DockLateral
from ebim_nav.local_map import LocalMap, rear_wall, wrap
from ebim_nav.prepare_pose import run_prepare
from ebim_nav.utils.feed import CloudOdomFeed

STAGES = ["prepare", "rotate", "backup", "strafe"]
RATE = 20.0


class StepByStep:
    def __init__(self, args):
        cfg = load_config()
        self.cfg = cfg
        self.a = cfg["approach"]
        self.lm = LocalMap(cfg["local_map"])
        self.dry = args.dry
        self.dir = +1.0 if (args.ccw or (not args.cw and self.a["rotate_dir"] == "ccw")) else -1.0
        self.feed = CloudOdomFeed("ebim_step_by_step").start().wait_ready()
        self.last_stamp = 0.0


    def send(self, vx, vy, wz):
        if self.dry:
            return vx, vy, wz
        return self.feed.send(vx, vy, wz, self.a["max_vel_xy"], self.a["max_vel_yaw"])

    def stop(self):
        if not self.dry:
            self.feed.stop_cmd()

    def step_lm(self):
        got = self.feed.latest()
        if got is None or got[0] <= self.last_stamp:
            return None
        self.last_stamp, self.pts, odom = got
        return self.lm.update(self.pts, odom)

    def warmup(self, n=5):
        k = 0
        while k < n:
            if self.step_lm() is not None:
                k += 1
            time.sleep(0.02)


    def st_prepare(self) -> bool:
        return run_prepare(self.a, dry=self.dry)


    def st_rotate(self) -> bool:
        a = self.a
        self.warmup()
        yaw0 = self.lm.pose[2]
        odom_yaw0 = self.feed.latest()[2][2]
        target = yaw0 + self.dir * math.radians(a["rotate_target_deg"])
        switch = math.radians(a["rotate_coarse_switch_deg"])
        max_turn = math.radians(a["rotate_target_deg"] + a["rotate_max_extra_deg"])
        print(f"旋转 {'+' if self.dir > 0 else '-'}{a['rotate_target_deg']:.0f}°："
              f"粗转(ICP)→后缘对墙精对(eyaw)")
        phase, seen, settle, miss, t0 = "coarse", 0, 0, 0, time.time()
        while time.time() - t0 < a["stage_timeout"]:
            p = self.step_lm()
            if p is None:
                time.sleep(1.0 / RATE / 2)
                continue
            rw = rear_wall(self.pts, a["wall_crop"], a["rear_len"])
            turned = wrap(p.yaw - yaw0)
            err90 = wrap(target - p.yaw)

            if phase == "coarse":
                wz = a["kp_yaw"] * err90 * (0.3 if not p.ok else 1.0)
                seen = seen + 1 if rw is not None else 0
                if abs(err90) < switch and seen >= a["rotate_wall_seen_frames"]:
                    phase = "fine"
                    print("\n  ↳ 后墙锁定，切精对（eyaw 判后缘对墙平行）")
            else:
                if rw is None:
                    miss += 1
                    wz = 0.0
                    if miss > 40:
                        self.stop(); print("\n❌ 精对阶段丢失后墙"); return False
                else:
                    miss, wz = 0, a["kp_yaw_wall"] * rw[1]
                    settle = settle + 1 if abs(rw[1]) < math.radians(a["rotate_tol_deg"]) else 0
                    if settle >= a["settle_frames"]:
                        self.stop()
                        print(f"\n✅ 后缘对墙平齐 eyaw={math.degrees(rw[1]):+.1f}°"
                              f"（共转 {math.degrees(turned):+.1f}°）")
                        return True
                if abs(turned) > max_turn:
                    self.stop(); print("\n❌ 精对越界（疑似锁错墙）"); return False

            v = self.send(0.0, 0.0, wz)
            d_odom = math.degrees(wrap(self.feed.latest()[2][2] - odom_yaw0))
            wall_s = f"墙eyaw={math.degrees(rw[1]):+5.1f}°" if rw is not None else "墙 --   "
            print(f"\r  [{'粗转' if phase == 'coarse' else '精对'}] "
                  f"icp={math.degrees(turned):+6.1f}° odom={d_odom:+6.1f}° "
                  f"剩{math.degrees(err90):+5.1f}° {wall_s} wz={v[2]:+.2f}   ", end="")
        self.stop()
        print("\n❌ rotate 超时")
        return False


    def st_backup(self) -> bool:
        a = self.a
        self.warmup()
        print(f"倒车至车尾距墙 {a['rear_gap_target']*100:.0f}cm")
        settle, t0, miss = 0, time.time(), 0
        while time.time() - t0 < a["stage_timeout"]:
            if self.step_lm() is None:
                time.sleep(1.0 / RATE / 2)
                continue
            rw = rear_wall(self.pts, a["wall_crop"], a["rear_len"])
            if rw is None:
                miss += 1
                self.send(0.0, 0.0, 0.0)
                if miss > 40:
                    print("\n❌ 连续拟合不到后墙")
                    self.stop()
                    return False
                continue
            miss = 0
            gap, eyaw = rw
            err = gap - a["rear_gap_target"]
            vx = -a["kp_xy"] * err
            if gap < a["rear_gap_min"]:
                vx = max(vx, 0.02)
            vx = min(vx, 0.05)
            v = self.send(vx, 0.0, a["kp_yaw_wall"] * eyaw)
            print(f"\r  gap={gap*100:5.1f}cm 平行差={math.degrees(eyaw):+5.1f}° "
                  f"vx={v[0]:+.2f} wz={v[2]:+.2f}   ", end="")
            settle = settle + 1 if abs(err) < a["rear_gap_tol"] else 0
            if settle >= a["settle_frames"]:
                self.stop()
                print(f"\n✅ 倒车到位，车尾距墙 {gap*100:.1f}cm")
                return True
        self.stop()
        print("\n❌ backup 超时")
        return False


    def st_strafe(self) -> bool:
        a = self.a
        self.warmup()
        d = self.cfg["dock"]
        try:
            dock = DockLateral(self.cfg)
            print(f"盲走向右走完整段 {a['strafe_dist']}m（全程保后墙21cm+平行）→ 停 → dock 只调左右")
        except FileNotFoundError as e:
            dock = None
            print(f"⚠️ dock 参考不可用（{e}），只盲走 {a['strafe_dist']}m")

        od0 = self.feed.latest()[2]
        t0 = time.time()
        while time.time() - t0 < a["stage_timeout"]:
            p = self.step_lm()
            if p is None:
                time.sleep(1.0 / RATE / 2)
                continue
            od = self.feed.latest()[2]
            moved = math.hypot(od[0] - od0[0], od[1] - od0[1])
            if moved >= a["strafe_dist"]:
                self.stop()
                print(f"\n✅ 盲走完成 {moved:.2f}m")
                break

            vx, wz = 0.0, 0.0
            rw = rear_wall(self.pts, a["wall_crop"], a["rear_len"])
            if rw is not None:
                vx = min(0.05, max(-0.05, -a["kp_xy"] * (rw[0] - a["rear_gap_target"])))
                wz = a["kp_yaw_wall"] * rw[1]
            v = self.send(vx, -a["max_vel_xy"], wz)
            print(f"\r  [盲走] {moved:.2f}/{a['strafe_dist']:.2f}m "
                  f"cmd=({v[0]:+.2f},{v[1]:+.2f},{v[2]:+.2f})   ", end="")
        else:
            self.stop()
            print("\n❌ strafe(盲走) 超时")
            return False

        if dock is None:
            return True
        if self.dry:
            print("[DRY] 跳过 dock 实际横移（可 ./run.sh dock --dry 单独看 Δy）")
            return True
        return dock.run(self.feed)


    def run(self, start_from: str, only: str | None, auto: bool):
        todo = [only] if only else STAGES[STAGES.index(start_from):]
        print(f"步骤: {' → '.join(todo)}  旋转方向={'逆时针' if self.dir > 0 else '顺时针'}"
              f"{' [DRY]' if self.dry else ''}")
        for i, st in enumerate(todo):
            if not auto:
                input(f"\n—— 回车执行 [{st}]（Ctrl+C 退出）——")
            else:
                if i > 0:
                    time.sleep(1.0)
                print(f"\n—— [{st}] ——")
            if not getattr(self, f"st_{st}")():
                sys.exit(f"流程中止于 [{st}]（修好后可 --from {st} 续跑）")
        print("\n🎉 趋近流程全部完成")


def main():
    ap = argparse.ArgumentParser(description="")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--cw", action="store_true", help="顺时针转 90°")
    g.add_argument("--ccw", action="store_true", help="逆时针转 90°")
    ap.add_argument("--from", dest="start_from", choices=STAGES, default="prepare")
    ap.add_argument("--only", choices=STAGES, help="只跑这一步")
    ap.add_argument("--auto", action="store_true", help="步骤间不等回车")
    ap.add_argument("--dry", action="store_true", help="只打印不发 cmd_vel/tmrctl")
    args = ap.parse_args()
    node = StepByStep(args)
    try:
        node.run(args.start_from, args.only, args.auto)
    except KeyboardInterrupt:
        print("\n手动中断，停车")
    finally:
        node.stop()


if __name__ == "__main__":
    import os
    try:
        main(); _code = 0
    except SystemExit as e:
        _code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
    sys.stdout.flush()
    os._exit(_code)
