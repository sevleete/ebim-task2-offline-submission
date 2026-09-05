from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import yaml

_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_DIR))

from smoother import config as sconfig


def _make_smoother(scfg_path: str, cycle_time: float):
    scfg = sconfig.load(scfg_path)
    scfg.cycle_time = cycle_time
    try:
        return sconfig.build(scfg), scfg.impl
    except Exception as e:
        from smoother.clamp_smoother import ClampSmoother
        print(f"[relay] {scfg.impl} 不可用({e}),退化 clamp 平滑器", flush=True)
        return ClampSmoother(scfg.limits, cycle_time), "clamp"


class _Side:

    def __init__(self, node, side: str, mode: str, cfg: dict, scfg_path: str):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self.side = side
        self.mode = mode
        self.cycle = 1.0 / float(cfg["rate_hz"])
        self.watchdog_s = float(cfg["watchdog_seconds"])
        self.max_engage_off = float(cfg.get("max_engage_offset_rad", 1.0))
        self.align_done = float(cfg.get("align_done_rad", 0.08))
        self.go_blend_s = float(cfg.get("go_blend_seconds", 0.5))
        self.smoother, self.impl = _make_smoother(scfg_path, self.cycle)

        self.lock = threading.Lock()
        self.gello_q: list[float] | None = None
        self.gello_names: list[str] = []
        self.gello_t = 0.0
        self.robot_q: list[float] | None = None
        self.robot_names: list[str] = []
        self.state = "idle"
        self.offset: list[float] | None = None
        self.go_t0 = 0.0
        self.ready_target: list[float] | None = None
        self.freeze_target: list[float] | None = None
        self.hold_target: list[float] | None = None
        self._last_cmd: list[float] | None = None
        self._warn_t = 0.0

        raw = cfg["raw_topic"].format(side=side)
        robot = cfg["robot_joint_topic"].format(side=side)
        out = cfg["out_topic"].format(side=side)
        if self.mode == "teleop":
            node.create_subscription(JointState, raw, self._on_gello, 10)
        node.create_subscription(JointState, robot, self._on_robot,
                                 qos_profile_sensor_data)
        self.pub = node.create_publisher(JointState, out, 10)
        self.msg = JointState()
        self._clock = node.get_clock()
        node.get_logger().info(
            f"[{side}] mode={mode} "
            + (f"{raw} --{self.impl}--> {out}" if mode == "teleop"
               else f"HOLD --{self.impl}--> {out}")
            + f"(robot={robot}, {cfg['rate_hz']}Hz)")

    def _on_gello(self, m):
        with self.lock:
            self.gello_q = list(m.position)
            if not self.gello_names:
                self.gello_names = list(m.name)
            self.gello_t = time.monotonic()

    def _on_robot(self, m):
        with self.lock:
            self.robot_q = list(m.position)
            if not self.robot_names:
                self.robot_names = list(m.name)

    def cmd_go(self, log) -> None:
        if self.mode != "teleop":
            return
        with self.lock:
            gello_q = self.gello_q
            fresh = (time.monotonic() - self.gello_t) < self.watchdog_s
        if self.state != "ready":
            log(f"[{self.side}] 当前 {self.state},非就绪态,忽略 g")
            return
        if not fresh or gello_q is None:
            log(f"[{self.side}] GELLO 无数据,忽略 g")
            return
        base = self.ready_target or self._last_cmd
        off = [b - g for b, g in zip(base, gello_q)]
        worst = max(abs(o) for o in off)
        if worst > 0.5:
            log(f"[{self.side}] GELLO 已被挪远(Δ{worst:.2f} rad),"
                f"忽略 g;请摆回或 q 退出重来")
            return
        self.offset = off
        self.go_t0 = time.monotonic()
        self.state = "tracking"
        log(f"[{self.side}] ▶ 开始跟随(偏差 {worst:.3f} rad,"
            f"{self.go_blend_s}s 内消化)")

    def cmd_pause(self, log) -> None:
        if self.mode != "teleop" or self.state != "tracking":
            return
        self.ready_target = list(self._last_cmd or [])
        self.state = "ready"
        log(f"[{self.side}] ⏸ 暂停,保持当前位形(g+回车 继续)")

    def tick(self, now: float, log):
        with self.lock:
            gello_q = self.gello_q
            robot_q = self.robot_q
            fresh = (now - self.gello_t) < self.watchdog_s

        if self.mode == "hold":
            if self.state == "idle":
                if robot_q is None:
                    return
                self.hold_target = list(robot_q)
                self.smoother.reset(robot_q)
                self.state = "holding"
                log(f"[{self.side}] HOLD 锁定位形:"
                    + " ".join(f"{v:.3f}" for v in self.hold_target))
            self._publish(self.smoother.step(self.hold_target))
            return

        if self.state == "idle":
            if robot_q is None or gello_q is None or not fresh:
                return
            worst = max(abs(r - g) for r, g in zip(robot_q, gello_q))
            if worst > self.max_engage_off:
                if now - self._warn_t > 2.0:
                    self._warn_t = now
                    log(f"[{self.side}] 待对齐:GELLO 与臂差 {worst:.2f} rad"
                        f" > 上限 {self.max_engage_off},请手持 GELLO"
                        f" 摆近臂当前姿态")
                return
            self.smoother.reset(robot_q)
            self.state = "aligning"
            log(f"[{self.side}] ① 对齐中:臂滑向 GELLO 位形"
                f"(Δmax={worst:.2f} rad),请举稳 GELLO 别动")

        if not fresh and self.state in ("aligning", "tracking"):
            self.freeze_target = list(self._last_cmd or robot_q or [])
            self.state = "frozen"
            log(f"[{self.side}] GELLO 断流 >{self.watchdog_s}s,冻结;"
                f"恢复后将重新对齐")
        if self.state == "frozen":
            if fresh:
                self.state = "idle"
                return
            if self.freeze_target:
                self._publish(self.smoother.step(self.freeze_target))
            return

        if self.state == "aligning":
            cmd = self.smoother.step(gello_q)
            self._last_cmd = cmd
            self._publish(cmd)
            if max(abs(c - g) for c, g in zip(cmd, gello_q)) < self.align_done:
                self.ready_target = list(cmd)
                self.state = "ready"
                log(f"[{self.side}] ② 已对齐并保持。"
                    f"g+回车=开始跟随  p+回车=暂停  q+回车=退出")
            return

        if self.state == "ready":
            cmd = self.smoother.step(self.ready_target)
            self._last_cmd = cmd
            self._publish(cmd)
            return

        if self.state == "tracking":
            k = 0.0
            u = (now - self.go_t0) / self.go_blend_s
            if u < 1.0:
                k = 0.5 * (1.0 + math.cos(math.pi * u))
            target = [g + o * k for g, o in zip(gello_q, self.offset)]
            cmd = self.smoother.step(target)
            self._last_cmd = cmd
            self._publish(cmd)

    def _publish(self, cmd):
        self.msg.header.stamp = self._clock.now().to_msg()
        self.msg.name = self.gello_names or self.robot_names or self.msg.name
        self.msg.position = list(cmd)
        self.pub.publish(self.msg)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=os.path.join(_DIR, "relay.yaml"))
    ap.add_argument("--dry-run", action="store_true",
                    help="发布到 *_relay_preview 话题,不接管控制器")
    args = ap.parse_args()
    with open(args.config, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if args.dry_run:
        cfg["out_topic"] = cfg["dry_run_out_topic"]

    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node("gello_relay")
    log = lambda m: node.get_logger().info(m)

    scfg = os.path.join(os.path.dirname(_DIR), "smoother", "config.yaml")
    sc = cfg["sides"]
    if isinstance(sc, list):
        sc = {s: "teleop" for s in sc}
    sides = [_Side(node, s, m, cfg, scfg) for s, m in sc.items()]

    stop = threading.Event()

    def stdin_loop():
        for line in sys.stdin:
            c = line.strip().lower()[:1]
            if c == "g":
                for s in sides:
                    s.cmd_go(log)
            elif c == "p":
                for s in sides:
                    s.cmd_pause(log)
            elif c == "q":
                stop.set()
                return
    threading.Thread(target=stdin_loop, daemon=True).start()

    period = 1.0 / float(cfg["rate_hz"])

    def loop():
        while rclpy.ok() and not stop.is_set():
            t = time.monotonic()
            for s in sides:
                try:
                    s.tick(t, log)
                except Exception as e:
                    log(f"[{s.side}] tick 异常:{e}")
            dt = time.monotonic() - t
            time.sleep(max(0.0, period - dt))
    threading.Thread(target=loop, daemon=True).start()

    mode = "DRY-RUN(preview 话题)" if args.dry_run else "接管控制器输入"
    log(f"gello_relay 运行:{sc} @{cfg['rate_hz']}Hz  输出:{mode}")
    log("流程:对齐(自动)→ 就绪(保持)→ g+回车 跟随;p 暂停;q 退出")
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
