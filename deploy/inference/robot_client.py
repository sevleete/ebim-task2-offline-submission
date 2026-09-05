#!/usr/bin/env python3
from __future__ import annotations

import argparse
import io
import math
import os
import socket
import sys
import threading
import time

import numpy as np
import yaml

_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_DIR))
sys.path.insert(0, _ROOT)
sys.path.insert(0, _DIR)
from smoother import config as sconfig
from protocol import recv_msg, send_msg
from head_link import HeadLink


def load_cfg(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


class ObsBank:

    def __init__(self, cfg, node):
        import cv2
        from cv_bridge import CvBridge
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, JointState

        self.cv2 = cv2
        self.cfg = cfg
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.imgs: dict[str, np.ndarray] = {}
        self.q: list[float] | None = None
        self.q_t: float = 0.0
        self.grip: float = 0.0

        o = cfg["obs"]
        topics = {"observation.images.head": o["head_topic"],
                  "observation.images.wrist_left": o["wrist_left_topic"],
                  "observation.images.wrist_right": o["wrist_right_topic"]}
        for key, t in topics.items():
            node.create_subscription(
                CompressedImage, t,
                lambda m, k=key: self._on_img(k, m), qos_profile_sensor_data)
        node.create_subscription(JointState, o["measured_topic"],
                                 self._on_q, qos_profile_sensor_data)
        node.create_subscription(JointState, o["gripper_topic"],
                                 self._on_grip, qos_profile_sensor_data)
        self.joint_names = cfg["act"]["joint_names"]

    def _on_img(self, key, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "rgb8")
        if key.endswith("head"):
            c = self.cfg["obs"]["head_crop"]
            img = img[c["y"]:c["y"] + c["h"], c["x"]:c["x"] + c["w"]]
        else:
            d = int(self.cfg["obs"].get("wrist_downscale", 1))
            if d > 1:
                img = self.cv2.resize(img, (img.shape[1] // d, img.shape[0] // d),
                                      interpolation=self.cv2.INTER_AREA)
        with self.lock:
            self.imgs[key] = img

    def _on_q(self, msg):
        mp = dict(zip(msg.name, msg.position))
        try:
            q = [mp[n] for n in self.joint_names]
        except KeyError:
            return
        with self.lock:
            self.q = q
            self.q_t = time.monotonic()

    def _on_grip(self, msg):
        o = self.cfg["obs"]
        mp = dict(zip(msg.name, msg.position))
        rad = mp.get(o["gripper_finger"])
        if rad is None or not math.isfinite(rad):
            return
        with self.lock:
            self.grip = max(0.0, min(1.0, 1.0 - rad / o["gripper_closed_rad"]))

    def ready(self):
        with self.lock:
            return len(self.imgs) == 3 and self.q is not None

    def missing(self):
        with self.lock:
            m = [k for k in ("observation.images.head", "observation.images.wrist_left",
                             "observation.images.wrist_right") if k not in self.imgs]
            if self.q is None:
                m.append("measured_joint_states")
            return m

    def snapshot(self):
        with self.lock:
            imgs = {k: v.copy() for k, v in self.imgs.items()}
            state = list(self.q) + [self.grip]
        keys, blobs = [], []
        for k, img in imgs.items():
            ok, enc = self.cv2.imencode(
                ".jpg", img[:, :, ::-1],
                [self.cv2.IMWRITE_JPEG_QUALITY, self.cfg["obs"]["jpeg_quality"]])
            assert ok
            keys.append(k)
            blobs.append(enc.tobytes())
        return state, keys, blobs


class InferLink:

    def __init__(self, cfg):
        self.host, self.port = cfg["server"]["host"], cfg["server"]["port"]
        self.task = cfg["task"]
        self.sock = None
        self.lock = threading.Lock()

    def connect(self):
        last = None
        for _ in range(4):
            try:
                self.sock = socket.create_connection(
                    (self.host, self.port), timeout=15)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                return
            except OSError as e:
                last = e
                time.sleep(1.5)
        raise ConnectionError(
            f"连不上 {self.host}:{self.port}({last});"
            f"检查 SSH 转发与服务器 infer_server 是否在跑")

    def infer(self, state, keys, blobs, rtc: dict | None = None) -> tuple[np.ndarray, float]:
        with self.lock:
            if self.sock is None:
                self.connect()
            try:
                send_msg(self.sock, {"state": state, "task": self.task,
                                     "keys": keys, "rtc": rtc,
                                     "jpeg_lens": [len(b) for b in blobs]}, blobs)
                self.sock.settimeout(20)
                header, _ = recv_msg(self.sock)
            except (ConnectionError, OSError):
                self.sock = None
                raise
        if "error" in header:
            raise RuntimeError("服务器端异常: " + header["error"][:200])
        return np.asarray(header["actions"], dtype=np.float32), header["t_infer_ms"]


class Executor:

    def __init__(self, cfg, node, obs: ObsBank, link: InferLink, log, use_head=True):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32

        self.cfg, self.obs, self.link, self.log = cfg, obs, link, log
        a = cfg["act"]
        self.dt_chunk = 1.0 / a["chunk_fps"]
        fm = cfg.get("fine_mode", {}) or {}
        self.exec_h_normal = int(a["exec_horizon"])
        self.exec_h_fine = int(fm.get("exec_horizon", self.exec_h_normal))
        self.fine_threshold = float(fm.get("auto_threshold_rad", 0.05))
        self.exec_h = self.exec_h_normal
        self.fine_mode = False
        self.auto_fine = True
        self.prefetch = int(a["prefetch_margin"])
        self.lag_extra = int(a.get("lag_extra_steps", 1))
        self.cycle = 1.0 / a["stream_hz"]
        self.qmin = np.asarray(cfg["safety"]["q_min"])
        self.qmax = np.asarray(cfg["safety"]["q_max"])
        self.max_step = float(cfg["safety"]["max_step_rad"])
        self.max_track_lag = float(cfg["safety"].get("max_track_lag_rad", 0.0))
        self.watchdog = float(cfg["safety"]["watchdog_s"])
        self.dbg_chunk = bool(cfg.get("debug", {}).get("chunk_dir", False))

        fin = cfg.get("finish", {}) or {}
        self.finish = None
        if fin.get("enabled"):
            self.finish = {
                "closed_below": float(fin.get("closed_below", 0.25)),
                "open_above": float(fin.get("open_above", 0.75)),
                "min_hold_s": float(fin.get("min_hold_s", 2.0)),
                "min_carry_rad": float(fin.get("min_carry_rad", 0.6)),
                "reopen_s": float(fin.get("reopen_s", 0.8)),
                "settle_s": float(fin.get("settle_s", 3.0)),
            }
        self._ep_phase = "wait"
        self._ep_t = 0.0
        self._ep_q0 = None

        scfg = sconfig.load(os.path.join(_ROOT, a["smoother_config"]))
        scfg.cycle_time = self.cycle
        try:
            self.smoother = sconfig.build(scfg)
        except Exception as e:
            from smoother.clamp_smoother import ClampSmoother
            log(f"ruckig 不可用({e}),退化 clamp")
            self.smoother = ClampSmoother(scfg.limits, self.cycle)

        self.pub = node.create_publisher(JointState, a["target_topic"], 10)
        self.pub_grip = node.create_publisher(Float32, a["gripper_cmd_topic"], 10)
        self.names = a["joint_names"]
        self.clock = node.get_clock()
        self.JointState, self.Float32 = JointState, Float32

        self.mode = "hold"
        self.hold_q: np.ndarray | None = None
        self.wps: list[np.ndarray] = []
        self.wp_meta: list[tuple] = []
        self.chunk_seq = 0
        self.cur_pos: tuple | None = None
        self.last_lag = 12
        self.wp_t0 = 0.0
        self.last_grip_cmd = None
        self.infer_flight = False
        self.last_chunk_t = 0.0
        self._leashed = False
        self._stalled = False
        self.stop = threading.Event()

        hc = cfg.get("head", {}) or {}
        self.head = None
        self._head_dema = np.zeros(8, np.float32)
        if use_head and hc.get("enabled"):
            self.head_correction = bool(hc.get("correction", True))
            self.head_bounds = np.asarray(hc.get(
                "bounds", [0.05] * 7 + [0.15]), np.float32)
            self.head_ema = float(hc.get("delta_ema", 0.6))
            self.head_no_grip = bool(hc.get("no_grip_delta", False))
            self.head = HeadLink(hc, obs, log)
            self.head.start()
            log(f"[head] enabled {hc.get('host')}:{hc.get('port')} "
                f"修正={'开' if self.head_correction else '关'} "
                f"bounds={self.head_bounds.tolist()}")

    def _request_chunk(self):
        if self.infer_flight:
            return
        self.infer_flight = True

        def work():
            try:
                planned_lag = self.last_lag
                exec_h_used = self.exec_h
                rtc = {"delay_est": planned_lag, "exec_h": exec_h_used,
                       "cur_index": None}
                if self.cur_pos is not None and self.cur_pos[0] == self.chunk_seq - 1:
                    rtc["cur_index"] = self.cur_pos[1] + 1
                state, keys, blobs = self.obs.snapshot()
                t0 = time.monotonic()
                chunk, ms = self.link.infer(state, keys, blobs, rtc)
                rtt = time.monotonic() - t0
                if self.dbg_chunk and len(chunk):
                    self._log_chunk_dir(chunk, np.asarray(state[:7]))
                self._auto_fine(chunk, np.asarray(state[:7]))
                needed = math.ceil(rtt / self.dt_chunk) + self.lag_extra
                lag_steps = planned_lag if needed <= planned_lag else needed
                if lag_steps != planned_lag:
                    self.log(f"⚠ 网络延迟波动({rtt*1e3:.0f}ms)")
                self.last_lag = max(needed, 2)
                take = chunk[lag_steps:lag_steps + exec_h_used]
                if len(take) < exec_h_used:
                    self.log(f"⚠ 延迟过大: 往返{rtt*1e3:.0f}ms 仅剩{len(take)}步"
                             f"(chunk共{len(chunk)})—— 检查网络")
                if not len(take):
                    return
                self._append_wps(take, lag_steps)
                self.last_chunk_t = time.monotonic()
                self.log(f"chunk#{self.chunk_seq-1}: 推理{ms:.0f}ms "
                         f"往返{rtt*1e3:.0f}ms 接入{len(take)} "
                         f"队列{len(self.wps)}")
            except Exception as e:
                self.log(f"✗ 推理失败: {e}")
            finally:
                self.infer_flight = False
        threading.Thread(target=work, daemon=True).start()

    def _append_wps(self, take: np.ndarray, lag_steps: int):
        seq = self.chunk_seq
        self.chunk_seq += 1
        prev = self.wps[-1][:7] if self.wps else (
            self.hold_q if self.hold_q is not None else None)
        out = []
        metas = []
        for j, a8 in enumerate(take):
            q = np.clip(a8[:7], self.qmin, self.qmax)
            if prev is not None:
                step = q - prev
                big = np.abs(step) > self.max_step
                if big.any():
                    self.log(f"⚠ 航点跳变钳制 {np.abs(step).max():.3f} rad")
                    q = prev + np.clip(step, -self.max_step, self.max_step)
            g = float(np.clip(a8[7], 0.0, 1.0))
            out.append(np.concatenate([q, [g]]))
            metas.append((seq, lag_steps + j))
            prev = q
        self.wps.extend(out)
        self.wp_meta.extend(metas)

    def loop(self):
        last = time.monotonic()
        while not self.stop.is_set():
            t = time.monotonic()
            gap = t - last
            if gap > 0.15:
                self.log(f"⚠ 发布循环间隙 {gap*1e3:.0f}ms(看门狗上限500ms)")
            last = t
            self._tick(t)
            time.sleep(max(0.0, self.cycle - (time.monotonic() - t)))

    def _tick(self, now):
        if self.mode == "hold":
            if self.hold_q is None:
                if self.obs.q is not None:
                    self.hold_q = np.asarray(self.obs.q)
                    self.smoother.reset(list(self.hold_q))
                    self.log("HOLD: 锁定当前位形 "
                             + " ".join(f"{v:.3f}" for v in self.hold_q))
                else:
                    return
            self._publish(self.smoother.step(list(self.hold_q)))
            return

        if self.mode == "frozen":
            if self.hold_q is not None:
                self._publish(self.smoother.step(list(self.hold_q)))
            return

        if self.obs.q_t and now - self.obs.q_t > 1.5:
            self.log("✗✗ 臂侧 measured 断流 >1.5s —— 控制器/硬件已死!"
                     "自动冻结;请 Ctrl-C 后跑 tmrctl --enable left_arm 再重来")
            self.mode = "frozen"
            return
        if len(self.wps) <= self.prefetch:
            self._request_chunk()
        if not self.wps:
            # 无航点(推理延迟尖峰/网络抖动):保持当前位形喂满流,持续请求下一段,
            # chunk 一到自动恢复——不闩锁、不停下等按键(偶发超时不该打断 eval)。
            if now - self.last_chunk_t > self.watchdog and not self._stalled:
                self._stalled = True
                self.log("⏳ 等待推理结果(延迟尖峰),保持位形,到达即自动继续")
            if self.hold_q is not None:
                self._publish(self.smoother.step(list(self.hold_q)))
            return
        if self._stalled:
            self._stalled = False
            self.wp_t0 = now
            self.log("▶ 推理已恢复,自动继续执行")
        if self._tracking_lagged():
            if self.hold_q is not None:
                self._publish(self.smoother.step(list(self.hold_q)))
            self.wp_t0 = now
            return
        if now - self.wp_t0 >= self.dt_chunk:
            raw = self.wps.pop(0)
            if self.wp_meta:
                self.cur_pos = self.wp_meta.pop(0)
            self.wp_t0 = now
            if self.head is not None:
                self.head.submit(raw)
                self.cur = self._apply_head(raw)
            else:
                self.cur = raw
            self.hold_q = self.cur[:7]
            self._grip(self.cur[7])
            self._episode_progress(now, float(self.cur[7]))
        u = min(1.0, (now - self.wp_t0) / self.dt_chunk)
        nxt = ((self.wps[0][:7] + self._head_dema[:7]) if self.wps
               else self.cur[:7])
        nxt = np.clip(nxt, self.qmin, self.qmax)
        target = (1 - u) * self.cur[:7] + u * nxt
        self._publish(self.smoother.step(list(target)))

    def _apply_head(self, raw8):
        if self.head is None or not self.head_correction:
            self._head_dema = np.zeros(8, np.float32)
            return raw8
        d = np.clip(self.head.latest_delta(), -self.head_bounds, self.head_bounds)
        self._head_dema = self.head_ema * self._head_dema + (1.0 - self.head_ema) * d
        d = np.clip(self._head_dema, -self.head_bounds, self.head_bounds)
        if self.head_no_grip:
            d[7] = 0.0
        out = raw8.copy()
        out[:7] = np.clip(out[:7] + d[:7], self.qmin, self.qmax)
        out[7] = float(np.clip(out[7] + d[7], 0.0, 1.0))
        return out

    def _log_chunk_dir(self, chunk: np.ndarray, q_meas: np.ndarray):
        d_end = chunk[-1][:7] - q_meas
        d_first = chunk[0][:7] - q_meas
        k = int(np.argmax(np.abs(d_end)))
        flip = ""
        if d_first[k] * d_end[k] < 0 and abs(d_first[k]) > 0.01:
            flip = f" flip@j{k}({d_first[k]:+.02f}->{d_end[k]:+.02f})"
        vec = "[" + " ".join(f"{v:+.02f}" for v in d_end) + "]"
        self.log(f"chunk#{self.chunk_seq} d_end{vec} "
                 f"|max|={abs(d_end[k]):.02f}@j{k}{flip}")

    def _tracking_lagged(self):
        if self.max_track_lag <= 0 or self.hold_q is None:
            return False
        with self.obs.lock:
            if self.obs.q is None:
                return False
            m = np.asarray(self.obs.q)
        lagged = bool(np.max(np.abs(self.hold_q[:7] - m)) > self.max_track_lag)
        if lagged != self._leashed:
            self._leashed = lagged
            if lagged:
                gap = float(np.max(np.abs(self.hold_q[:7] - m)))
                self.log(f"⏳ 等待跟踪同步({gap:.3f}rad)")
            else:
                self.log("▶ 恢复推进")
        return lagged

    def _episode_progress(self, now, grip):
        fc = self.finish
        if not fc or self.stop.is_set():
            return
        ph = self._ep_phase
        if grip < fc["closed_below"]:
            if ph in ("wait", "settling"):
                self._ep_phase = "closing"
                self._ep_t = now
                with self.obs.lock:
                    self._ep_q0 = np.asarray(self.obs.q) if self.obs.q else None
            elif ph == "closing" and now - self._ep_t >= fc["min_hold_s"]:
                self._ep_phase = "holding"
            elif ph == "opening":
                self._ep_phase = "holding"
        elif grip > fc["open_above"]:
            if ph == "closing":
                self._ep_phase = "wait"
            elif ph == "holding":
                with self.obs.lock:
                    q1 = np.asarray(self.obs.q) if self.obs.q else None
                moved = (self._ep_q0 is not None and q1 is not None and
                         float(np.linalg.norm(q1 - self._ep_q0)) >= fc["min_carry_rad"])
                self._ep_phase = "opening" if moved else "wait"
                self._ep_t = now
            elif ph == "opening" and now - self._ep_t >= fc["reopen_s"]:
                self._ep_phase = "settling"
                self._ep_t = now
                self.log(f"⏱ 检测到放置完成,{fc['settle_s']:.0f}s 后自动结束")
            elif ph == "settling" and now - self._ep_t >= fc["settle_s"]:
                self.log("✓ 本回合结束(放置完成)")
                self.stop.set()

    def _grip(self, g):
        if self.last_grip_cmd is None or abs(g - self.last_grip_cmd) > 0.02:
            m = self.Float32()
            m.data = float(g)
            self.pub_grip.publish(m)
            self.last_grip_cmd = g

    def _publish(self, q):
        m = self.JointState()
        m.header.stamp = self.clock.now().to_msg()
        m.name = self.names
        m.position = [float(v) for v in q[:7]]
        self.pub.publish(m)

    def start_run(self):
        if not self.obs.ready():
            self.log(f"✗ 观测不全: {self.obs.missing()}")
            return
        self.cur = np.concatenate([self.hold_q, [self.obs.grip]])
        self.wp_t0 = time.monotonic()
        self.last_chunk_t = time.monotonic()
        self._ep_phase = "wait"
        self._ep_q0 = None
        self._stalled = False
        self.mode = "run"
        self.log("▶ 策略执行开始")

    def pause(self):
        self.wps.clear()
        self.wp_meta.clear()
        self.cur_pos = None
        self.mode = "hold"
        self.hold_q = np.asarray(self.obs.q) if self.obs.q else self.hold_q
        self._ep_phase = "wait"
        self._ep_q0 = None
        self.log("⏸ 暂停,保持当前位形(回车继续)")

    def toggle_fine(self):
        self.auto_fine = not self.auto_fine
        self.log(f"mode={'auto' if self.auto_fine else 'locked'}"
                 f"(exec_horizon={self.exec_h})")

    def _auto_fine(self, chunk: np.ndarray, q_meas: np.ndarray):
        if not self.auto_fine or self.exec_h_fine == self.exec_h_normal:
            return
        mag = float(np.max(np.abs(chunk[-1][:7] - q_meas)))
        fine = mag < self.fine_threshold
        if fine != self.fine_mode:
            self.fine_mode = fine
            self.exec_h = self.exec_h_fine if fine else self.exec_h_normal
            self.log(f"exec_horizon → {self.exec_h}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=os.path.join(_DIR, "config.yaml"))
    ap.add_argument("--rearm-cmd", default=None,
                    help="HOLD 建立后自动执行的接管命令(pipeline 用),"
                         "如: '~/.pixi/bin/pixi run tmrctl --rearm left'")
    args = ap.parse_args()
    cfg = load_cfg(args.config)

    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node("pi05_deploy_client")
    log = lambda m: print(m, flush=True)

    obs = ObsBank(cfg, node)
    link = InferLink(cfg)
    ex = Executor(cfg, node, obs, link, log)

    log("等待观测就绪…")
    t0 = time.time()
    while not obs.ready():
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() - t0 > 10:
            log(f"仍缺: {obs.missing()}")
            t0 = time.time()
    log("观测就绪 ✓;测试服务器连接…")
    s, k, b = obs.snapshot()
    chunk, ms = link.infer(s, k, b)
    log(f"服务器 ✓ 推理{ms:.0f}ms chunk形状{chunk.shape}")

    threading.Thread(target=ex.loop, daemon=True).start()

    if args.rearm_cmd:
        def auto_rearm():
            while ex.hold_q is None and not ex.stop.is_set():
                time.sleep(0.1)
            time.sleep(1.0)
            import subprocess
            enable_cmd = args.rearm_cmd.replace("--rearm left", "--enable left_arm")
            for attempt in (1, 2, 3):
                log(f"[auto] 执行接管(第{attempt}次): {args.rearm_cmd}")
                r = subprocess.run(args.rearm_cmd, shell=True,
                                   capture_output=True, text=True, cwd=_ROOT)
                for ln in (r.stdout + r.stderr).strip().splitlines()[-3:]:
                    log(f"[auto] {ln}")
                time.sleep(1.5)
                if r.returncode == 0 and ex.obs.q_t and \
                        time.monotonic() - ex.obs.q_t < 1.0:
                    log("[auto] 接管完成,2s 后自动开始执行"
                        "(p=暂停 q=退出 Ctrl-C=急停)")
                    time.sleep(2.0)
                    for _ in range(20):
                        if ex.stop.is_set() or ex.mode != "hold":
                            return
                        if ex.obs.ready():
                            ex.start_run()
                            return
                        time.sleep(0.5)
                    log("[auto] 观测未就绪,未自动开始;就绪后按 回车 手动开始")
                    return
                log("[auto] 接管未成/臂侧被反射打死,先恢复臂再重试…")
                subprocess.run(enable_cmd, shell=True,
                               capture_output=True, text=True, cwd=_ROOT)
                time.sleep(1.0)
            log("[auto] ✗ 三次接管都失败,请手动排查(tmr_arms.log)")
        threading.Thread(target=auto_rearm, daemon=True).start()

    def stdin_loop():
        log(">>> HOLD 中(已向控制器发布当前位形)。"
            "pipeline 模式下 rearm 后自动开始;手动模式按 回车 开始;p=暂停 q=退出")
        for line in sys.stdin:
            c = line.strip().lower()[:1]
            if c == "" and ex.mode in ("hold", "frozen"):
                ex.start_run()
            elif c == "f":
                ex.toggle_fine()
            elif c == "p":
                ex.pause()
            elif c == "q":
                ex.stop.set()
                return
    threading.Thread(target=stdin_loop, daemon=True).start()

    try:
        while rclpy.ok() and not ex.stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    log("退出:建议 tmrctl --rearm left --off 释放控制器")
    return 0


if __name__ == "__main__":
    sys.exit(main())
