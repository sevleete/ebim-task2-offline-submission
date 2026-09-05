#!/usr/bin/env python3
from __future__ import annotations

import pickle
import queue
import socket
import struct
import threading
import time

import numpy as np


def _send_pkl(s, o):
    p = pickle.dumps(o, protocol=pickle.HIGHEST_PROTOCOL)
    s.sendall(struct.pack(">Q", len(p)) + p)


def _recv_exact(s, n):
    b = b""
    while len(b) < n:
        d = s.recv(n - len(b))
        if not d:
            return None
        b += d
    return b


def _recv_pkl(s):
    h = _recv_exact(s, 8)
    if h is None:
        return None
    (ln,) = struct.unpack(">Q", h)
    return pickle.loads(_recv_exact(s, ln))


def pack_wrist_jpg(obs, img_size, jpeg_q):
    import cv2
    with obs.lock:
        img = obs.imgs.get("observation.images.wrist_left")
        if img is None:
            return None
        img = img.copy()
    img = cv2.resize(img, (img_size, img_size), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img[:, :, ::-1],
                           [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])
    return buf.tobytes() if ok else None


def pack_state8(obs):
    with obs.lock:
        if obs.q is None:
            return None
        return [float(x) for x in list(obs.q) + [obs.grip]]


class HeadLink(threading.Thread):

    def __init__(self, cfg_head, obs, log):
        super().__init__(daemon=True)
        self.host = cfg_head.get("host", "127.0.0.1")
        self.port = int(cfg_head.get("port", 5561))
        self.obs, self.log = obs, log
        self.img_size = int(cfg_head.get("img_size", 144))
        self.jpeg_q = int(cfg_head.get("jpeg_q", 85))
        self.max_age = float(cfg_head.get("delta_max_age", 0.5))
        self.sq: queue.Queue = queue.Queue(maxsize=1)
        self.delta = np.zeros(8, np.float32)
        self.delta_time = 0.0
        self.ver = -1
        self.gate = None
        self.sock = None
        self.stop = threading.Event()

    def submit(self, a_base):
        jpg = pack_wrist_jpg(self.obs, self.img_size, self.jpeg_q)
        st = pack_state8(self.obs)
        if jpg is None or st is None:
            return
        msg = {"kind": "sample", "mid": "deploy", "sim_time": time.time(),
               "mode": "auto", "imgs": {"wrist_left": jpg},
               "state": st,
               "a_base": [float(x) for x in np.asarray(a_base).ravel()],
               "a_applied": None, "want_delta": True}
        try:
            self.sq.put_nowait(msg)
        except queue.Full:
            try:
                self.sq.get_nowait()
            except queue.Empty:
                pass
            try:
                self.sq.put_nowait(msg)
            except queue.Full:
                pass

    def latest_delta(self):
        if time.time() - self.delta_time > self.max_age:
            return np.zeros(8, np.float32)
        return self.delta

    def _connect(self):
        while self.sock is None and not self.stop.is_set():
            try:
                s = socket.create_connection((self.host, self.port), timeout=5)
                s.settimeout(3.0)
                self.sock = s
                self.log(f"[head] 已连冻结头 {self.host}:{self.port}")
            except OSError:
                time.sleep(2.0)

    def _drop(self):
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None
        self.delta_time = 0.0

    def run(self):
        self._connect()
        while not self.stop.is_set():
            try:
                msg = self.sq.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if self.sock is None:
                    raise ConnectionResetError
                _send_pkl(self.sock, msg)
                r = _recv_pkl(self.sock)
                if r is None:
                    raise ConnectionResetError
                d = r.get("delta") if isinstance(r, dict) else None
                if d is not None:
                    self.delta = np.asarray(d, np.float32)
                    self.delta_time = time.time()
                self.ver = int(r.get("ver", -1)) if isinstance(r, dict) else -1
                self.gate = r.get("gate") if isinstance(r, dict) else None
            except (OSError, ConnectionResetError, struct.error, EOFError):
                self.log("[head] reconnecting")
                self._drop()
                self._connect()
