#!/usr/bin/env python3
from __future__ import annotations

import argparse
import io
import socket
import time

import numpy as np

from protocol import recv_msg, send_msg


def fake_jpeg(h, w):
    from PIL import Image
    rng = np.random.default_rng(0)
    img = rng.integers(0, 255, (h, w, 3), dtype=np.uint8)
    buf = io.BytesIO()
    Image.fromarray(img).save(buf, format="JPEG", quality=90)
    return buf.getvalue()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=6060)
    ap.add_argument("--n", type=int, default=5)
    args = ap.parse_args()

    blobs = [fake_jpeg(286, 570), fake_jpeg(480, 640), fake_jpeg(480, 640)]
    keys = ["observation.images.head", "observation.images.wrist_left",
            "observation.images.wrist_right"]
    state = [0.41, -0.69, -0.24, -2.59, 0.09, 2.16, 1.31, 1.0]

    sock = socket.create_connection((args.host, args.port), timeout=120)
    lat = []
    for i in range(args.n):
        t0 = time.time()
        rtc = {"cur_index": 20, "delay_est": 12, "exec_h": 32} if i > 0 else None
        send_msg(sock, {"state": state, "task": "smoke test", "keys": keys,
                        "rtc": rtc,
                        "jpeg_lens": [len(b) for b in blobs]}, blobs)
        header, _ = recv_msg(sock)
        dt = (time.time() - t0) * 1e3
        assert "error" not in header, header.get("error")
        a = np.asarray(header["actions"])
        lat.append(dt)
        print(f"#{i}: 端到端 {dt:.0f}ms(纯推理 {header['t_infer_ms']:.0f}ms) "
              f"chunk {a.shape} 范围[{a.min():.3f},{a.max():.3f}]")
    print(f"均值端到端 {np.mean(lat[1:]):.0f}ms")
    print("SMOKE-OK")


if __name__ == "__main__":
    main()
