#!/usr/bin/env python3
from __future__ import annotations

import argparse
import io
import socket
import time
import traceback

import numpy as np

from protocol import recv_msg, send_msg

RAW_KEYS = ("observation.images.head",
            "observation.images.wrist_left",
            "observation.images.wrist_right")


def load_policy(ckpt: str, device: str, use_rtc: bool = True):
    import torch
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy
    from lerobot.policies.rtc.configuration_rtc import RTCConfig
    from lerobot.policies.rtc.modeling_rtc import RTCProcessor

    print(f"[server] 加载 ckpt: {ckpt}")
    t0 = time.time()
    policy = PI05Policy.from_pretrained(ckpt)
    policy.to(device).eval()
    if use_rtc:
        rtc_cfg = RTCConfig(enabled=True, mode="guided")
        policy.config.rtc_config = rtc_cfg
        proc = RTCProcessor(rtc_cfg, trained_mode_supported=False)
        policy.rtc_processor = proc
        model = getattr(policy, "model", None)
        if model is not None:
            model.rtc_processor = proc
            if getattr(model, "config", None) is not policy.config:
                model.config.rtc_config = rtc_cfg
        print("[server] RTC guided 已启用")
    else:
        _RTC_ON["on"] = False
        print("[server] RTC disabled")
    pre, post = make_pre_post_processors(
        policy.config, pretrained_path=ckpt,
        preprocessor_overrides={"device_processor": {"device": device}},
    )
    print(f"[server] 就绪({time.time()-t0:.1f}s, device={device})")
    return policy, pre, post


def build_batch(header: dict, blobs: list[bytes]):
    import torch
    from PIL import Image

    batch: dict = {}
    for key, blob in zip(header["keys"], blobs):
        img = np.asarray(Image.open(io.BytesIO(blob)).convert("RGB"))
        t = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        batch[key] = t
    batch["observation.state"] = torch.tensor(header["state"], dtype=torch.float32)
    batch["task"] = header["task"]
    return batch


_LAST = {"chunk": None}
_RTC_ON = {"on": True}


def infer(policy, pre, post, batch, rtc: dict | None = None) -> np.ndarray:
    import torch
    from lerobot.policies.pi05.modeling_pi05 import pad_vector
    with torch.no_grad():
        proc = pre(batch)
        kwargs = {}
        if _RTC_ON["on"] and rtc and rtc.get("cur_index") is not None \
                and _LAST["chunk"] is not None:
            cur = max(0, int(rtc["cur_index"]))
            left = _LAST["chunk"][:, cur:, :]
            if left.shape[1] > 0:
                madim = int(getattr(policy.config, "max_action_dim", 32))
                kwargs = {
                    "prev_chunk_left_over": pad_vector(left, madim),
                    "inference_delay": int(rtc.get("delay_est", 12)),
                    "execution_horizon": int(rtc.get("exec_h", 25)),
                }
        chunk = policy.predict_action_chunk(proc, **kwargs)
        _LAST["chunk"] = chunk.detach()
        out = chunk
        try:
            out = post(out)
        except Exception:
            out = post({"action": out})["action"]
    return out.squeeze(0).float().cpu().numpy()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--port", type=int, default=6060)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--no-rtc", action="store_true",
                    help="disable RTC")
    args = ap.parse_args()

    policy, pre, post = load_policy(args.ckpt, args.device, use_rtc=not args.no_rtc)

    import torch
    warm = {k: torch.zeros(3, 286, 570) for k in RAW_KEYS}
    warm["observation.state"] = torch.zeros(8)
    warm["task"] = "warmup"
    t0 = time.time()
    infer(policy, pre, post, warm)
    print(f"[server] 预热推理 {time.time()-t0:.2f}s")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    print(f"[server] 监听 :{args.port}")
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        _LAST["chunk"] = None
        print(f"[server] 客户端接入 {addr}(RTC缓存已重置)")
        try:
            while True:
                header, blobs = recv_msg(conn)
                t0 = time.time()
                chunk = infer(policy, pre, post, build_batch(header, blobs),
                              header.get("rtc"))
                ms = (time.time() - t0) * 1e3
                send_msg(conn, {"actions": chunk.tolist(), "t_infer_ms": ms})
        except (ConnectionError, OSError) as e:
            print(f"[server] 客户端断开: {e}")
        except Exception:
            traceback.print_exc()
            try:
                send_msg(conn, {"error": traceback.format_exc()[-500:]})
            except Exception:
                pass
        finally:
            conn.close()


if __name__ == "__main__":
    main()
