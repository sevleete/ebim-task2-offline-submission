from __future__ import annotations

import math
import pathlib

import numpy as np
import yaml

PKG_ROOT = pathlib.Path(__file__).resolve().parent.parent


def load_config(path: str | None = None) -> dict:
    p = pathlib.Path(path) if path else PKG_ROOT / "config" / "ebim_nav.yaml"
    with open(p) as f:
        return yaml.safe_load(f)


def scan_to_points(msg, range_clip_min: float = 0.12) -> np.ndarray:
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
    valid = (ranges > max(msg.range_min, range_clip_min)) & (ranges < msg.range_max) & np.isfinite(ranges)
    r, a = ranges[valid], angles[valid]
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def points_to_virtual_scan(pts: np.ndarray, n_bins: int, range_max: float):
    ranges = np.full(n_bins, float("inf"))
    if len(pts):
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        r = np.hypot(pts[:, 0], pts[:, 1])
        inc = 2.0 * math.pi / n_bins
        bins = ((ang + math.pi) / inc).astype(int) % n_bins
        np.minimum.at(ranges, bins, r)
    ranges[ranges > range_max] = float("inf")
    return ranges, -math.pi, 2.0 * math.pi / n_bins


EXTR_FILE = PKG_ROOT / "config" / "extrinsics.yaml"

URDF_DEFAULTS = {
    "front": {"x": 0.3275, "y": 0.2175, "yaw_deg": 135.0, "mirror": True},
    "rear": {"x": -0.3275, "y": -0.2175, "yaw_deg": 315.0, "mirror": True},
}


def load_extrinsics() -> dict:
    data = {}
    if EXTR_FILE.exists():
        with open(EXTR_FILE) as f:
            data = yaml.safe_load(f) or {}
    return {k: {**URDF_DEFAULTS[k], **data.get(k, {})} for k in URDF_DEFAULTS}


def save_extrinsics(extr: dict):
    with open(EXTR_FILE, "w") as f:
        f.write("# 双雷达平面外参（extrinsic_tuner 标定产物，scan_merger 优先采用）\n")
        f.write("# mirror=true 表示倒装镜像（先翻转 x 再旋转平移）\n")
        yaml.safe_dump(extr, f, sort_keys=False)


def planar_apply(pts: np.ndarray, p: dict) -> np.ndarray:
    yaw = math.radians(p["yaw_deg"])
    c, s = math.cos(yaw), math.sin(yaw)
    M = np.array([[c, -s], [s, c]]) @ (np.diag([-1.0, 1.0]) if p["mirror"] else np.eye(2))
    return pts @ M.T + np.array([p["x"], p["y"]])


def tf_to_se2(tf_msg) -> np.ndarray:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, t.x], [s, c, t.y], [0.0, 0.0, 1.0]])
