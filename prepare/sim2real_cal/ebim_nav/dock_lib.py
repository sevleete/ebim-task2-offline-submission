from __future__ import annotations

import math
import time
from dataclasses import dataclass
from itertools import combinations

import numpy as np
from scipy.spatial import cKDTree

from ebim_nav.common import PKG_ROOT
from ebim_nav.icp2d import icp_2d, se2_matrix, se2_params, transform_points


def crop_radius(pts: np.ndarray, r: float) -> np.ndarray:
    return pts[np.hypot(pts[:, 0], pts[:, 1]) < r]


def crop_box(pts: np.ndarray, xmin, xmax, ymin, ymax) -> np.ndarray:
    m = (pts[:, 0] > xmin) & (pts[:, 0] < xmax) & (pts[:, 1] > ymin) & (pts[:, 1] < ymax)
    return pts[m]


def cloud_to_virtual_scan(pts3: np.ndarray, band: tuple[float, float]) -> np.ndarray:
    z = pts3[:, 2]
    sel = pts3[(z > band[0]) & (z < band[1])]
    return sel[:, :2]


@dataclass
class DockEstimate:
    ex: float
    ey: float
    eyaw: float
    fitness: float
    rmse: float
    ok: bool

    @property
    def err_xy(self) -> float:
        return math.hypot(self.ex, self.ey)


def estimate_icp(cur_pts: np.ndarray, ref_pts: np.ndarray,
                 max_corr: float, iters: int) -> DockEstimate:
    if len(cur_pts) < 15 or len(ref_pts) < 15:
        return DockEstimate(0, 0, 0, 0.0, 9.9, False)
    res = icp_2d(cur_pts, ref_pts, max_iters=iters, max_corr_dist=max_corr)
    ex, ey, eyaw = se2_params(res.T)
    ok = res.fitness > 0.5
    return DockEstimate(ex, ey, eyaw, res.fitness, res.rmse, ok)


def fit_line_ransac(pts: np.ndarray, thresh: float = 0.03, iters: int = 200):
    best_in = None
    best_n = 0
    n = len(pts)
    if n < 20:
        return None
    rng = np.random.default_rng(0)
    for _ in range(iters):
        i, j = rng.integers(0, n, 2)
        p, q = pts[i], pts[j]
        d = q - p
        L = math.hypot(*d)
        if L < 0.1:
            continue
        nx, ny = -d[1] / L, d[0] / L
        dist = np.abs(nx * (pts[:, 0] - p[0]) + ny * (pts[:, 1] - p[1]))
        inl = dist < thresh
        if inl.sum() > best_n:
            best_n = int(inl.sum())
            best_in = inl
    if best_in is None or best_n < 20:
        return None
    P = pts[best_in]
    c = P.mean(axis=0)
    u, s, vt = np.linalg.svd(P - c)
    direction = vt[0]
    theta = math.atan2(-direction[0], direction[1])
    d0 = math.cos(theta) * c[0] + math.sin(theta) * c[1]
    return theta, d0, best_in


class ServoController:

    def __init__(self, cfg_dock: dict):
        d = cfg_dock
        self.kp_xy = d["kp_xy"]
        self.kp_yaw = d["kp_yaw"]
        self.max_xy = d["max_vel_xy"]
        self.max_yaw = d["max_vel_yaw"]
        self.db_xy = d["deadband_xy"]
        self.db_yaw = math.radians(d["deadband_yaw_deg"])
        self.tol_xy = d["tol_xy"]
        self.tol_yaw = math.radians(d["tol_yaw_deg"])
        self.settle_need = d["settle_frames"]
        self._settle = 0

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _deadband(self, v, db):
        return 0.0 if abs(v) < db else v

    def step(self, est: DockEstimate):
        in_tol = est.err_xy < self.tol_xy and abs(est.eyaw) < self.tol_yaw
        self._settle = self._settle + 1 if in_tol else 0
        converged = self._settle >= self.settle_need
        if converged or not est.ok:
            return 0.0, 0.0, 0.0, converged
        vx = self._clamp(self.kp_xy * self._deadband(est.ex, self.db_xy), -self.max_xy, self.max_xy)
        vy = self._clamp(self.kp_xy * self._deadband(est.ey, self.db_xy), -self.max_xy, self.max_xy)
        wz = self._clamp(self.kp_yaw * self._deadband(est.eyaw, self.db_yaw), -self.max_yaw, self.max_yaw)
        return vx, vy, wz, False


def cluster_points(pts: np.ndarray, eps: float) -> np.ndarray:
    n = len(pts)
    parent = np.arange(n)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i, j in cKDTree(pts).query_pairs(eps, output_type="ndarray"):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj
    return np.array([find(i) for i in range(n)])


def _order_corners(p4: np.ndarray) -> np.ndarray:
    c = p4.mean(axis=0)
    ang = np.arctan2(p4[:, 1] - c[1], p4[:, 0] - c[0])
    return p4[np.argsort(ang)]


def front_leg_pair(pts: np.ndarray, leg_cfg: dict):
    if len(pts) < leg_cfg["min_pts"]:
        return None
    labels = cluster_points(pts, leg_cfg["cluster_eps"])
    comp = []
    for lb in np.unique(labels):
        c = pts[labels == lb]
        if len(c) < leg_cfg["min_pts"]:
            continue
        ext = float(max(c[:, 0].ptp(), c[:, 1].ptp()))
        if ext > leg_cfg["leg_max_extent"]:
            continue
        comp.append((c.mean(axis=0), len(c)))
    if len(comp) < 2:
        return None
    L, tol = leg_cfg["rect_long"], leg_cfg["rect_tol"]
    best = None
    for i in range(len(comp)):
        for j in range(i + 1, len(comp)):
            ca, cb = comp[i][0], comp[j][0]
            if abs(abs(ca[1] - cb[1]) - L) > tol:
                continue
            xnear = min(ca[0], cb[0])
            score = abs(abs(ca[1] - cb[1]) - L) + 0.3 * xnear - 0.001 * (comp[i][1] + comp[j][1])
            if best is None or score < best[0]:
                best = (score, ca, cb)
    if best is None:
        return None
    _, ca, cb = best
    return 0.5 * (ca[1] + cb[1]), ca, cb


def select_table_legs(pts: np.ndarray, leg_cfg: dict):
    if len(pts) < leg_cfg["min_pts"]:
        return np.empty((0, 2)), np.empty((0, 2))
    labels = cluster_points(pts, leg_cfg["cluster_eps"])
    cand_c, cand_p = [], []
    for lb in np.unique(labels):
        c = pts[labels == lb]
        if len(c) < leg_cfg["min_pts"]:
            continue
        ext = float(max(c[:, 0].ptp(), c[:, 1].ptp()))
        if ext > leg_cfg["big_ext_max"]:
            continue
        cand_c.append(c.mean(axis=0))
        cand_p.append(c)
    if len(cand_c) < 4:
        return np.empty((0, 2)), np.empty((0, 2))
    if len(cand_c) > 24:
        order = np.argsort([-len(p) for p in cand_p])[:24]
        cand_c = [cand_c[i] for i in order]
        cand_p = [cand_p[i] for i in order]
    cents = np.asarray(cand_c)
    long_, short_, tol = leg_cfg["rect_long"], leg_cfg["rect_short"], leg_cfg["rect_tol"]
    best = None
    for idx in combinations(range(len(cents)), 4):
        Q = _order_corners(cents[list(idx)])
        sides = [float(np.linalg.norm(Q[i] - Q[(i + 1) % 4])) for i in range(4)]
        a, b = 0.5 * (sides[0] + sides[2]), 0.5 * (sides[1] + sides[3])
        s, l = min(a, b), max(a, b)
        if abs(s - short_) > tol or abs(l - long_) > tol:
            continue
        diag = [np.linalg.norm(Q[0] - Q[2]), np.linalg.norm(Q[1] - Q[3])]
        rect_err = abs(sides[0] - sides[2]) + abs(sides[1] - sides[3]) + abs(diag[0] - diag[1])
        dim_err = abs(s - short_) + abs(l - long_)
        npts = sum(len(cand_p[i]) for i in idx)
        score = dim_err + rect_err - 0.001 * npts
        if best is None or score < best[0]:
            best = (score, idx, Q)
    if best is None:
        return np.empty((0, 2)), np.empty((0, 2))
    leg_pts = np.vstack([cand_p[i] for i in best[1]])
    return best[2], leg_pts


class DockAligner:

    def __init__(self, cfg: dict):
        from ebim_nav.local_map import voxel_down
        self._voxel_down = voxel_down
        d = cfg["dock"]
        data = np.load(PKG_ROOT / d["ref_scan_file"])
        if "leg_pts" not in data:
            raise FileNotFoundError(
                f"{d['ref_scan_file']} 是旧格式（整框快照版），请重新采集: ./run.sh dock_ref")
        self.ref_pts = data["leg_pts"].astype(np.float64)
        self.ref_cent_mean = data["centroids"].astype(np.float64).mean(axis=0)
        self.ref_tree = cKDTree(self.ref_pts)
        fc = d["front_crop"]
        self.crop = (fc["x_min"], fc["x_max"], fc["y_min"], fc["y_max"])
        self.voxel = d["match_voxel"]
        self.legs = d["legs"]
        self.max_corr, self.iters = d["icp_max_corr"], d["icp_iters"]
        self.lock_fit, self.lock_min = d["lock_fitness"], d["lock_min_pts"]
        self.capture_r = d["capture_radius"]
        self.servo = ServoController(d)
        self.last_npts = 0
        self.last_nlegs = 0

    def estimate(self, pts_base: np.ndarray):
        front = crop_box(pts_base, self.crop[0], self.crop[1], self.crop[2], self.crop[3])
        front = self._voxel_down(front, self.voxel)
        cents, leg_pts = select_table_legs(front, self.legs)
        self.last_nlegs, self.last_npts = len(cents), len(leg_pts)
        if len(cents) < 4 or len(leg_pts) < self.lock_min:
            return DockEstimate(0, 0, 0, 0.0, 9.9, False), leg_pts, False
        dt = self.ref_cent_mean - cents.mean(axis=0)
        T0 = se2_matrix(dt[0], dt[1], 0.0)
        res = icp_2d(leg_pts, self.ref_pts, T_init=T0, dst_tree=self.ref_tree,
                     max_iters=self.iters, max_corr_dist=self.max_corr)
        ex, ey, eyaw = se2_params(res.T)
        est = DockEstimate(ex, ey, eyaw, res.fitness, res.rmse, res.fitness > 0.5)
        locked = est.ok and est.fitness >= self.lock_fit and est.err_xy < self.capture_r
        return est, leg_pts, locked

    def step(self, pts_base: np.ndarray):
        est, front, locked = self.estimate(pts_base)
        if not locked:
            self.servo._settle = 0
            return 0.0, 0.0, 0.0, False, False, est
        vx, vy, wz, converged = self.servo.step(est)
        return vx, vy, wz, True, converged, est


class DockLateral:

    def __init__(self, cfg: dict):
        from ebim_nav.local_map import voxel_down
        self._voxel = voxel_down
        d = cfg["dock"]
        fc = d["front_crop"]
        data = np.load(PKG_ROOT / d["ref_scan_file"])
        if "mid_y" not in data:
            raise FileNotFoundError(
                f"{d['ref_scan_file']} 无 mid_y（旧格式），请重采: ./run.sh dock_ref")
        self.ref_mid_y = float(data["mid_y"])
        self.crop = (fc["x_min"], fc["x_max"], fc["y_min"], fc["y_max"])
        self.voxel = d["match_voxel"]
        self.legs = d["legs"]
        self.tol = d["lat_tol"]
        self.speed = d["lat_speed"]
        self.max_iter = d["dock_max_iters"]
        self.max_yaw = d["max_vel_yaw"]
        self.timeout = cfg["approach"]["stage_timeout"]
        self.confirm_n = d["confirm_frames"]
        self.confirm_std = d["confirm_std"]

    def _midy_once(self, pts):
        front = self._voxel(crop_box(pts, *self.crop), self.voxel)
        r = front_leg_pair(front, self.legs)
        return None if r is None else r[0]

    def measure_midy(self, feed):
        vals, last, t0 = [], 0.0, time.time()
        while time.time() - t0 < 4.0:
            g = feed.latest()
            if g is None or g[0] <= last:
                time.sleep(0.02)
                continue
            last = g[0]
            my = self._midy_once(g[1])
            if my is not None:
                vals.append(my)
            if len(vals) >= self.confirm_n and float(np.std(vals[-self.confirm_n:])) <= self.confirm_std:
                return float(np.median(vals[-self.confirm_n:])), len(vals), None
        if len(vals) >= 3 and float(np.std(vals)) <= self.confirm_std:
            return float(np.median(vals)), len(vals), None
        return None, len(vals), None

    def run(self, feed, dry: bool = False) -> bool:
        for it in range(self.max_iter):
            feed.stop_cmd()
            time.sleep(0.4)
            my, ng, _ = self.measure_midy(feed)
            if my is None:
                print(f"\n✗ dock：{ng} 帧未稳定确认前2腿（车没正对桌 / 抖动 / front_crop 没框住）")
                return False
            dy = self.ref_mid_y - my
            print(f"  [dock#{it + 1}] 前2腿已确认({ng}帧) y中点={my:+.3f} 目标={self.ref_mid_y:+.3f} "
                  f"→ 需横移 Δy={dy:+.3f}m（{'左' if dy < 0 else '右'}）")
            if abs(dy) < self.tol:
                print(f"✅ dock 到位（|Δy|={abs(dy) * 100:.1f}cm < {self.tol * 100:.0f}cm）")
                return True
            if dry:
                print("  [DRY] 只测不横移")
                return True
            self._move(feed, -math.copysign(self.speed, dy), abs(dy))
        print("⚠️ dock 迭代到上限仍未进容差")
        return False

    def _move(self, feed, vy: float, dist: float):
        g = feed.latest()
        ox, oy = g[2][0], g[2][1]
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            g = feed.latest()
            if g is None:
                time.sleep(0.02)
                continue
            if math.hypot(g[2][0] - ox, g[2][1] - oy) >= dist:
                break
            feed.send(0.0, vy, 0.0, self.speed, self.max_yaw)
            time.sleep(0.02)
        feed.stop_cmd()
