from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from ebim_nav.dock_lib import fit_line_ransac
from ebim_nav.icp2d import icp_2d, se2_matrix, se2_params, transform_points


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def se2_inv(T: np.ndarray) -> np.ndarray:
    Ti = np.eye(3)
    R = T[:2, :2]
    Ti[:2, :2] = R.T
    Ti[:2, 2] = -R.T @ T[:2, 2]
    return Ti


def voxel_down(pts: np.ndarray, size: float) -> np.ndarray:
    if len(pts) == 0:
        return pts
    keys = np.round(pts / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


@dataclass
class LocalPose:
    x: float
    y: float
    yaw: float
    fitness: float
    ok: bool

    @property
    def T(self) -> np.ndarray:
        return se2_matrix(self.x, self.y, self.yaw)


class LocalMap:
    def __init__(self, cfg_lm: dict):
        self.c = cfg_lm
        self.map_pts: np.ndarray | None = None
        self.tree: cKDTree | None = None
        self.T_pose = np.eye(3)
        self.odom_prev: np.ndarray | None = None
        self.T_last_key = np.eye(3)
        self.fitness = 0.0


    def _prep(self, pts: np.ndarray) -> np.ndarray:
        r = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[(r > 0.3) & (r < self.c["crop_radius"])]
        return voxel_down(pts[:: self.c["scan_subsample"]], self.c["voxel"])


    def update(self, scan_pts: np.ndarray, odom_xyyaw: tuple) -> LocalPose:
        cur = self._prep(scan_pts)
        odom_T = se2_matrix(*odom_xyyaw)

        if self.map_pts is None:
            self.map_pts = cur
            self.tree = cKDTree(cur)
            self.odom_prev = odom_T
            self.fitness = 1.0
            return LocalPose(0.0, 0.0, 0.0, 1.0, True)

        T_init = self.T_pose @ se2_inv(self.odom_prev) @ odom_T
        self.odom_prev = odom_T
        res = icp_2d(cur, self.map_pts, T_init=T_init, dst_tree=self.tree,
                     max_iters=self.c["icp_iters"], max_corr_dist=self.c["icp_max_corr"])
        ok = res.fitness >= self.c["fitness_min"]
        self.T_pose = res.T if ok else T_init
        self.fitness = res.fitness

        if ok:
            dT = se2_inv(self.T_last_key) @ self.T_pose
            dx, dy, dyaw = se2_params(dT)
            if (math.hypot(dx, dy) > self.c["keyframe_trans"]
                    or abs(dyaw) > math.radians(self.c["keyframe_rot_deg"])):
                merged = np.vstack([self.map_pts, transform_points(cur, self.T_pose)])
                merged = voxel_down(merged, self.c["voxel"])
                self.map_pts = merged[-self.c["max_points"]:]
                self.tree = cKDTree(self.map_pts)
                self.T_last_key = self.T_pose.copy()

        x, y, yaw = se2_params(self.T_pose)
        return LocalPose(x, y, yaw, res.fitness, ok)

    @property
    def pose(self) -> tuple[float, float, float]:
        return se2_params(self.T_pose)

    def points_in_base(self) -> np.ndarray:
        if self.map_pts is None:
            return np.empty((0, 2))
        return transform_points(self.map_pts, se2_inv(self.T_pose))


def rear_wall(pts_base: np.ndarray, wall_cfg: dict, rear_len: float):
    w = wall_cfg
    m = ((pts_base[:, 0] > w["x_min"]) & (pts_base[:, 0] < w["x_max"])
         & (np.abs(pts_base[:, 1]) < w["y_half"]))
    fit = fit_line_ransac(pts_base[m], thresh=0.03, iters=150)
    if fit is None:
        return None
    theta, d0, _ = fit
    if math.cos(theta) > 0:
        theta, d0 = wrap(theta + math.pi), -d0
    gap = abs(d0) - rear_len
    return gap, wrap(theta - math.pi)
