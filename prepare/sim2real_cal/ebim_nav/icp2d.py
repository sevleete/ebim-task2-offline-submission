from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


def se2_matrix(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def se2_params(T: np.ndarray) -> tuple[float, float, float]:
    return float(T[0, 2]), float(T[1, 2]), float(np.arctan2(T[1, 0], T[0, 0]))


def transform_points(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    return pts @ T[:2, :2].T + T[:2, 2]


@dataclass
class ICPResult:
    T: np.ndarray
    fitness: float
    rmse: float
    iters: int
    converged: bool

    @property
    def pose(self) -> tuple[float, float, float]:
        return se2_params(self.T)


def icp_2d(
    src: np.ndarray,
    dst: np.ndarray,
    T_init: np.ndarray | None = None,
    max_iters: int = 40,
    max_corr_dist: float = 0.5,
    tol: float = 1e-5,
    dst_tree: cKDTree | None = None,
) -> ICPResult:
    if T_init is None:
        T_init = np.eye(3)
    if dst_tree is None:
        dst_tree = cKDTree(dst)

    T = T_init.copy()
    prev_err = np.inf
    it = 0
    fitness = 0.0
    rmse = np.inf
    converged = False

    for it in range(1, max_iters + 1):
        moved = transform_points(src, T)
        dists, idx = dst_tree.query(moved, distance_upper_bound=max_corr_dist)
        mask = np.isfinite(dists)
        n_in = int(mask.sum())
        fitness = n_in / len(src)
        if n_in < 10:
            break
        p = moved[mask]
        q = dst[idx[mask]]
        rmse = float(np.sqrt(np.mean(dists[mask] ** 2)))

        pc, qc = p.mean(axis=0), q.mean(axis=0)
        H = (p - pc).T @ (q - qc)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = qc - R @ pc
        dT = np.eye(3)
        dT[:2, :2] = R
        dT[:2, 2] = t
        T = dT @ T

        if abs(prev_err - rmse) < tol:
            converged = True
            break
        prev_err = rmse

    return ICPResult(T=T, fitness=fitness, rmse=rmse, iters=it, converged=converged)


def score_pose_likelihood(
    scan_pts: np.ndarray,
    pose: tuple[float, float, float],
    map_tree: cKDTree,
    sigma: float = 0.1,
    subsample: int = 4,
) -> float:
    pts = scan_pts[::subsample]
    moved = transform_points(pts, se2_matrix(*pose))
    dists, _ = map_tree.query(moved, distance_upper_bound=3.0 * sigma)
    dists = np.where(np.isfinite(dists), dists, 3.0 * sigma)
    return float(np.mean(np.exp(-0.5 * (dists / sigma) ** 2)))
