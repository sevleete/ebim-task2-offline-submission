#!/usr/bin/env python3
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


@dataclass
class Cluster:
    idx: np.ndarray
    pts: np.ndarray
    centroid: np.ndarray
    count: int
    extent: float


@dataclass
class Rectangle:
    corners: np.ndarray
    clusters: list
    center: np.ndarray
    short_side: float
    long_side: float
    diag: float
    yaw: float
    score: float


@dataclass
class DetectResult:
    rect: Rectangle | None
    candidates: list = field(default_factory=list)
    n_clusters: int = 0


def cluster_euclidean(pts: np.ndarray, eps: float, min_pts: int) -> list[np.ndarray]:
    n = len(pts)
    if n == 0:
        return []
    tree = cKDTree(pts)
    pairs = tree.query_pairs(r=eps, output_type="ndarray")
    if len(pairs) == 0:
        return []
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    _, labels = connected_components(g, directed=False)
    order = np.argsort(labels, kind="stable")
    groups = np.split(order, np.nonzero(np.diff(labels[order]))[0] + 1)
    return [grp for grp in groups if len(grp) >= min_pts]


def cluster_features(pts: np.ndarray, idx: np.ndarray) -> Cluster:
    P = pts[idx]
    mn, mx = P.min(axis=0), P.max(axis=0)
    return Cluster(idx=idx, pts=P, centroid=P.mean(axis=0),
                   count=len(idx), extent=float(np.hypot(*(mx - mn))))


def filter_leg_candidates(clusters: list[Cluster], det: dict) -> list[Cluster]:
    roi = det.get("roi")
    out = []
    for c in clusters:
        if not (det["min_pts"] <= c.count <= det["max_pts"]):
            continue
        if c.extent > det["leg_max_extent"]:
            continue
        if roi:
            x, y = c.centroid
            if not (roi["x_min"] < x < roi["x_max"] and roi["y_min"] < y < roi["y_max"]):
                continue
        out.append(c)
    return out


def _rect_metrics(P4: np.ndarray):
    ctr = P4.mean(axis=0)
    order = np.argsort(np.arctan2(P4[:, 1] - ctr[1], P4[:, 0] - ctr[0]))
    q = P4[order]
    s = [np.linalg.norm(q[(k + 1) % 4] - q[k]) for k in range(4)]
    d02 = np.linalg.norm(q[2] - q[0])
    d13 = np.linalg.norm(q[3] - q[1])
    w, h, diag = 0.5 * (s[0] + s[2]), 0.5 * (s[1] + s[3]), 0.5 * (d02 + d13)
    if w < 1e-6 or h < 1e-6 or diag < 1e-6:
        return None
    devs = []
    for k in range(4):
        v1, v2 = q[(k - 1) % 4] - q[k], q[(k + 1) % 4] - q[k]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            devs.append(math.pi)
            continue
        devs.append(abs(math.acos(np.clip(np.dot(v1, v2) / (n1 * n2), -1, 1)) - math.pi / 2))
    return dict(order=order, q=q, ctr=ctr,
                e_sideA=abs(s[0] - s[2]) / w, e_sideB=abs(s[1] - s[3]) / h,
                e_diag=abs(d02 - d13) / diag, ang_dev=float(np.mean(devs)),
                w=w, h=h, diag=diag)


def find_rectangle(cands: list[Cluster], det: dict) -> Rectangle | None:
    M = len(cands)
    if M < 4:
        return None
    C = np.array([c.centroid for c in cands])
    ang_tol = math.radians(det["rect_angle_tol_deg"])
    span, span_tol = det.get("leg_span"), det.get("span_tol", 0.05)
    best, best_score = None, -1.0
    for combo in itertools.combinations(range(M), 4):
        m = _rect_metrics(C[list(combo)])
        if m is None:
            continue
        if max(m["e_sideA"], m["e_sideB"], m["e_diag"]) > det["rect_side_tol"]:
            continue
        if m["ang_dev"] > ang_tol:
            continue
        short, long_ = sorted([m["w"], m["h"]])
        if span and (abs(short - span[0]) > span_tol or abs(long_ - span[1]) > span_tol):
            continue
        e_ang = m["ang_dev"] / ang_tol if ang_tol > 0 else 0.0
        score = 1.0 - min(max(0.25 * (m["e_sideA"] + m["e_sideB"] + m["e_diag"] + e_ang), 0.0), 1.0)
        if score > best_score:
            best_score, best = score, (combo, m, short, long_)
    if best is None or best_score < det["rect_min_score"]:
        return None
    combo, m, short, long_ = best
    corners = m["q"]
    clusters4 = [cands[combo[k]] for k in m["order"]]
    e1, e3 = corners[1] - corners[0], corners[3] - corners[0]
    edge = e1 if np.linalg.norm(e1) >= np.linalg.norm(e3) else e3
    yaw = math.atan2(edge[1], edge[0])
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
    return Rectangle(corners=corners, clusters=clusters4, center=m["ctr"],
                     short_side=short, long_side=long_, diag=m["diag"],
                     yaw=yaw, score=best_score)


def detect_legs(pts: np.ndarray, det: dict) -> DetectResult:
    if pts is None or len(pts) == 0:
        return DetectResult(None, [], 0)
    clusters = [cluster_features(pts, g) for g in
                cluster_euclidean(pts, det["cluster_eps"], det["min_pts"])]
    cands = filter_leg_candidates(clusters, det)
    return DetectResult(find_rectangle(cands, det), cands, len(clusters))
