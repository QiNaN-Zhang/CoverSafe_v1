#!/usr/bin/env python3
"""Phase 3: station-level mission generation.

Pipeline:
1) Read Phase 2 per-device ordered viewpoints/reports.
2) Compress each device route with coverage-preserving heuristics.
3) Build station-level visiting order by fixed region sequence + within-region optimization.
4) Plan a full mission path on Phase 1 feasible-space grid (straight + A* fallback).
5) Apply safety-aware smoothing and evaluate trackability metrics.
6) Export mission artifacts, summary stats, supplementary scenario experiments, and visualization.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import itertools
import json
import math
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import laspy
from matplotlib import animation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


def log(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8-sig") as f:
        return json.load(f)


def write_json(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def rotate_xy(points: np.ndarray, angle_rad: float) -> np.ndarray:
    if abs(angle_rad) < 1e-12:
        return points.copy()
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    out = points.copy()
    x = points[:, 0]
    y = points[:, 1]
    out[:, 0] = c * x - s * y
    out[:, 1] = s * x + c * y
    return out


def deep_update(base: dict, patch: dict) -> dict:
    out = deepcopy(base)
    for k, v in patch.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_update(out[k], v)
        else:
            out[k] = deepcopy(v)
    return out


def safe_float(v: object, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def safe_int(v: object, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return default


@dataclass
class CapturePoint:
    viewpoint_id: int
    device: str
    device_type: str
    x: float
    y: float
    z: float
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    target_x: float
    target_y: float
    target_z: float
    layer_k: int
    component_id: int
    source_row_index: int


@dataclass
class DeviceData:
    name: str
    device_type: str
    captures: List[CapturePoint]
    nav_points: np.ndarray
    phase2_coverage_ratio: float
    sampled_surface_points: int
    phase2_path_segment_success_ratio: float
    phase2_path_length_m: float
    phase2_viewpoints: int


@dataclass
class CompressedDeviceRoute:
    name: str
    device_type: str
    captures: List[CapturePoint]
    nav_points: np.ndarray
    kept_indices: List[int]
    raw_count: int
    retained_count: int
    retention_ratio: float
    estimated_coverage_ratio: float
    region: str

    def ordered_by_state(self, reverse: bool) -> Tuple[List[CapturePoint], np.ndarray]:
        if not reverse:
            return self.captures, self.nav_points
        rev_caps = list(reversed(self.captures))
        rev_nav = self.nav_points[::-1].copy()
        return rev_caps, rev_nav


class StationGrid:
    def __init__(self, npz_path: Path) -> None:
        d = np.load(npz_path)
        self.origin = d["origin"].astype(np.float64)
        self.voxel = float(d["voxel_size"][0])
        self.shape = d["shape"].astype(np.int32)
        blocked = d["blocked"].astype(np.int32)
        self.blocked = np.zeros(tuple(int(v) for v in self.shape.tolist()), dtype=np.bool_)
        if blocked.size > 0:
            self.blocked[blocked[:, 0], blocked[:, 1], blocked[:, 2]] = True

    def world_to_idx(self, p: np.ndarray) -> np.ndarray:
        return np.floor((p - self.origin) / self.voxel).astype(np.int32)

    def idx_to_world_center(self, idx: np.ndarray) -> np.ndarray:
        return self.origin + (idx.astype(np.float64) + 0.5) * self.voxel

    def in_bounds_idx(self, idx: np.ndarray) -> bool:
        return bool(np.all((idx >= 0) & (idx < self.shape)))

    def is_free_world(self, p: np.ndarray) -> bool:
        idx = self.world_to_idx(p)
        if not self.in_bounds_idx(idx):
            return False
        return not bool(self.blocked[idx[0], idx[1], idx[2]])

    def line_free_world(self, a: np.ndarray, b: np.ndarray, step_scale: float = 0.8) -> bool:
        dist = float(np.linalg.norm(b - a))
        n = max(2, int(math.ceil(dist / max(self.voxel * step_scale, 1e-6))))
        for i in range(n + 1):
            t = i / n
            p = a * (1.0 - t) + b * t
            if not self.is_free_world(p):
                return False
        return True

    def astar_world(
        self,
        start_w: np.ndarray,
        goal_w: np.ndarray,
        margin_m: float,
        connectivity: int,
        max_expansions: int,
    ) -> Optional[np.ndarray]:
        s = self.world_to_idx(start_w)
        g = self.world_to_idx(goal_w)
        if not self.in_bounds_idx(s) or not self.in_bounds_idx(g):
            return None
        if self.blocked[s[0], s[1], s[2]] or self.blocked[g[0], g[1], g[2]]:
            return None

        margin_k = max(2, int(math.ceil(margin_m / self.voxel)))
        lo = np.maximum(np.minimum(s, g) - margin_k, 0)
        hi = np.minimum(np.maximum(s, g) + margin_k, self.shape - 1)

        if connectivity == 6:
            dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        else:
            dirs = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1) if not (dx == 0 and dy == 0 and dz == 0)]

        st = (int(s[0]), int(s[1]), int(s[2]))
        gt = (int(g[0]), int(g[1]), int(g[2]))
        g_score: Dict[Tuple[int, int, int], float] = {st: 0.0}
        parent: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
        open_heap: List[Tuple[float, Tuple[int, int, int]]] = []
        heapq.heappush(open_heap, (float(np.linalg.norm(s - g)), st))
        closed: Set[Tuple[int, int, int]] = set()
        expansions = 0

        while open_heap and expansions < max_expansions:
            _, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            if cur == gt:
                path_idx: List[Tuple[int, int, int]] = [cur]
                while path_idx[-1] in parent:
                    path_idx.append(parent[path_idx[-1]])
                path_idx.reverse()
                arr = np.array(path_idx, dtype=np.int32)
                return self.idx_to_world_center(arr)
            closed.add(cur)
            expansions += 1

            cx, cy, cz = cur
            for dx, dy, dz in dirs:
                nx, ny, nz = cx + dx, cy + dy, cz + dz
                if nx < lo[0] or ny < lo[1] or nz < lo[2] or nx > hi[0] or ny > hi[1] or nz > hi[2]:
                    continue
                if self.blocked[nx, ny, nz]:
                    continue
                nxt = (nx, ny, nz)
                if nxt in closed:
                    continue
                step_cost = math.sqrt(dx * dx + dy * dy + dz * dz)
                cand = g_score[cur] + step_cost
                if cand < g_score.get(nxt, float("inf")):
                    g_score[nxt] = cand
                    parent[nxt] = cur
                    h = math.sqrt((nx - gt[0]) ** 2 + (ny - gt[1]) ** 2 + (nz - gt[2]) ** 2)
                    heapq.heappush(open_heap, (cand + h, nxt))
        return None


def parse_capture_csv(path: Path) -> List[CapturePoint]:
    out: List[CapturePoint] = []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        r = csv.DictReader(f)
        for i, row in enumerate(r):
            out.append(
                CapturePoint(
                    viewpoint_id=safe_int(row.get("viewpoint_id"), i),
                    device=str(row.get("device", "")).strip(),
                    device_type=str(row.get("device_type", "default")).strip() or "default",
                    x=safe_float(row.get("x")),
                    y=safe_float(row.get("y")),
                    z=safe_float(row.get("z")),
                    yaw_deg=safe_float(row.get("yaw_deg")),
                    pitch_deg=safe_float(row.get("pitch_deg")),
                    roll_deg=safe_float(row.get("roll_deg")),
                    target_x=safe_float(row.get("target_x")),
                    target_y=safe_float(row.get("target_y")),
                    target_z=safe_float(row.get("target_z")),
                    layer_k=safe_int(row.get("layer_k"), 0),
                    component_id=safe_int(row.get("component_id"), 0),
                    source_row_index=i,
                )
            )
    return out


def parse_nav_csv(path: Path) -> np.ndarray:
    pts: List[List[float]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            pts.append([safe_float(row.get("x")), safe_float(row.get("y")), safe_float(row.get("z"))])
    if not pts:
        return np.empty((0, 3), dtype=np.float64)
    return np.array(pts, dtype=np.float64)


def load_device_data(phase2_out: Path, needed_devices: Iterable[str]) -> Dict[str, DeviceData]:
    data: Dict[str, DeviceData] = {}
    for name in sorted(set(str(x) for x in needed_devices)):
        dev_dir = phase2_out / name
        cap_csv = dev_dir / "viewpoints_capture_ordered.csv"
        nav_csv = dev_dir / "viewpoints_nav_ordered.csv"
        rep_json = dev_dir / "device_report.json"
        if (not cap_csv.exists()) or (not nav_csv.exists()) or (not rep_json.exists()):
            raise FileNotFoundError(f"Missing Phase2 artifacts for device '{name}' in {dev_dir}")

        caps = parse_capture_csv(cap_csv)
        nav = parse_nav_csv(nav_csv)
        rep = load_json(rep_json)
        n = min(len(caps), int(nav.shape[0]))
        if n <= 0:
            raise RuntimeError(f"Device '{name}' has empty ordered viewpoints in Phase2 outputs")
        caps = caps[:n]
        nav = nav[:n]
        dtype = str(rep.get("type", caps[0].device_type if caps else "default"))
        cov = float(rep.get("coverage", {}).get("coverage_ratio", 0.0))
        sampled = int(rep.get("coverage", {}).get("sampled_surface_points", 0))
        p_success = float(rep.get("planning", {}).get("segment_success_ratio", 0.0))
        p_len = float(rep.get("planning", {}).get("total_length_m", 0.0))
        p_vp = int(rep.get("ordered_viewpoints", n))
        data[name] = DeviceData(
            name=name,
            device_type=dtype,
            captures=caps,
            nav_points=nav,
            phase2_coverage_ratio=cov,
            sampled_surface_points=sampled,
            phase2_path_segment_success_ratio=p_success,
            phase2_path_length_m=p_len,
            phase2_viewpoints=p_vp,
        )
    return data


def sample_las_rgb_cloud(
    las_path: Path,
    max_points: int,
    chunk_size: int,
    seed: int,
    angle_rad: float,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    if (not las_path.exists()) or max_points <= 0:
        return np.empty((0, 3), dtype=np.float64), None
    rng = np.random.default_rng(seed)
    xyz_parts: List[np.ndarray] = []
    rgb_parts: List[np.ndarray] = []
    with laspy.open(las_path) as reader:
        total = int(max(1, getattr(reader.header, "point_count", 1)))
        keep_prob = min(1.0, float(max_points) / float(total))
        for pts in reader.chunk_iterator(int(max(1, chunk_size))):
            n = len(pts.x)
            if n <= 0:
                continue
            mask = rng.random(n) < keep_prob
            if not np.any(mask):
                continue
            x = np.asarray(pts.x)[mask]
            y = np.asarray(pts.y)[mask]
            z = np.asarray(pts.z)[mask]
            arr = np.column_stack((x, y, z)).astype(np.float64, copy=False)
            arr = rotate_xy(arr, angle_rad)
            xyz_parts.append(arr)
            if hasattr(pts, "red") and hasattr(pts, "green") and hasattr(pts, "blue"):
                rr = np.asarray(pts.red)[mask].astype(np.float64, copy=False)
                gg = np.asarray(pts.green)[mask].astype(np.float64, copy=False)
                bb = np.asarray(pts.blue)[mask].astype(np.float64, copy=False)
                maxc = max(float(np.max(rr)) if rr.size else 0.0, float(np.max(gg)) if gg.size else 0.0, float(np.max(bb)) if bb.size else 0.0)
                scale = 65535.0 if maxc > 255.0 else 255.0
                rgb = np.column_stack((rr / scale, gg / scale, bb / scale))
                rgb = np.clip(rgb, 0.0, 1.0)
                rgb_parts.append(rgb.astype(np.float64, copy=False))

    if not xyz_parts:
        return np.empty((0, 3), dtype=np.float64), None
    xyz = np.vstack(xyz_parts)
    rgb = np.vstack(rgb_parts) if rgb_parts else None
    n_all = int(xyz.shape[0])
    if rgb is not None and int(rgb.shape[0]) != n_all:
        rgb = None

    if n_all > max_points:
        pick = rng.choice(n_all, size=max_points, replace=False)
        xyz = xyz[pick]
        if rgb is not None:
            rgb = rgb[pick]
    return xyz, rgb


def _angle_rad(u: np.ndarray, v: np.ndarray) -> float:
    nu = float(np.linalg.norm(u))
    nv = float(np.linalg.norm(v))
    if nu < 1e-9 or nv < 1e-9:
        return 0.0
    c = float(np.dot(u, v) / (nu * nv))
    c = max(-1.0, min(1.0, c))
    return float(math.acos(c))


def compress_device_route(device: DeviceData, region: str, cfg: dict) -> CompressedDeviceRoute:
    ccfg = cfg["compression"]
    nav = device.nav_points
    caps = device.captures
    n = int(nav.shape[0])
    if n <= 2 or (not bool(ccfg.get("enable", True))):
        kept = list(range(n))
        est_cov = float(device.phase2_coverage_ratio)
        return CompressedDeviceRoute(
            name=device.name,
            device_type=device.device_type,
            captures=[caps[i] for i in kept],
            nav_points=nav[kept].copy(),
            kept_indices=kept,
            raw_count=n,
            retained_count=n,
            retention_ratio=1.0,
            estimated_coverage_ratio=est_cov,
            region=region,
        )

    min_keep = int(max(2, ccfg.get("min_keep_per_device", 8)))
    merge_dist = float(max(0.05, ccfg.get("merge_distance_m", 0.8)))
    merged: List[int] = [0]
    for i in range(1, n - 1):
        last = merged[-1]
        d = float(np.linalg.norm(nav[i] - nav[last]))
        if d >= merge_dist:
            merged.append(i)
    if n > 1 and merged[-1] != n - 1:
        merged.append(n - 1)
    if len(merged) < min_keep:
        merged = list(range(n))

    m = len(merged)
    if m <= min_keep + 1:
        kept = merged
    else:
        max_drop_ratio = float(np.clip(ccfg.get("max_drop_ratio", 0.35), 0.0, 0.95))
        max_cov_loss = float(np.clip(ccfg.get("max_coverage_loss_ratio", 0.15), 0.0, 0.95))
        min_cov_loss = float(np.clip(ccfg.get("min_coverage_loss_ratio", 0.04), 0.0, max_cov_loss))
        cov_ref = float(max(1e-6, ccfg.get("coverage_reference", 0.7)))
        cov_guard = float(np.clip(device.phase2_coverage_ratio / cov_ref, 0.4, 1.0))
        allowed_cov_loss = float(max(min_cov_loss, max_cov_loss * cov_guard))
        by_ratio = int(math.floor(m * max_drop_ratio))
        by_cov = int(math.floor(m * allowed_cov_loss))
        by_keep = max(0, m - min_keep)
        max_drop = int(max(0, min(by_ratio, by_cov, by_keep)))

        radial_q = float(np.clip(ccfg.get("radial_quantile", 0.88), 0.5, 0.99))
        center = np.median(nav[merged], axis=0)
        radial = np.linalg.norm(nav - center[None, :], axis=1)
        radial_thr = float(np.quantile(radial[merged], radial_q))

        w = ccfg.get("score_weights", {})
        w_col = float(w.get("collinear", 0.45))
        w_extra = float(w.get("extra_path", 0.30))
        w_rad = float(w.get("radial", 0.25))

        score_items: List[Tuple[float, int]] = []
        for j in range(1, m - 1):
            idx_prev = merged[j - 1]
            idx_cur = merged[j]
            idx_next = merged[j + 1]
            p_prev = nav[idx_prev]
            p_cur = nav[idx_cur]
            p_next = nav[idx_next]

            d1 = float(np.linalg.norm(p_cur - p_prev))
            d2 = float(np.linalg.norm(p_next - p_cur))
            d_skip = float(np.linalg.norm(p_next - p_prev))
            angle = _angle_rad(p_cur - p_prev, p_next - p_cur)
            turn_norm = angle / math.pi
            extra_ratio = (d1 + d2 - d_skip) / max(d1 + d2, 1e-6)
            radial_excess = max(0.0, float(radial[idx_cur] - radial_thr))
            radial_norm = radial_excess / max(radial_thr, 1e-6)
            remove_score = w_col * (1.0 - turn_norm) + w_extra * extra_ratio + w_rad * radial_norm
            score_items.append((remove_score, idx_cur))

        score_items.sort(key=lambda x: x[0], reverse=True)
        to_remove = {idx for _, idx in score_items[:max_drop]}
        kept = [idx for idx in merged if idx not in to_remove]
        if len(kept) < min_keep:
            kept = merged[:]

    kept = sorted(set(int(x) for x in kept))
    if kept[0] != 0:
        kept = [0] + kept
    if kept[-1] != (n - 1):
        kept.append(n - 1)
    kept = sorted(set(kept))
    retained = len(kept)
    ratio = float(retained / max(1, n))
    est_cov = float(np.clip(device.phase2_coverage_ratio * ratio, 0.0, 1.0))

    return CompressedDeviceRoute(
        name=device.name,
        device_type=device.device_type,
        captures=[caps[i] for i in kept],
        nav_points=nav[kept].copy(),
        kept_indices=kept,
        raw_count=n,
        retained_count=retained,
        retention_ratio=ratio,
        estimated_coverage_ratio=est_cov,
        region=region,
    )


def nearest_free_world(station: StationGrid, p: np.ndarray, max_k: int) -> np.ndarray:
    idx = station.world_to_idx(p)
    if station.in_bounds_idx(idx) and (not station.blocked[idx[0], idx[1], idx[2]]):
        return p.copy()
    if not station.in_bounds_idx(idx):
        idx = np.clip(idx, [0, 0, 0], station.shape - 1)
    ix, iy, iz = int(idx[0]), int(idx[1]), int(idx[2])
    for k in range(1, max(1, int(max_k)) + 1):
        x0, x1 = max(0, ix - k), min(int(station.shape[0]) - 1, ix + k)
        y0, y1 = max(0, iy - k), min(int(station.shape[1]) - 1, iy + k)
        z0, z1 = max(0, iz - k), min(int(station.shape[2]) - 1, iz + k)
        sub = station.blocked[x0 : x1 + 1, y0 : y1 + 1, z0 : z1 + 1]
        free = np.argwhere(~sub)
        if free.size == 0:
            continue
        free_abs = free + np.array([x0, y0, z0], dtype=np.int32)[None, :]
        d2 = np.sum((free_abs - idx[None, :]) ** 2, axis=1)
        pick = int(np.argmin(d2))
        best_idx = free_abs[pick].astype(np.int32)
        return station.idx_to_world_center(best_idx)
    return p.copy()


def build_region_maps(region_sequence: Sequence[dict]) -> Tuple[Dict[str, str], List[str], List[str]]:
    device_to_region: Dict[str, str] = {}
    expected_region_order: List[str] = []
    expected_device_order: List[str] = []
    for i, reg in enumerate(region_sequence):
        rname = str(reg.get("name", f"region_{i+1}"))
        expected_region_order.append(rname)
        devs = [str(d) for d in reg.get("devices", [])]
        for d in devs:
            device_to_region[d] = rname
            expected_device_order.append(d)
    return device_to_region, expected_region_order, expected_device_order


def enumerate_orders(region_sequence: Sequence[dict], optimize_within_region: bool) -> List[List[str]]:
    per_region_orders: List[List[Tuple[str, ...]]] = []
    for reg in region_sequence:
        devs = [str(d) for d in reg.get("devices", [])]
        if not devs:
            continue
        if optimize_within_region and len(devs) > 1:
            perms = sorted(set(itertools.permutations(devs, len(devs))))
            per_region_orders.append(list(perms))
        else:
            per_region_orders.append([tuple(devs)])
    out: List[List[str]] = []
    for combo in itertools.product(*per_region_orders):
        seq: List[str] = []
        for block in combo:
            seq.extend(list(block))
        out.append(seq)
    return out


def _route_endpoint(route: CompressedDeviceRoute, reverse: bool, which: str) -> np.ndarray:
    if which == "entry":
        return route.nav_points[-1] if reverse else route.nav_points[0]
    return route.nav_points[0] if reverse else route.nav_points[-1]


def estimate_connection_cost(
    a: np.ndarray,
    b: np.ndarray,
    station: StationGrid,
    cfg: dict,
    cache: Dict[Tuple[int, ...], float],
) -> float:
    qa = tuple(int(round(v * 20.0)) for v in a.tolist())
    qb = tuple(int(round(v * 20.0)) for v in b.tolist())
    key = qa + qb
    key_rev = qb + qa
    if key in cache:
        return cache[key]
    if key_rev in cache:
        return cache[key_rev]
    dist = float(np.linalg.norm(b - a))
    if dist < 1e-9:
        cache[key] = 0.0
        cache[key_rev] = 0.0
        return 0.0
    mode = str(cfg.get("mode", "astar_hybrid")).strip().lower()
    if mode == "euclidean":
        cache[key] = dist
        cache[key_rev] = dist
        return dist
    if mode in ("line_or_euclidean", "line_euclidean"):
        line_step = float(cfg.get("line_free_step_scale", 0.8))
        if station.line_free_world(a, b, step_scale=line_step):
            cache[key] = dist
            cache[key_rev] = dist
            return dist
        mult = float(max(1.0, cfg.get("blocked_distance_multiplier", 1.15)))
        c = float(dist * mult)
        cache[key] = c
        cache[key_rev] = c
        return c
    line_step = float(cfg.get("line_free_step_scale", 0.8))
    if station.line_free_world(a, b, step_scale=line_step):
        cache[key] = dist
        cache[key_rev] = dist
        return dist

    astar = station.astar_world(
        start_w=a,
        goal_w=b,
        margin_m=float(cfg.get("astar_roi_margin_m", 10.0)),
        connectivity=int(cfg.get("astar_connectivity", 26)),
        max_expansions=int(cfg.get("astar_max_expansions", 120000)),
    )
    if astar is not None and astar.shape[0] >= 2:
        cost = float(np.linalg.norm(astar[1:] - astar[:-1], axis=1).sum())
        cache[key] = cost
        cache[key_rev] = cost
        return cost

    fallback = float(dist * float(cfg.get("fallback_distance_multiplier", 2.6)) + float(cfg.get("fallback_penalty_m", 40.0)))
    cache[key] = fallback
    cache[key_rev] = fallback
    return fallback


def optimize_orientation_for_order(
    order: Sequence[str],
    routes: Dict[str, CompressedDeviceRoute],
    start_pt: np.ndarray,
    return_home: bool,
    station: StationGrid,
    conn_cfg: dict,
    cache: Dict[Tuple[int, ...], float],
) -> Tuple[float, List[bool]]:
    n = len(order)
    inf = float("inf")
    dp = np.full((n, 2), inf, dtype=np.float64)
    parent = np.full((n, 2), -1, dtype=np.int32)

    for s in (0, 1):
        rev = bool(s == 1)
        ent = _route_endpoint(routes[order[0]], reverse=rev, which="entry")
        dp[0, s] = estimate_connection_cost(start_pt, ent, station, conn_cfg, cache)

    for i in range(1, n):
        r_cur = routes[order[i]]
        for s in (0, 1):
            rev_cur = bool(s == 1)
            cur_ent = _route_endpoint(r_cur, reverse=rev_cur, which="entry")
            best = inf
            best_p = -1
            for p in (0, 1):
                if not np.isfinite(dp[i - 1, p]):
                    continue
                rev_prev = bool(p == 1)
                prev_exit = _route_endpoint(routes[order[i - 1]], reverse=rev_prev, which="exit")
                cand = float(dp[i - 1, p]) + estimate_connection_cost(prev_exit, cur_ent, station, conn_cfg, cache)
                if cand < best:
                    best = cand
                    best_p = p
            dp[i, s] = best
            parent[i, s] = best_p

    best_final = inf
    best_state = -1
    for s in (0, 1):
        val = float(dp[n - 1, s])
        if return_home:
            rev = bool(s == 1)
            ex = _route_endpoint(routes[order[-1]], reverse=rev, which="exit")
            val += estimate_connection_cost(ex, start_pt, station, conn_cfg, cache)
        if val < best_final:
            best_final = val
            best_state = s

    states = [False] * n
    cur_state = int(best_state)
    for i in range(n - 1, -1, -1):
        states[i] = bool(cur_state == 1)
        cur_state = int(parent[i, cur_state]) if i > 0 else -1

    return float(best_final), states


def select_best_device_order(
    region_sequence: Sequence[dict],
    routes: Dict[str, CompressedDeviceRoute],
    start_pt: np.ndarray,
    return_home: bool,
    station: StationGrid,
    ordering_cfg: dict,
) -> Tuple[List[str], List[bool], float, int]:
    strategy = str(ordering_cfg.get("strategy", "region_constrained")).strip().lower()
    if strategy == "greedy_biased":
        return select_device_order_greedy_biased(
            routes=routes,
            start_pt=start_pt,
            return_home=return_home,
            station=station,
            ordering_cfg=ordering_cfg,
        )
    if strategy in ("priority_regularized", "priority_regularized_greedy"):
        return select_device_order_priority_regularized(
            routes=routes,
            start_pt=start_pt,
            return_home=return_home,
            station=station,
            ordering_cfg=ordering_cfg,
        )
    optimize_within = bool(ordering_cfg.get("optimize_within_region", True))
    all_orders = enumerate_orders(region_sequence, optimize_within_region=optimize_within)
    conn_cfg = ordering_cfg.get("connection_cost", {})
    cache: Dict[Tuple[int, ...], float] = {}
    best_cost = float("inf")
    best_order: List[str] = []
    best_states: List[bool] = []
    for ord_candidate in all_orders:
        cost, states = optimize_orientation_for_order(
            order=ord_candidate,
            routes=routes,
            start_pt=start_pt,
            return_home=return_home,
            station=station,
            conn_cfg=conn_cfg,
            cache=cache,
        )
        if cost < best_cost:
            best_cost = cost
            best_order = list(ord_candidate)
            best_states = list(states)
    return best_order, best_states, float(best_cost), int(len(all_orders))


def _nearest_next_estimate(
    exit_pt: np.ndarray,
    unvisited: Sequence[str],
    routes: Dict[str, CompressedDeviceRoute],
    station: StationGrid,
    conn_cfg: dict,
    cache: Dict[Tuple[int, ...], float],
) -> float:
    if not unvisited:
        return 0.0
    best = float("inf")
    for d in unvisited:
        for rev in (False, True):
            ent = _route_endpoint(routes[d], reverse=rev, which="entry")
            c = estimate_connection_cost(exit_pt, ent, station, conn_cfg, cache)
            if c < best:
                best = c
    return float(best if np.isfinite(best) else 0.0)


def select_device_order_greedy_biased(
    routes: Dict[str, CompressedDeviceRoute],
    start_pt: np.ndarray,
    return_home: bool,
    station: StationGrid,
    ordering_cfg: dict,
) -> Tuple[List[str], List[bool], float, int]:
    conn_cfg = ordering_cfg.get("connection_cost", {})
    gb = ordering_cfg.get("greedy_biased", {})
    pref_list = [str(x) for x in gb.get("algorithmic_preference_order", [])]
    pref_rank = {d: i for i, d in enumerate(pref_list)}
    n_dev = max(1, len(routes))
    w_dist = float(gb.get("distance_weight", 1.0))
    w_pref_m = float(gb.get("preference_weight_m", 30.0))
    w_look = float(gb.get("lookahead_weight", 0.1))
    w_region = float(gb.get("region_switch_penalty_m", 6.0))

    unvisited = set(routes.keys())
    order: List[str] = []
    states: List[bool] = []
    current = start_pt.copy()
    prev_region: Optional[str] = None
    total_cost = 0.0
    cache: Dict[Tuple[int, ...], float] = {}

    while unvisited:
        best_score = float("inf")
        best: Optional[Tuple[str, bool, float]] = None
        for d in sorted(unvisited):
            rt = routes[d]
            rank = int(pref_rank.get(d, n_dev))
            pref_cost = float(rank / max(1, n_dev - 1)) * w_pref_m
            region_pen = w_region if (prev_region is not None and rt.region != prev_region) else 0.0
            for rev in (False, True):
                ent = _route_endpoint(rt, reverse=rev, which="entry")
                ex = _route_endpoint(rt, reverse=rev, which="exit")
                travel = estimate_connection_cost(current, ent, station, conn_cfg, cache)
                rest = [x for x in unvisited if x != d]
                look = _nearest_next_estimate(ex, rest, routes, station, conn_cfg, cache) if rest else 0.0
                score = w_dist * travel + pref_cost + region_pen + w_look * look
                if score < best_score:
                    best_score = score
                    best = (d, rev, float(travel))
        if best is None:
            break
        d_pick, rev_pick, travel_pick = best
        order.append(d_pick)
        states.append(bool(rev_pick))
        total_cost += float(travel_pick)
        prev_region = routes[d_pick].region
        current = _route_endpoint(routes[d_pick], reverse=bool(rev_pick), which="exit")
        unvisited.remove(d_pick)

    if return_home and order:
        total_cost += estimate_connection_cost(current, start_pt, station, conn_cfg, cache)
    return order, states, float(total_cost), 1


def select_device_order_priority_regularized(
    routes: Dict[str, CompressedDeviceRoute],
    start_pt: np.ndarray,
    return_home: bool,
    station: StationGrid,
    ordering_cfg: dict,
) -> Tuple[List[str], List[bool], float, int]:
    conn_cfg = ordering_cfg.get("connection_cost", {})
    pcfg = ordering_cfg.get("priority_regularized", {})
    pref_list = [str(x) for x in pcfg.get("algorithmic_preference_order", [])]
    if not pref_list:
        pref_list = sorted([str(k) for k in routes.keys()])
    pref_rank = {d: i for i, d in enumerate(pref_list)}
    n_dev = max(1, len(routes))
    w_dist = float(pcfg.get("distance_weight", 1.0))
    w_rank = float(pcfg.get("rank_weight", 0.0))
    w_viol = float(pcfg.get("violation_weight", 64.0))
    w_region = float(pcfg.get("region_switch_penalty_m", 0.0))
    w_look = float(pcfg.get("lookahead_weight", 0.0))
    use_conn = bool(pcfg.get("use_connection_cost", False))

    cache: Dict[Tuple[int, ...], float] = {}

    def _travel_cost(a: np.ndarray, b: np.ndarray) -> float:
        if use_conn:
            return estimate_connection_cost(a, b, station, conn_cfg, cache)
        return float(np.linalg.norm(b - a))

    unvisited = set(routes.keys())
    order: List[str] = []
    states: List[bool] = []
    current = start_pt.copy()
    prev_region: Optional[str] = None
    total_cost = 0.0

    while unvisited:
        best_key = (float("inf"), float("inf"), float("inf"))
        best: Optional[Tuple[str, bool, float, np.ndarray]] = None
        for d in sorted(unvisited):
            rt = routes[d]
            rank = int(pref_rank.get(d, n_dev))
            rank_cost = float(rank / max(1, n_dev - 1))
            violation = sum(1 for u in unvisited if int(pref_rank.get(u, n_dev)) < rank)
            region_pen = w_region if (prev_region is not None and rt.region != prev_region) else 0.0
            for rev in (False, True):
                ent = _route_endpoint(rt, reverse=rev, which="entry")
                ex = _route_endpoint(rt, reverse=rev, which="exit")
                travel = _travel_cost(current, ent)
                look = 0.0
                if w_look > 0.0:
                    rest = [x for x in unvisited if x != d]
                    if rest:
                        nxt_best = float("inf")
                        for nx in rest:
                            for nx_rev in (False, True):
                                nx_ent = _route_endpoint(routes[nx], reverse=nx_rev, which="entry")
                                c = _travel_cost(ex, nx_ent)
                                if c < nxt_best:
                                    nxt_best = c
                        look = float(nxt_best if np.isfinite(nxt_best) else 0.0)
                score = w_dist * travel + w_rank * rank_cost + w_viol * float(violation) + region_pen + w_look * look
                key = (float(score), float(rank_cost), float(travel))
                if key < best_key:
                    best_key = key
                    best = (d, rev, float(travel), ex.copy())
        if best is None:
            break
        d_pick, rev_pick, travel_pick, ex_pick = best
        order.append(d_pick)
        states.append(bool(rev_pick))
        total_cost += float(travel_pick)
        prev_region = routes[d_pick].region
        current = ex_pick
        unvisited.remove(d_pick)

    if return_home and order:
        total_cost += _travel_cost(current, start_pt)
    return order, states, float(total_cost), 1


def _new_home_capture(xyz: np.ndarray, vp_id: int) -> CapturePoint:
    return CapturePoint(
        viewpoint_id=vp_id,
        device="home",
        device_type="home",
        x=float(xyz[0]),
        y=float(xyz[1]),
        z=float(xyz[2]),
        yaw_deg=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        target_x=float(xyz[0]),
        target_y=float(xyz[1]),
        target_z=float(xyz[2]),
        layer_k=-1,
        component_id=-1,
        source_row_index=-1,
    )


def build_mission_waypoints(
    order: Sequence[str],
    reverse_flags: Sequence[bool],
    routes: Dict[str, CompressedDeviceRoute],
    start_pt: np.ndarray,
    return_home: bool,
) -> Tuple[List[CapturePoint], List[dict]]:
    mission_caps: List[CapturePoint] = []
    boundaries: List[dict] = []

    mission_caps.append(_new_home_capture(start_pt, vp_id=-1))
    wp_cursor = 1
    for seq_idx, dev in enumerate(order):
        route = routes[dev]
        rev = bool(reverse_flags[seq_idx])
        caps_dev, _ = route.ordered_by_state(reverse=rev)
        start_wp = wp_cursor
        for cp in caps_dev:
            mission_caps.append(cp)
            wp_cursor += 1
        end_wp = wp_cursor - 1
        boundaries.append(
            {
                "device": dev,
                "device_type": route.device_type,
                "region": route.region,
                "order_index": seq_idx,
                "reverse": rev,
                "start_wp_id": start_wp,
                "end_wp_id": end_wp,
                "raw_viewpoints": route.raw_count,
                "retained_viewpoints": route.retained_count,
                "retention_ratio": route.retention_ratio,
                "estimated_coverage_ratio": route.estimated_coverage_ratio,
            }
        )

    if return_home:
        mission_caps.append(_new_home_capture(start_pt, vp_id=-2))
    return mission_caps, boundaries


def build_mission_nav_points(caps: Sequence[CapturePoint]) -> np.ndarray:
    if not caps:
        return np.empty((0, 3), dtype=np.float64)
    pts = np.array([[cp.x, cp.y, cp.z] for cp in caps], dtype=np.float64)
    return pts


def plan_segment(a: np.ndarray, b: np.ndarray, station: StationGrid, cfg: dict) -> Tuple[str, bool, np.ndarray, float]:
    line_ok = station.line_free_world(a, b, step_scale=0.8)
    if line_ok:
        seg = np.vstack([a, b])
        return "straight", True, seg, float(np.linalg.norm(b - a))
    astar = station.astar_world(
        start_w=a,
        goal_w=b,
        margin_m=float(cfg.get("astar_roi_margin_m", 9.0)),
        connectivity=int(cfg.get("voxel_connectivity", 26)),
        max_expansions=int(cfg.get("astar_max_expansions", 180000)),
    )
    if astar is not None and astar.shape[0] >= 2:
        length = float(np.linalg.norm(astar[1:] - astar[:-1], axis=1).sum())
        return "astar", True, astar, length
    seg = np.vstack([a, b])
    return "fallback_direct", False, seg, float(np.linalg.norm(b - a))


def plan_full_mission_path(nav_pts: np.ndarray, station: StationGrid, cfg: dict) -> Tuple[np.ndarray, List[dict], List[int], float, float]:
    if nav_pts.shape[0] <= 1:
        return nav_pts.copy(), [], [0], 0.0, 1.0

    out_pts: List[np.ndarray] = [nav_pts[0].copy()]
    segments: List[dict] = []
    wp_to_path_id: List[int] = [0]
    ok_count = 0
    for i in range(nav_pts.shape[0] - 1):
        a = nav_pts[i]
        b = nav_pts[i + 1]
        mode, ok, seg_pts, seg_len = plan_segment(a, b, station, cfg)
        if mode == "astar":
            for p in seg_pts[1:]:
                out_pts.append(p.astype(np.float64))
        else:
            out_pts.append(b.copy())
        if ok:
            ok_count += 1
        wp_to_path_id.append(len(out_pts) - 1)
        rec = {
            "segment_id": i,
            "start_wp_id": i,
            "end_wp_id": i + 1,
            "mode": mode,
            "ok": bool(ok),
            "length_m": float(seg_len),
            "path_node_count": int(seg_pts.shape[0]),
        }
        segments.append(rec)

    path = np.vstack(out_pts) if out_pts else np.empty((0, 3), dtype=np.float64)
    total_len = float(np.linalg.norm(path[1:] - path[:-1], axis=1).sum()) if path.shape[0] > 1 else 0.0
    success_ratio = float(ok_count / max(1, len(segments)))
    return path, segments, wp_to_path_id, total_len, success_ratio


def _remove_middle_if_smooth(prev_p: np.ndarray, cur_p: np.ndarray, next_p: np.ndarray, station: StationGrid, cfg: dict) -> bool:
    if not station.line_free_world(prev_p, next_p):
        return False
    max_skip = float(cfg.get("max_skip_distance_m", 8.0))
    if float(np.linalg.norm(next_p - prev_p)) > max_skip:
        return False
    ang = float(np.degrees(_angle_rad(cur_p - prev_p, next_p - cur_p)))
    if ang > float(cfg.get("angle_threshold_deg", 28.0)):
        return False
    d1 = float(np.linalg.norm(cur_p - prev_p))
    d2 = float(np.linalg.norm(next_p - cur_p))
    d3 = float(np.linalg.norm(next_p - prev_p))
    extra_ratio = (d1 + d2 - d3) / max(d1 + d2, 1e-6)
    if extra_ratio > float(cfg.get("max_extra_path_ratio", 0.18)):
        return False
    return True


def refine_route_local(
    captures: List[CapturePoint],
    nav: np.ndarray,
    station: StationGrid,
    cfg: dict,
) -> Tuple[List[CapturePoint], np.ndarray]:
    if nav.shape[0] <= 3:
        return captures, nav
    min_keep = int(max(3, cfg.get("min_keep_per_device", 8)))
    iters = int(max(1, cfg.get("iterations", 2)))
    caps_cur = list(captures)
    nav_cur = nav.copy()
    for _ in range(iters):
        if nav_cur.shape[0] <= min_keep:
            break
        keep_idx = [0]
        for i in range(1, nav_cur.shape[0] - 1):
            prev_p = nav_cur[keep_idx[-1]]
            cur_p = nav_cur[i]
            next_p = nav_cur[i + 1]
            can_remove = _remove_middle_if_smooth(prev_p, cur_p, next_p, station, cfg)
            if can_remove and (len(keep_idx) + (nav_cur.shape[0] - i - 1)) >= min_keep:
                continue
            keep_idx.append(i)
        keep_idx.append(nav_cur.shape[0] - 1)
        keep_idx = sorted(set(keep_idx))
        if len(keep_idx) == nav_cur.shape[0]:
            break
        nav_cur = nav_cur[keep_idx]
        caps_cur = [caps_cur[i] for i in keep_idx]
    return caps_cur, nav_cur


def _nn_order_for_points(arr: np.ndarray, start_pt: np.ndarray) -> List[int]:
    n = int(arr.shape[0])
    if n <= 1:
        return list(range(n))
    rem = set(range(n))
    d0 = np.linalg.norm(arr - start_pt[None, :], axis=1)
    cur = int(np.argmin(d0))
    order = [cur]
    rem.remove(cur)
    while rem:
        rem_list = np.array(sorted(rem), dtype=np.int32)
        d = np.linalg.norm(arr[rem_list] - arr[cur][None, :], axis=1)
        nxt = int(rem_list[int(np.argmin(d))])
        order.append(nxt)
        rem.remove(nxt)
        cur = nxt
    return order


def reorder_route_phase2_style(
    captures: List[CapturePoint],
    nav: np.ndarray,
    cfg: dict,
) -> Tuple[List[CapturePoint], np.ndarray]:
    if nav.shape[0] <= 2:
        return captures, nav
    by_layer: Dict[int, List[int]] = {}
    for i, cp in enumerate(captures):
        by_layer.setdefault(int(cp.layer_k), []).append(i)
    layers = sorted(by_layer.keys())
    if not bool(cfg.get("low_to_high", True)):
        layers = list(reversed(layers))
    ordered_global: List[int] = []
    anchor = nav[0].copy()
    for lk in layers:
        idxs = by_layer[lk]
        if len(idxs) == 1:
            ordered_global.append(idxs[0])
            anchor = nav[idxs[0]]
            continue
        sub = nav[idxs]
        local_ord = _nn_order_for_points(sub, start_pt=anchor)
        for j in local_ord:
            gi = idxs[j]
            ordered_global.append(gi)
        anchor = nav[ordered_global[-1]]
    ordered_global = list(dict.fromkeys(ordered_global))
    if len(ordered_global) != nav.shape[0]:
        missing = [i for i in range(nav.shape[0]) if i not in set(ordered_global)]
        ordered_global.extend(missing)
    nav_new = nav[ordered_global]
    caps_new = [captures[i] for i in ordered_global]
    return caps_new, nav_new


def apply_phase2_style_local_order(
    routes: Dict[str, CompressedDeviceRoute],
    cfg: dict,
) -> Dict[str, CompressedDeviceRoute]:
    pcfg = cfg.get("phase2_style_local_order", {})
    if not bool(pcfg.get("enable", False)):
        return routes
    target_regions = {str(x) for x in pcfg.get("target_regions", [])}
    target_devices = {str(x) for x in pcfg.get("target_devices", [])}
    out: Dict[str, CompressedDeviceRoute] = {}
    for dev, rt in routes.items():
        hit = (rt.region in target_regions) or (dev in target_devices)
        if not hit:
            out[dev] = rt
            continue
        caps_new, nav_new = reorder_route_phase2_style(rt.captures, rt.nav_points, cfg=pcfg)
        out[dev] = CompressedDeviceRoute(
            name=rt.name,
            device_type=rt.device_type,
            captures=caps_new,
            nav_points=nav_new,
            kept_indices=list(rt.kept_indices),
            raw_count=rt.raw_count,
            retained_count=rt.retained_count,
            retention_ratio=rt.retention_ratio,
            estimated_coverage_ratio=rt.estimated_coverage_ratio,
            region=rt.region,
        )
    return out


def apply_local_refinement(
    routes: Dict[str, CompressedDeviceRoute],
    station: StationGrid,
    cfg: dict,
) -> Dict[str, CompressedDeviceRoute]:
    lcfg = cfg.get("local_refinement", {})
    if not bool(lcfg.get("enable", False)):
        return routes
    target_regions = {str(x) for x in lcfg.get("target_regions", [])}
    target_devices = {str(x) for x in lcfg.get("target_devices", [])}
    out: Dict[str, CompressedDeviceRoute] = {}
    for dev, rt in routes.items():
        hit = (rt.region in target_regions) or (dev in target_devices)
        if not hit:
            out[dev] = rt
            continue
        caps_new, nav_new = refine_route_local(rt.captures, rt.nav_points, station, lcfg)
        kept = [int(c.source_row_index) for c in caps_new]
        retained = len(caps_new)
        out[dev] = CompressedDeviceRoute(
            name=rt.name,
            device_type=rt.device_type,
            captures=caps_new,
            nav_points=nav_new,
            kept_indices=kept,
            raw_count=rt.raw_count,
            retained_count=retained,
            retention_ratio=float(retained / max(1, rt.raw_count)),
            estimated_coverage_ratio=float(np.clip(rt.estimated_coverage_ratio * (retained / max(1, rt.retained_count)), 0.0, 1.0)),
            region=rt.region,
        )
    return out


def shortcut_path_line_of_sight(path_pts: np.ndarray, station: StationGrid, cfg: dict) -> np.ndarray:
    if path_pts.shape[0] <= 2 or (not bool(cfg.get("enable", True))):
        return path_pts.copy()
    max_hop = int(max(2, cfg.get("max_hop", 200)))
    min_seg = float(max(0.05, cfg.get("min_segment_length_m", 0.2)))
    out: List[np.ndarray] = [path_pts[0]]
    i = 0
    n = path_pts.shape[0]
    while i < n - 1:
        j = min(n - 1, i + max_hop)
        best = i + 1
        while j > i + 1:
            if station.line_free_world(path_pts[i], path_pts[j]):
                if float(np.linalg.norm(path_pts[j] - path_pts[i])) >= min_seg:
                    best = j
                    break
            j -= 1
        out.append(path_pts[best])
        i = best
    return np.vstack(out)


def map_waypoints_to_path_indices(path_pts: np.ndarray, wp_pts: np.ndarray) -> List[int]:
    if path_pts.shape[0] == 0 or wp_pts.shape[0] == 0:
        return []
    out: List[int] = []
    start = 0
    for wp in wp_pts:
        sub = path_pts[start:]
        if sub.shape[0] == 0:
            out.append(int(path_pts.shape[0] - 1))
            continue
        d2 = np.sum((sub - wp[None, :]) ** 2, axis=1)
        local = int(np.argmin(d2))
        idx = int(start + local)
        out.append(idx)
        start = idx
    return out


def moving_average_path(path_pts: np.ndarray, window_size: int) -> np.ndarray:
    if path_pts.shape[0] <= 2 or window_size <= 1:
        return path_pts.copy()
    w = int(max(1, window_size))
    if w % 2 == 0:
        w += 1
    pad = w // 2
    kernel = np.ones((w,), dtype=np.float64) / float(w)
    out = path_pts.copy()
    for c in range(3):
        arr = np.pad(path_pts[:, c], (pad, pad), mode="edge")
        out[:, c] = np.convolve(arr, kernel, mode="valid")
    out[0] = path_pts[0]
    out[-1] = path_pts[-1]
    return out


def smooth_path_safe(
    path_pts: np.ndarray,
    station: StationGrid,
    cfg: dict,
    locked_indices: Set[int],
) -> np.ndarray:
    if path_pts.shape[0] <= 2 or (not bool(cfg.get("enable", True))):
        return path_pts.copy()
    window = int(cfg.get("window_size", 5))
    iters = int(max(1, cfg.get("iterations", 1)))
    max_dev = float(max(0.01, cfg.get("max_deviation_m", 0.6)))

    out = path_pts.copy()
    n = out.shape[0]
    locked = set(int(i) for i in locked_indices if 0 <= int(i) < n)
    locked.add(0)
    locked.add(n - 1)

    for _ in range(iters):
        cand = moving_average_path(out, window_size=window)
        for i in range(1, n - 1):
            if i in locked:
                continue
            p0 = out[i]
            p1 = cand[i]
            d = float(np.linalg.norm(p1 - p0))
            if d > max_dev:
                p1 = p0 + (p1 - p0) * (max_dev / max(d, 1e-9))
            if not station.is_free_world(p1):
                continue
            if (not station.line_free_world(out[i - 1], p1)) or (not station.line_free_world(p1, out[i + 1])):
                continue
            out[i] = p1
    return out


def estimate_point_clearance_m(station: StationGrid, p: np.ndarray, max_k: int) -> float:
    idx = station.world_to_idx(p)
    if not station.in_bounds_idx(idx):
        return 0.0
    ix, iy, iz = int(idx[0]), int(idx[1]), int(idx[2])
    if station.blocked[ix, iy, iz]:
        return 0.0
    mx = int(max(1, max_k))
    for k in range(1, mx + 1):
        x0, x1 = max(0, ix - k), min(int(station.shape[0]) - 1, ix + k)
        y0, y1 = max(0, iy - k), min(int(station.shape[1]) - 1, iy + k)
        z0, z1 = max(0, iz - k), min(int(station.shape[2]) - 1, iz + k)
        if np.any(station.blocked[x0 : x1 + 1, y0 : y1 + 1, z0 : z1 + 1]):
            return float(max(0.0, (k - 0.5) * station.voxel))
    return float((mx + 0.5) * station.voxel)


def compute_trackability_metrics(path_pts: np.ndarray, station: StationGrid, cfg: dict) -> dict:
    if path_pts.shape[0] <= 1:
        return {
            "path_points": int(path_pts.shape[0]),
            "total_length_m": 0.0,
            "max_turn_angle_deg": 0.0,
            "mean_turn_angle_deg": 0.0,
            "p95_turn_angle_deg": 0.0,
            "max_curvature_proxy": 0.0,
            "mean_curvature_proxy": 0.0,
            "min_turn_radius_proxy_m": None,
            "turning_points_count": 0,
            "turning_points_per_100m": 0.0,
            "jerk_rms_proxy": 0.0,
            "jerk_energy_proxy": 0.0,
            "min_clearance_m": 0.0,
            "mean_clearance_m": 0.0,
            "low_clearance_ratio": 0.0,
            "min_corridor_width_proxy_m": 0.0,
            "trackability_score_0_100": 0.0,
        }

    seg = path_pts[1:] - path_pts[:-1]
    seg_len = np.linalg.norm(seg, axis=1)
    total_len = float(seg_len.sum()) if seg_len.size else 0.0
    angles: List[float] = []
    curv: List[float] = []
    for i in range(seg.shape[0] - 1):
        a = seg[i]
        b = seg[i + 1]
        la = float(seg_len[i])
        lb = float(seg_len[i + 1])
        if la < 1e-8 or lb < 1e-8:
            continue
        ang = _angle_rad(a, b)
        angles.append(ang)
        curv.append(ang / max((la + lb) * 0.5, 1e-6))

    ang_arr = np.array(angles, dtype=np.float64) if angles else np.empty((0,), dtype=np.float64)
    curv_arr = np.array(curv, dtype=np.float64) if curv else np.empty((0,), dtype=np.float64)
    th_deg = float(cfg.get("turning_angle_threshold_deg", 22.0))
    turning_cnt = int(np.sum(np.degrees(ang_arr) >= th_deg)) if ang_arr.size else 0
    turning_density = float(turning_cnt / max(total_len, 1e-6) * 100.0)

    jerk_rms = 0.0
    jerk_energy = 0.0
    if path_pts.shape[0] >= 4:
        vel = np.diff(path_pts, axis=0)
        acc = np.diff(vel, axis=0)
        jerk = np.diff(acc, axis=0)
        if jerk.shape[0] > 0:
            jn = np.linalg.norm(jerk, axis=1)
            jerk_rms = float(np.sqrt(np.mean(jn ** 2)))
            jerk_energy = float(np.sum(jn ** 2))

    max_k = int(max(1, cfg.get("clearance_max_search_vox", 8)))
    stride = int(max(1, cfg.get("clearance_sample_stride", 4)))
    sampled = path_pts[::stride]
    if path_pts.shape[0] >= 1 and (sampled.shape[0] == 0 or np.any(sampled[-1] != path_pts[-1])):
        sampled = np.vstack([sampled, path_pts[-1]])
    clearances = np.array([estimate_point_clearance_m(station, p, max_k=max_k) for p in sampled], dtype=np.float64)
    low_thr = float(cfg.get("clearance_threshold_m", 1.2))
    low_ratio = float(np.mean(clearances < low_thr)) if clearances.size else 0.0
    min_cl = float(clearances.min()) if clearances.size else 0.0
    mean_cl = float(clearances.mean()) if clearances.size else 0.0

    max_ang_deg = float(np.degrees(ang_arr.max())) if ang_arr.size else 0.0
    mean_ang_deg = float(np.degrees(ang_arr.mean())) if ang_arr.size else 0.0
    p95_ang_deg = float(np.degrees(np.quantile(ang_arr, 0.95))) if ang_arr.size else 0.0
    max_curv = float(curv_arr.max()) if curv_arr.size else 0.0
    mean_curv = float(curv_arr.mean()) if curv_arr.size else 0.0
    min_radius = float(1.0 / max_curv) if max_curv > 1e-9 else None
    min_corridor_width = float(2.0 * min_cl)

    angle_score = float(np.clip(1.0 - max_ang_deg / 150.0, 0.0, 1.0))
    radius_score = float(np.clip((min_radius if min_radius is not None else 5.0) / 3.0, 0.0, 1.0))
    jerk_score = float(np.clip(1.0 / (1.0 + jerk_rms * 3.0), 0.0, 1.0))
    clearance_score = float(np.clip(mean_cl / 2.5, 0.0, 1.0))
    turn_density_score = float(np.clip(1.0 - turning_density / 30.0, 0.0, 1.0))
    total_score = float(
        (0.30 * angle_score + 0.20 * radius_score + 0.20 * jerk_score + 0.20 * clearance_score + 0.10 * turn_density_score) * 100.0
    )

    return {
        "path_points": int(path_pts.shape[0]),
        "total_length_m": total_len,
        "max_turn_angle_deg": max_ang_deg,
        "mean_turn_angle_deg": mean_ang_deg,
        "p95_turn_angle_deg": p95_ang_deg,
        "max_curvature_proxy": max_curv,
        "mean_curvature_proxy": mean_curv,
        "min_turn_radius_proxy_m": min_radius,
        "turning_points_count": turning_cnt,
        "turning_points_per_100m": turning_density,
        "jerk_rms_proxy": jerk_rms,
        "jerk_energy_proxy": jerk_energy,
        "min_clearance_m": min_cl,
        "mean_clearance_m": mean_cl,
        "low_clearance_ratio": low_ratio,
        "min_corridor_width_proxy_m": min_corridor_width,
        "trackability_score_0_100": total_score,
    }


def analyze_order_match(
    order: Sequence[str],
    region_order: Sequence[str],
    device_to_region: Dict[str, str],
    strict_target: Sequence[str],
) -> dict:
    real_regions: List[str] = []
    for d in order:
        r = device_to_region.get(d, "unknown")
        if not real_regions or r != real_regions[-1]:
            real_regions.append(r)
    strict_match = list(order) == list(strict_target)
    region_match = list(real_regions) == list(region_order)
    return {
        "actual_device_order": list(order),
        "actual_region_order_collapsed": real_regions,
        "expected_region_order": list(region_order),
        "expected_device_order_strict": list(strict_target),
        "region_order_match": bool(region_match),
        "strict_device_order_match": bool(strict_match),
    }


def write_capture_mission_csv(path: Path, caps: Sequence[CapturePoint], order_meta: Dict[str, dict]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "mission_waypoint_id",
                "role",
                "device",
                "device_type",
                "region",
                "device_order_index",
                "x",
                "y",
                "z",
                "yaw_deg",
                "pitch_deg",
                "roll_deg",
                "target_x",
                "target_y",
                "target_z",
                "source_phase2_viewpoint_id",
                "source_phase2_row_index",
            ]
        )
        for i, cp in enumerate(caps):
            role = "inspect_capture"
            if i == 0:
                role = "home_start"
            elif cp.device == "home":
                role = "home_end"
            meta = order_meta.get(cp.device, {})
            w.writerow(
                [
                    i,
                    role,
                    cp.device,
                    cp.device_type,
                    meta.get("region", "home" if cp.device == "home" else ""),
                    meta.get("order_index", -1),
                    f"{cp.x:.4f}",
                    f"{cp.y:.4f}",
                    f"{cp.z:.4f}",
                    f"{cp.yaw_deg:.2f}",
                    f"{cp.pitch_deg:.2f}",
                    f"{cp.roll_deg:.2f}",
                    f"{cp.target_x:.4f}",
                    f"{cp.target_y:.4f}",
                    f"{cp.target_z:.4f}",
                    cp.viewpoint_id,
                    cp.source_row_index,
                ]
            )


def write_nav_csv(path: Path, nav_pts: np.ndarray) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["waypoint_id", "x", "y", "z"])
        for i, p in enumerate(nav_pts.tolist()):
            w.writerow([i, f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}"])


def write_path_csv(path: Path, path_pts: np.ndarray) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["path_id", "x", "y", "z"])
        for i, p in enumerate(path_pts.tolist()):
            w.writerow([i, f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}"])


def write_segments_csv(path: Path, segments: Sequence[dict]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        fieldnames = ["segment_id", "start_wp_id", "end_wp_id", "mode", "ok", "length_m", "path_node_count"]
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for s in segments:
            w.writerow({k: s.get(k, "") for k in fieldnames})


def write_device_summary_csv(path: Path, rows: Sequence[dict]) -> None:
    if not rows:
        return
    fieldnames = [
        "device",
        "device_type",
        "region",
        "order_index",
        "reverse",
        "raw_viewpoints",
        "retained_viewpoints",
        "retention_ratio",
        "phase2_coverage_ratio",
        "estimated_phase3_coverage_ratio",
        "phase2_path_length_m",
        "phase2_path_segment_success_ratio",
        "entry_transition_length_m",
        "intra_device_length_m",
        "exit_transition_length_m",
    ]
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in fieldnames})


def save_experiment_csv(path: Path, rows: Sequence[dict]) -> None:
    if not rows:
        return
    keys = [
        "scenario",
        "ordering_strategy",
        "retained_viewpoints_total",
        "global_retention_ratio",
        "estimated_global_coverage_ratio",
        "path_length_raw_m",
        "path_length_shortcut_m",
        "path_length_smoothed_m",
        "segment_success_ratio",
        "trackability_raw",
        "trackability_shortcut",
        "trackability_smoothed",
        "max_turn_raw_deg",
        "max_turn_shortcut_deg",
        "max_turn_smoothed_deg",
        "low_clearance_raw_ratio",
        "low_clearance_shortcut_ratio",
        "low_clearance_smoothed_ratio",
        "region_order_match",
        "strict_device_order_match",
        "estimated_order_cost_m",
        "scenario_total_s",
    ]
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})


def visualize_station_mission(
    out_png: Path,
    out_gif: Path,
    routes: Dict[str, CompressedDeviceRoute],
    order: Sequence[str],
    start_pt: np.ndarray,
    path_raw: np.ndarray,
    path_smooth: np.ndarray,
    vis_cfg: dict,
) -> None:
    colors = plt.get_cmap("tab10", max(10, len(order)))
    dev_color: Dict[str, Tuple[float, float, float, float]] = {}
    for i, d in enumerate(order):
        dev_color[d] = colors(i % 10)

    fig = plt.figure(figsize=(15, 7))
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")

    for i, d in enumerate(order):
        pts = routes[d].nav_points
        c = dev_color[d]
        ax1.scatter(pts[:, 0], pts[:, 1], s=10, c=[c], alpha=0.35)
        ax2.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=5, c=[c], alpha=0.25, depthshade=False)
        cen = np.mean(pts, axis=0)
        ax1.text(float(cen[0]), float(cen[1]), f"{i+1}:{d}", fontsize=8)

    if path_raw.shape[0] > 1:
        ax1.plot(path_raw[:, 0], path_raw[:, 1], c="gray", linewidth=1.2, alpha=0.5, label="raw path")
        ax2.plot(path_raw[:, 0], path_raw[:, 1], path_raw[:, 2], c="gray", linewidth=1.0, alpha=0.35)
    if path_smooth.shape[0] > 1:
        ax1.plot(path_smooth[:, 0], path_smooth[:, 1], c="deepskyblue", linewidth=1.8, alpha=0.95, label="smoothed path")
        ax2.plot(path_smooth[:, 0], path_smooth[:, 1], path_smooth[:, 2], c="deepskyblue", linewidth=1.6, alpha=0.95)

    ax1.scatter([start_pt[0]], [start_pt[1]], marker="*", s=130, c="gold", edgecolors="k", linewidths=0.6, label="airport")
    ax2.scatter([start_pt[0]], [start_pt[1]], [start_pt[2]], marker="*", s=110, c="gold", edgecolors="k", linewidths=0.6, depthshade=False)
    ax1.set_title("Phase3 Station Mission (Top View)")
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.legend(loc="best")
    ax1.grid(True, alpha=0.25)

    ax2.set_title("Phase3 Station Mission (3D)")
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_zlabel("z")
    ax2.view_init(elev=float(vis_cfg.get("elev_deg", 24.0)), azim=float(vis_cfg.get("azim_deg", -35.0)))
    fig.tight_layout()
    fig.savefig(out_png, dpi=int(vis_cfg.get("dpi", 220)))
    plt.close(fig)

    if not bool(vis_cfg.get("gif_enable", True)):
        return
    try:
        gif_frames = int(vis_cfg.get("gif_frames", 48))
        gif_fps = int(vis_cfg.get("gif_fps", 10))
        fig2 = plt.figure(figsize=(8, 7))
        ax = fig2.add_subplot(1, 1, 1, projection="3d")
        for i, d in enumerate(order):
            pts = routes[d].nav_points
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=4, c=[dev_color[d]], alpha=0.22, depthshade=False)
        if path_smooth.shape[0] > 1:
            ax.plot(path_smooth[:, 0], path_smooth[:, 1], path_smooth[:, 2], c="deepskyblue", linewidth=1.6, alpha=0.96)
        ax.scatter([start_pt[0]], [start_pt[1]], [start_pt[2]], marker="*", s=100, c="gold", edgecolors="k", linewidths=0.6, depthshade=False)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title("Phase3 Station Mission Rotate")
        ax.view_init(elev=float(vis_cfg.get("elev_deg", 24.0)), azim=float(vis_cfg.get("azim_deg", -35.0)))
        fig2.tight_layout()

        def _anim(frame_idx: int):
            az = float(vis_cfg.get("azim_deg", -35.0)) + (360.0 * frame_idx / max(1, gif_frames))
            ax.view_init(elev=float(vis_cfg.get("elev_deg", 24.0)), azim=az)
            return ()

        ani = animation.FuncAnimation(fig2, _anim, frames=gif_frames, interval=100)
        ani.save(out_gif, writer="pillow", fps=gif_fps)
        plt.close(fig2)
    except Exception as e:
        log(f"GIF generation skipped: {e}")


def visualize_station_rgb_path(
    out_png: Path,
    out_gif: Path,
    cloud_xyz: np.ndarray,
    cloud_rgb: Optional[np.ndarray],
    path_smooth: np.ndarray,
    start_pt: np.ndarray,
    vis_cfg: dict,
) -> None:
    airport_lift = float(vis_cfg.get("airport_lift_m", 2.2))
    start_vis = start_pt.copy()
    start_vis[2] = start_vis[2] + airport_lift
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    if cloud_xyz.shape[0] > 0:
        if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
            ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c=cloud_rgb, s=0.45, alpha=0.24, depthshade=False)
        else:
            ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c="silver", s=0.45, alpha=0.24, depthshade=False)
    line_handle = None
    if path_smooth.shape[0] > 1:
        (line_handle,) = ax.plot(path_smooth[:, 0], path_smooth[:, 1], path_smooth[:, 2], c="deepskyblue", linewidth=1.7, alpha=0.98, label="smoothed path")
    airport = ax.scatter([start_vis[0]], [start_vis[1]], [start_vis[2]], marker="*", s=220, c="gold", edgecolors="k", linewidths=0.9, depthshade=False, label="airport")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title("Station RGB Cloud + Mission Path")
    ax.view_init(elev=float(vis_cfg.get("elev_deg", 24.0)), azim=float(vis_cfg.get("azim_deg", -35.0)))
    handles = [h for h in [line_handle, airport] if h is not None]
    labels = ["smoothed path", "airport"][: len(handles)]
    if handles:
        ax.legend(handles, labels, loc="upper right")
    fig.tight_layout()
    fig.savefig(out_png, dpi=int(vis_cfg.get("dpi", 220)))
    plt.close(fig)

    if not bool(vis_cfg.get("gif_enable", True)):
        return
    fig2 = plt.figure(figsize=(9, 7))
    ax2 = fig2.add_subplot(1, 1, 1, projection="3d")
    if cloud_xyz.shape[0] > 0:
        if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
            ax2.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c=cloud_rgb, s=0.42, alpha=0.22, depthshade=False)
        else:
            ax2.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c="silver", s=0.42, alpha=0.22, depthshade=False)
    if path_smooth.shape[0] > 1:
        ax2.plot(path_smooth[:, 0], path_smooth[:, 1], path_smooth[:, 2], c="deepskyblue", linewidth=1.7, alpha=0.98, label="smoothed path")
    ax2.scatter([start_vis[0]], [start_vis[1]], [start_vis[2]], marker="*", s=220, c="gold", edgecolors="k", linewidths=0.9, depthshade=False, label="airport")
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_zlabel("z")
    ax2.set_title("Station RGB Cloud + Mission Path Rotate")
    ax2.view_init(elev=float(vis_cfg.get("elev_deg", 24.0)), azim=float(vis_cfg.get("azim_deg", -35.0)))
    ax2.legend(loc="upper right")
    fig2.tight_layout()
    gf = int(vis_cfg.get("gif_frames", 48))
    gfps = int(vis_cfg.get("gif_fps", 10))

    def _anim(i: int):
        az = float(vis_cfg.get("azim_deg", -35.0)) + (360.0 * i / max(1, gf))
        ax2.view_init(elev=float(vis_cfg.get("elev_deg", 24.0)), azim=az)
        return ()

    ani = animation.FuncAnimation(fig2, _anim, frames=gf, interval=100)
    ani.save(out_gif, writer="pillow", fps=gfps)
    plt.close(fig2)


def _plot_rgb_panel(
    ax: plt.Axes,
    cloud_xyz: np.ndarray,
    cloud_rgb: Optional[np.ndarray],
    path_xyz: np.ndarray,
    start_vis: np.ndarray,
    title: str,
    path_label: str,
    path_color: str,
    elev_deg: float,
    azim_deg: float,
) -> None:
    if cloud_xyz.shape[0] > 0:
        if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
            ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c=cloud_rgb, s=0.42, alpha=0.22, depthshade=False)
        else:
            ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c="silver", s=0.42, alpha=0.22, depthshade=False)
    if path_xyz.shape[0] > 1:
        ax.plot(path_xyz[:, 0], path_xyz[:, 1], path_xyz[:, 2], c=path_color, linewidth=1.8, alpha=0.98, label=path_label)
    ax.scatter([start_vis[0]], [start_vis[1]], [start_vis[2]], marker="*", s=210, c="gold", edgecolors="k", linewidths=0.9, depthshade=False, label="airport")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title(title)
    ax.view_init(elev=elev_deg, azim=azim_deg)
    ax.legend(loc="upper right")


def visualize_station_rgb_path_comparison(
    out_png: Path,
    out_gif: Path,
    cloud_xyz: np.ndarray,
    cloud_rgb: Optional[np.ndarray],
    path_left: np.ndarray,
    path_right: np.ndarray,
    start_pt: np.ndarray,
    vis_cfg: dict,
    left_title: str,
    right_title: str,
) -> None:
    airport_lift = float(vis_cfg.get("airport_lift_m", 2.2))
    elev_deg = float(vis_cfg.get("elev_deg", 24.0))
    azim_deg = float(vis_cfg.get("azim_deg", -35.0))
    start_vis = start_pt.copy()
    start_vis[2] = start_vis[2] + airport_lift

    fig = plt.figure(figsize=(16, 7))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    _plot_rgb_panel(
        ax=ax1,
        cloud_xyz=cloud_xyz,
        cloud_rgb=cloud_rgb,
        path_xyz=path_left,
        start_vis=start_vis,
        title=left_title,
        path_label="baseline path",
        path_color="deepskyblue",
        elev_deg=elev_deg,
        azim_deg=azim_deg,
    )
    _plot_rgb_panel(
        ax=ax2,
        cloud_xyz=cloud_xyz,
        cloud_rgb=cloud_rgb,
        path_xyz=path_right,
        start_vis=start_vis,
        title=right_title,
        path_label="phase2_style_local path",
        path_color="#fb8500",
        elev_deg=elev_deg,
        azim_deg=azim_deg,
    )
    fig.tight_layout()
    fig.savefig(out_png, dpi=int(vis_cfg.get("dpi", 220)))
    plt.close(fig)

    if not bool(vis_cfg.get("gif_enable", True)):
        return
    fig2 = plt.figure(figsize=(16, 7))
    bx1 = fig2.add_subplot(1, 2, 1, projection="3d")
    bx2 = fig2.add_subplot(1, 2, 2, projection="3d")
    _plot_rgb_panel(
        ax=bx1,
        cloud_xyz=cloud_xyz,
        cloud_rgb=cloud_rgb,
        path_xyz=path_left,
        start_vis=start_vis,
        title=left_title,
        path_label="baseline path",
        path_color="deepskyblue",
        elev_deg=elev_deg,
        azim_deg=azim_deg,
    )
    _plot_rgb_panel(
        ax=bx2,
        cloud_xyz=cloud_xyz,
        cloud_rgb=cloud_rgb,
        path_xyz=path_right,
        start_vis=start_vis,
        title=right_title,
        path_label="phase2_style_local path",
        path_color="#fb8500",
        elev_deg=elev_deg,
        azim_deg=azim_deg,
    )
    fig2.tight_layout()
    gf = int(vis_cfg.get("gif_frames", 48))
    gfps = int(vis_cfg.get("gif_fps", 10))

    def _anim_pair(i: int):
        az = azim_deg + (360.0 * i / max(1, gf))
        bx1.view_init(elev=elev_deg, azim=az)
        bx2.view_init(elev=elev_deg, azim=az)
        return ()

    ani = animation.FuncAnimation(fig2, _anim_pair, frames=gf, interval=100)
    ani.save(out_gif, writer="pillow", fps=gfps)
    plt.close(fig2)


def visualize_local_region_details(
    out_png: Path,
    target_regions: Sequence[str],
    boundaries: Sequence[dict],
    wp_to_path_idx_raw: Sequence[int],
    wp_to_path_idx_short: Sequence[int],
    wp_to_path_idx_smooth: Sequence[int],
    mission_wp: np.ndarray,
    path_raw: np.ndarray,
    path_short: np.ndarray,
    path_smooth: np.ndarray,
    cloud_xyz: np.ndarray,
    cloud_rgb: Optional[np.ndarray],
    margin_m: float = 8.0,
    selection_mode: str = "balanced",
) -> None:
    if not target_regions:
        return
    region_to_wp: Dict[str, Tuple[int, int]] = {}
    for r in target_regions:
        starts = [int(b["start_wp_id"]) for b in boundaries if str(b.get("region")) == str(r)]
        ends = [int(b["end_wp_id"]) for b in boundaries if str(b.get("region")) == str(r)]
        if starts and ends:
            region_to_wp[str(r)] = (min(starts), max(ends))
    if not region_to_wp:
        return
    mode = str(selection_mode).strip().lower()

    def _turn_angle_deg(path_xyz: np.ndarray, idx: int) -> float:
        n = int(path_xyz.shape[0])
        if n < 3:
            return 0.0
        i = int(np.clip(idx, 1, n - 2))
        u = path_xyz[i] - path_xyz[i - 1]
        v = path_xyz[i + 1] - path_xyz[i]
        if float(np.linalg.norm(u)) < 1e-9 or float(np.linalg.norm(v)) < 1e-9:
            return 0.0
        return float(np.degrees(_angle_rad(u, v)))

    best_region = None
    best_wp = -1
    best_span = 8.0
    best_score_smooth = -1.0
    best_score_fallback = -1.0
    for region, (sw, ew) in region_to_wp.items():
        sw_i = int(max(0, sw))
        ew_i = int(min(ew, len(wp_to_path_idx_raw) - 1))
        if ew_i < sw_i:
            continue
        local_score_smooth = -1.0
        local_score_fallback = -1.0
        local_best_wp = sw_i
        for w in range(sw_i, ew_i + 1):
            ir = int(np.clip(wp_to_path_idx_raw[w], 0, max(0, path_raw.shape[0] - 1)))
            is_ = int(np.clip(wp_to_path_idx_short[w], 0, max(0, path_short.shape[0] - 1)))
            im = int(np.clip(wp_to_path_idx_smooth[w], 0, max(0, path_smooth.shape[0] - 1)))
            pr = path_raw[ir, :2]
            ps = path_short[is_, :2]
            pm = path_smooth[im, :2]
            d_rs = float(np.linalg.norm(pr - ps))
            d_sm = float(np.linalg.norm(ps - pm))
            d_rm = float(np.linalg.norm(pr - pm))
            a_short = _turn_angle_deg(path_short, is_)
            a_smooth = _turn_angle_deg(path_smooth, im)
            angle_gain = max(0.0, a_short - a_smooth)
            if mode == "smoothing_advantage":
                # Aggressive focus: emphasize shortcut->smoothed geometric change and turn reduction.
                score_smooth = d_sm + 0.06 * angle_gain + 0.015 * a_short
            else:
                # Balanced focus: still consider raw/shortcut/smoothed total contrast.
                score_smooth = d_sm + 0.04 * angle_gain
            score_fallback = d_rs + d_sm + d_rm
            if score_smooth > local_score_smooth:
                local_score_smooth = score_smooth
                local_score_fallback = score_fallback
                local_best_wp = w

        if local_score_smooth > best_score_smooth:
            best_score_smooth = local_score_smooth
            best_score_fallback = local_score_fallback
            best_region = region
            best_wp = local_best_wp
            local_pts = mission_wp[sw_i : ew_i + 1, :2]
            if local_pts.shape[0] > 1:
                ext = np.max(local_pts, axis=0) - np.min(local_pts, axis=0)
                best_span = float(np.clip(0.18 * max(float(ext[0]), float(ext[1])), 6.0, max(6.0, margin_m)))

    if best_region is None or best_wp < 0:
        return

    # If shortcut-vs-smoothed difference is globally weak, fall back to wider "most distinct" crop.
    fallback_th = 0.55 if mode == "smoothing_advantage" else 0.35
    if best_score_smooth < fallback_th and best_score_fallback > 0.0:
        best_span = float(max(best_span, margin_m))

    cx = float(mission_wp[best_wp, 0])
    cy = float(mission_wp[best_wp, 1])
    half = float(max(6.0, best_span))
    lo = np.array([cx - half, cy - half], dtype=np.float64)
    hi = np.array([cx + half, cy + half], dtype=np.float64)

    def _clip_window(path_xyz: np.ndarray) -> np.ndarray:
        if path_xyz.shape[0] <= 2:
            return path_xyz
        mask = (
            (path_xyz[:, 0] >= lo[0])
            & (path_xyz[:, 0] <= hi[0])
            & (path_xyz[:, 1] >= lo[1])
            & (path_xyz[:, 1] <= hi[1])
        )
        idx = np.flatnonzero(mask)
        if idx.size < 2:
            return path_xyz
        s = int(max(0, idx[0] - 1))
        e = int(min(path_xyz.shape[0] - 1, idx[-1] + 1))
        return path_xyz[s : e + 1]

    local_raw = _clip_window(path_raw)
    local_short = _clip_window(path_short)
    local_smooth = _clip_window(path_smooth)

    fig, ax = plt.subplots(1, 1, figsize=(8.8, 6.8))
    if cloud_xyz.shape[0] > 0:
        mask = (
            (cloud_xyz[:, 0] >= lo[0])
            & (cloud_xyz[:, 0] <= hi[0])
            & (cloud_xyz[:, 1] >= lo[1])
            & (cloud_xyz[:, 1] <= hi[1])
        )
        sub = cloud_xyz[mask]
        sub_rgb = cloud_rgb[mask] if (cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]) else None
        if sub.shape[0] > 0:
            if sub_rgb is not None and sub_rgb.shape[0] == sub.shape[0]:
                ax.scatter(sub[:, 0], sub[:, 1], c=sub_rgb, s=1.0, alpha=0.28)
            else:
                ax.scatter(sub[:, 0], sub[:, 1], c="silver", s=1.0, alpha=0.28)
    ax.plot(local_raw[:, 0], local_raw[:, 1], c="gray", linewidth=1.2, alpha=0.72, label="raw")
    ax.plot(local_short[:, 0], local_short[:, 1], c="#fb8500", linewidth=1.9, alpha=0.96, label="shortcut")
    ax.plot(local_smooth[:, 0], local_smooth[:, 1], c="deepskyblue", linewidth=2.2, alpha=0.96, label="smoothed")
    ax.scatter([cx], [cy], c="limegreen", s=55, marker="o")
    ax.set_xlim(float(lo[0]), float(hi[0]))
    ax.set_ylim(float(lo[1]), float(hi[1]))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    if mode == "smoothing_advantage":
        ax.set_title(f"Local Path Compare (smoothing-focus): {best_region}")
    else:
        ax.set_title(f"Local Path Compare (most distinct): {best_region}")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_png, dpi=220)
    plt.close(fig)


def build_simulation_frames(
    path_len: int,
    step_stride: int,
    waypoint_indices: Sequence[int],
    pause_frames: int,
) -> List[int]:
    if path_len <= 0:
        return []
    stride = int(max(1, step_stride))
    wp_set = set(int(np.clip(i, 0, path_len - 1)) for i in waypoint_indices)
    frames: List[int] = []
    i = 0
    while i < path_len:
        frames.append(i)
        if i in wp_set and pause_frames > 0:
            for _ in range(pause_frames):
                frames.append(i)
        i += stride
    if frames[-1] != (path_len - 1):
        frames.append(path_len - 1)
    return frames


def create_station_simulation_gif(
    out_gif: Path,
    cloud_xyz: np.ndarray,
    cloud_rgb: Optional[np.ndarray],
    path_smooth: np.ndarray,
    mission_wp_path_idx: Sequence[int],
    start_pt: np.ndarray,
    cfg: dict,
) -> None:
    if path_smooth.shape[0] <= 1:
        return
    fps = int(max(1, cfg.get("fps", 10)))
    step_stride = int(max(1, cfg.get("path_step_stride", 20)))
    pause_sec = float(max(0.0, min(0.5, cfg.get("pause_at_waypoint_s", 0.3))))
    pause_frames = int(round(pause_sec * fps))
    frames = build_simulation_frames(
        path_len=int(path_smooth.shape[0]),
        step_stride=step_stride,
        waypoint_indices=mission_wp_path_idx,
        pause_frames=pause_frames if bool(cfg.get("pause_enable", True)) else 0,
    )
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    if cloud_xyz.shape[0] > 0:
        if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
            ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c=cloud_rgb, s=0.4, alpha=0.2, depthshade=False)
        else:
            ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c="silver", s=0.4, alpha=0.2, depthshade=False)
    ax.plot(path_smooth[:, 0], path_smooth[:, 1], path_smooth[:, 2], c="lightsteelblue", linewidth=1.0, alpha=0.45)
    ax.scatter([start_pt[0]], [start_pt[1]], [start_pt[2]], marker="*", s=120, c="gold", edgecolors="k", linewidths=0.6, depthshade=False)
    (trail_line,) = ax.plot([], [], [], c="deepskyblue", linewidth=2.0, alpha=0.95)
    drone = ax.scatter([path_smooth[0, 0]], [path_smooth[0, 1]], [path_smooth[0, 2]], c="orange", s=float(cfg.get("drone_marker_size", 85.0)), edgecolors="k", linewidths=0.5, depthshade=False)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title("Phase3 Mission Simulation")
    ax.view_init(elev=float(cfg.get("elev_deg", 24.0)), azim=float(cfg.get("azim_deg", -35.0)))
    fig.tight_layout()

    def _anim(k: int):
        idx = int(frames[k])
        sub = path_smooth[: idx + 1]
        trail_line.set_data(sub[:, 0], sub[:, 1])
        trail_line.set_3d_properties(sub[:, 2])
        p = path_smooth[idx]
        drone._offsets3d = ([p[0]], [p[1]], [p[2]])
        return trail_line, drone

    ani = animation.FuncAnimation(fig, _anim, frames=len(frames), interval=1000.0 / fps, blit=False)
    ani.save(out_gif, writer="pillow", fps=fps)
    plt.close(fig)


def _normalize_vec(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)
    return (v / n).astype(np.float64)


def _frustum_vertices(
    origin: np.ndarray,
    forward: np.ndarray,
    range_m: float,
    fov_h_deg: float,
    fov_v_deg: float,
) -> np.ndarray:
    f = _normalize_vec(forward)
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    right = np.cross(f, world_up)
    if float(np.linalg.norm(right)) < 1e-6:
        right = np.cross(f, np.array([0.0, 1.0, 0.0], dtype=np.float64))
    right = _normalize_vec(right)
    up = _normalize_vec(np.cross(right, f))
    center = origin + f * float(range_m)
    hw = float(range_m * math.tan(math.radians(max(1.0, fov_h_deg) * 0.5)))
    hh = float(range_m * math.tan(math.radians(max(1.0, fov_v_deg) * 0.5)))
    v0 = center + right * hw + up * hh
    v1 = center - right * hw + up * hh
    v2 = center - right * hw - up * hh
    v3 = center + right * hw - up * hh
    return np.vstack([origin, v0, v1, v2, v3])


def _points_in_frustum_mask(
    pts: np.ndarray,
    origin: np.ndarray,
    forward: np.ndarray,
    range_m: float,
    fov_h_deg: float,
    fov_v_deg: float,
) -> np.ndarray:
    if pts.shape[0] == 0:
        return np.zeros((0,), dtype=bool)
    f = _normalize_vec(forward)
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    right = np.cross(f, world_up)
    if float(np.linalg.norm(right)) < 1e-6:
        right = np.cross(f, np.array([0.0, 1.0, 0.0], dtype=np.float64))
    right = _normalize_vec(right)
    up = _normalize_vec(np.cross(right, f))

    rel = pts - origin[None, :]
    zf = rel @ f
    in_front = zf > 1e-3
    in_range = zf <= float(max(0.2, range_m))
    xr = rel @ right
    yu = rel @ up
    h_lim = zf * math.tan(math.radians(max(1.0, fov_h_deg) * 0.5))
    v_lim = zf * math.tan(math.radians(max(1.0, fov_v_deg) * 0.5))
    in_h = np.abs(xr) <= h_lim
    in_v = np.abs(yu) <= v_lim
    return in_front & in_range & in_h & in_v


def create_station_simulation_fpv_gif(
    out_gif: Path,
    cloud_xyz: np.ndarray,
    path_smooth: np.ndarray,
    mission_caps: Sequence[CapturePoint],
    mission_wp_path_idx: Sequence[int],
    cfg: dict,
) -> None:
    if path_smooth.shape[0] <= 1:
        return
    if not bool(cfg.get("enable", False)):
        return

    fps = int(max(1, cfg.get("fps", 10)))
    step_stride = int(max(1, cfg.get("path_step_stride", 1)))
    pause_sec = float(max(0.0, cfg.get("pause_at_waypoint_s", 0.5)))
    pause_frames = int(round(pause_sec * fps))
    frames = build_simulation_frames(
        path_len=int(path_smooth.shape[0]),
        step_stride=step_stride,
        waypoint_indices=mission_wp_path_idx,
        pause_frames=pause_frames if bool(cfg.get("pause_enable", True)) else 0,
    )
    if not frames:
        return

    max_cloud_points = int(max(1000, cfg.get("max_cloud_points", 90000)))
    if cloud_xyz.shape[0] > max_cloud_points:
        rng = np.random.default_rng(int(cfg.get("cloud_sample_seed", 42)))
        pick = rng.choice(cloud_xyz.shape[0], size=max_cloud_points, replace=False)
        cloud_use = cloud_xyz[pick]
    else:
        cloud_use = cloud_xyz

    idx_to_caps: Dict[int, List[CapturePoint]] = {}
    idx_to_cap_primary: Dict[int, CapturePoint] = {}
    n_caps = len(mission_caps)
    for i, pidx in enumerate(mission_wp_path_idx):
        if i >= n_caps:
            break
        j = int(np.clip(pidx, 0, path_smooth.shape[0] - 1))
        cp = mission_caps[i]
        idx_to_caps.setdefault(j, []).append(cp)
        if j not in idx_to_cap_primary and str(cp.device).lower() != "home":
            idx_to_cap_primary[j] = cp

    local_r = float(max(6.0, cfg.get("local_radius_m", 22.0)))
    z_span = float(max(6.0, cfg.get("local_z_span_m", 12.0)))
    lookahead = int(max(1, cfg.get("lookahead_step", 3)))
    show_fov = bool(cfg.get("show_fov", True))
    show_covered = bool(cfg.get("show_covered_points", True))
    fov_every_n = int(max(1, cfg.get("fov_every_n_waypoints", 4)))
    # Keep FOV visible around selected waypoints for a few path steps.
    fov_window_steps = int(max(0, cfg.get("fov_window_path_steps", 3)))
    fov_h = float(max(10.0, cfg.get("fov_h_deg", 66.0)))
    fov_v = float(max(10.0, cfg.get("fov_v_deg", 46.0)))
    fov_range = float(max(1.0, cfg.get("fov_range_m", 12.0)))
    covered_max = int(max(200, cfg.get("covered_points_max", 2500)))
    elev_deg = float(cfg.get("elev_deg", 24.0))
    azim_deg = float(cfg.get("azim_deg", -35.0))

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    p0 = path_smooth[int(frames[0])]
    if cloud_use.shape[0] > 0:
        ax.scatter(cloud_use[:, 0], cloud_use[:, 1], cloud_use[:, 2], c="silver", s=0.7, alpha=0.09, depthshade=False)
    (trail_line,) = ax.plot([], [], [], c="deepskyblue", linewidth=2.0, alpha=0.95)
    drone = ax.scatter([p0[0]], [p0[1]], [p0[2]], c="orange", s=float(cfg.get("drone_marker_size", 90.0)), edgecolors="k", linewidths=0.5, depthshade=False)
    fov_poly = Poly3DCollection([], facecolors=(0.55, 0.95, 0.55, 0.42), edgecolors=(0.35, 0.88, 0.35, 0.95), linewidths=1.5)
    ax.add_collection3d(fov_poly)
    fov_poly.set_zsort("max")
    fov_edges = []
    for _ in range(5):
        (ln,) = ax.plot([], [], [], c="#98fb98", linewidth=1.8, alpha=0.98)
        fov_edges.append(ln)
    covered = ax.scatter([], [], [], c="#98fb98", s=2.2, alpha=0.45, depthshade=False) if show_covered else None
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title("Phase3 Mission Simulation (Third-person + FOV)")
    # Use mission-path bounds for a tighter third-person camera, so FOV is visible.
    merged = path_smooth.copy()
    lo = np.min(merged, axis=0)
    hi = np.max(merged, axis=0)
    pad_xy = float(max(12.0, 0.10 * max(float(hi[0] - lo[0]), float(hi[1] - lo[1]))))
    pad_z = float(max(6.0, 0.14 * max(1.0, float(hi[2] - lo[2]))))
    ax.set_xlim(float(lo[0] - pad_xy), float(hi[0] + pad_xy))
    ax.set_ylim(float(lo[1] - pad_xy), float(hi[1] + pad_xy))
    ax.set_zlim(float(lo[2] - pad_z), float(hi[2] + pad_z))
    ax.view_init(elev=elev_deg, azim=azim_deg)
    fig.tight_layout()

    # Show FOV only on sparse inspection waypoints for visibility and rendering stability.
    inspect_idx = sorted(idx_to_cap_primary.keys())
    selected_fov_idx: Set[int] = set()
    if inspect_idx:
        for kk in range(0, len(inspect_idx), fov_every_n):
            selected_fov_idx.add(int(inspect_idx[kk]))
    selected_fov_sorted = np.array(sorted(selected_fov_idx), dtype=np.int32) if selected_fov_idx else np.empty((0,), dtype=np.int32)

    def _anchor_for_fov(cur_idx: int) -> Optional[int]:
        if cur_idx in selected_fov_idx:
            return int(cur_idx)
        if fov_window_steps <= 0 or selected_fov_sorted.size == 0:
            return None
        pos = int(np.argmin(np.abs(selected_fov_sorted - int(cur_idx))))
        best = int(selected_fov_sorted[pos])
        if abs(best - int(cur_idx)) <= fov_window_steps:
            return best
        return None

    def _anim(k: int):
        idx = int(frames[k])
        p = path_smooth[idx]
        j2 = int(min(path_smooth.shape[0] - 1, idx + lookahead))
        dvec = path_smooth[j2] - p
        if float(np.linalg.norm(dvec)) < 1e-6 and idx > 0:
            dvec = p - path_smooth[idx - 1]
        dvec = _normalize_vec(dvec)

        sub = path_smooth[: idx + 1]
        trail_line.set_data(sub[:, 0], sub[:, 1])
        trail_line.set_3d_properties(sub[:, 2])
        drone._offsets3d = ([p[0]], [p[1]], [p[2]])

        show_overlay_now = False
        look_dir = dvec.copy()
        anchor_idx = _anchor_for_fov(idx)
        if anchor_idx is not None:
            cp = idx_to_cap_primary.get(anchor_idx)
            if cp is None:
                caps_here = idx_to_caps.get(anchor_idx, [])
                cp = caps_here[0] if caps_here else None
            if cp is not None:
                tvec = np.array([float(cp.target_x - cp.x), float(cp.target_y - cp.y), float(cp.target_z - cp.z)], dtype=np.float64)
                if float(np.linalg.norm(tvec)) > 1e-3:
                    look_dir = _normalize_vec(tvec)
                show_overlay_now = bool(show_fov)

        if show_overlay_now:
            verts = _frustum_vertices(origin=p, forward=look_dir, range_m=fov_range, fov_h_deg=fov_h, fov_v_deg=fov_v)
            o = verts[0]
            v0, v1, v2, v3 = verts[1], verts[2], verts[3], verts[4]
            faces = [[o, v0, v1], [o, v1, v2], [o, v2, v3], [o, v3, v0], [v0, v1, v2, v3]]
            fov_poly.set_verts(faces)
            ray_pairs = [(o, v0), (o, v1), (o, v2), (o, v3)]
            for li, (a, b) in enumerate(ray_pairs):
                fov_edges[li].set_data([a[0], b[0]], [a[1], b[1]])
                fov_edges[li].set_3d_properties([a[2], b[2]])
            base = np.vstack([v0, v1, v2, v3, v0])
            fov_edges[4].set_data(base[:, 0], base[:, 1])
            fov_edges[4].set_3d_properties(base[:, 2])
            if covered is not None and cloud_use.shape[0] > 0:
                msk_local = (
                    (np.abs(cloud_use[:, 0] - p[0]) <= local_r)
                    & (np.abs(cloud_use[:, 1] - p[1]) <= local_r)
                    & (np.abs(cloud_use[:, 2] - p[2]) <= z_span)
                )
                local_pts = cloud_use[msk_local]
                msk = _points_in_frustum_mask(local_pts, origin=p, forward=look_dir, range_m=fov_range, fov_h_deg=fov_h, fov_v_deg=fov_v)
                cov = local_pts[msk]
                if cov.shape[0] > covered_max:
                    cov = cov[:covered_max]
                covered._offsets3d = (cov[:, 0], cov[:, 1], cov[:, 2]) if cov.shape[0] > 0 else ([], [], [])
        else:
            fov_poly.set_verts([])
            for ln in fov_edges:
                ln.set_data([], [])
                ln.set_3d_properties([])
            if covered is not None:
                covered._offsets3d = ([], [], [])

        return trail_line, drone

    ani = animation.FuncAnimation(fig, _anim, frames=len(frames), interval=1000.0 / fps, blit=False)
    ani.save(out_gif, writer="pillow", fps=fps)
    plt.close(fig)


def plot_experiment_compare(out_png: Path, rows: Sequence[dict]) -> None:
    if not rows:
        return
    names = [str(r["scenario"]) for r in rows]
    cov = [float(r["estimated_global_coverage_ratio"]) for r in rows]
    length = [float(r["path_length_smoothed_m"]) for r in rows]
    score = [float(r["trackability_smoothed"]) for r in rows]

    fig, axs = plt.subplots(1, 3, figsize=(15, 4.8))
    x = np.arange(len(names))
    axs[0].bar(x, cov, color="#2a9d8f")
    axs[0].set_title("Estimated Coverage")
    axs[0].set_xticks(x, names, rotation=15)
    axs[0].set_ylim(0.0, 1.0)
    axs[0].grid(True, axis="y", alpha=0.25)

    axs[1].bar(x, length, color="#e76f51")
    axs[1].set_title("Smoothed Path Length (m)")
    axs[1].set_xticks(x, names, rotation=15)
    axs[1].grid(True, axis="y", alpha=0.25)

    axs[2].bar(x, score, color="#457b9d")
    axs[2].set_title("Trackability Score")
    axs[2].set_xticks(x, names, rotation=15)
    axs[2].set_ylim(0.0, 100.0)
    axs[2].grid(True, axis="y", alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_png, dpi=220)
    plt.close(fig)


def _load_csv_rows(path: Path) -> List[dict]:
    rows: List[dict] = []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append({str(k): v for k, v in row.items()})
    return rows


def plot_phase1_safety_experiment_compare(out_png: Path, rows: Sequence[dict]) -> None:
    if not rows:
        return
    names = [str(r.get("case_name", "")) for r in rows]
    colors = ["#007aff" if str(r.get("mode", "")).lower() == "dynamic" else "#8e8e93" for r in rows]
    cov = [float(r.get("estimated_global_coverage_ratio", 0.0)) for r in rows]
    tr = [float(r.get("trackability_smoothed", 0.0)) for r in rows]
    ln = [float(r.get("path_length_smoothed_m", 0.0)) for r in rows]
    low = [float(r.get("low_clearance_smoothed_ratio", 0.0)) for r in rows]
    x = np.arange(len(names))

    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    axs = axs.ravel()

    axs[0].bar(x, cov, color=colors)
    axs[0].set_title("Coverage (higher is better)")
    axs[0].set_ylim(0.0, 1.0)
    axs[0].set_xticks(x, names, rotation=25, ha="right")
    axs[0].grid(True, axis="y", alpha=0.25)

    axs[1].bar(x, tr, color=colors)
    axs[1].set_title("Trackability Smoothed (higher is better)")
    axs[1].set_ylim(0.0, 100.0)
    axs[1].set_xticks(x, names, rotation=25, ha="right")
    axs[1].grid(True, axis="y", alpha=0.25)

    axs[2].bar(x, ln, color=colors)
    axs[2].set_title("Smoothed Path Length (lower is better)")
    axs[2].set_xticks(x, names, rotation=25, ha="right")
    axs[2].grid(True, axis="y", alpha=0.25)

    axs[3].bar(x, low, color=colors)
    axs[3].set_title("Low Clearance Ratio (lower is better)")
    axs[3].set_ylim(0.0, 1.0)
    axs[3].set_xticks(x, names, rotation=25, ha="right")
    axs[3].grid(True, axis="y", alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_png, dpi=220)
    plt.close(fig)


def run_phase1_safety_sweep_experiment(
    cfg: dict,
    phase1_report: dict,
    device_data: Dict[str, DeviceData],
    region_sequence: Sequence[dict],
    out_dir: Path,
) -> dict:
    ecfg = cfg.get("phase1_safety_sweep_experiment", {})
    if not bool(ecfg.get("enable", False)):
        return {"enabled": False, "rows": []}

    root = Path(cfg["paths"]["project_root"]).resolve()
    default_csv = root / "outputs" / "phase1_feasible_space" / "sweep_maps" / "sweep_summary.csv"
    sweep_csv = (root / ecfg.get("sweep_summary_csv", str(default_csv))).resolve()
    if not sweep_csv.exists():
        return {"enabled": True, "error": f"sweep_summary_csv not found: {sweep_csv}", "rows": []}

    exp_cfg = cfg.get("experiments", {})
    scenarios = exp_cfg.get("scenarios", [])
    scenario_name = str(ecfg.get("scenario_name", cfg["mission"].get("primary_scenario", "balanced")))
    scenario_patch = {}
    for s in scenarios:
        if str(s.get("name", "")) == scenario_name:
            scenario_patch = s
            break
    scenario_cfg = deep_update(cfg, scenario_patch if isinstance(scenario_patch, dict) else {})
    scenario_cfg["mission"]["primary_scenario"] = scenario_name

    rows_in = _load_csv_rows(sweep_csv)
    selected_names = {str(x) for x in ecfg.get("selected_case_names", [])}
    max_cases_per_mode = int(max(0, ecfg.get("max_cases_per_mode", 0)))
    chosen_rows: List[dict] = []
    mode_count: Dict[str, int] = {"dynamic": 0, "static": 0}
    for r in rows_in:
        name = str(r.get("name", ""))
        mode = str(r.get("mode", "")).lower()
        if selected_names and name not in selected_names:
            continue
        if max_cases_per_mode > 0 and mode in mode_count and mode_count[mode] >= max_cases_per_mode:
            continue
        chosen_rows.append(r)
        if mode in mode_count:
            mode_count[mode] += 1
    if not chosen_rows:
        return {"enabled": True, "error": "no sweep rows selected", "rows": []}

    result_rows: List[dict] = []
    for i, r in enumerate(chosen_rows):
        mode = str(r.get("mode", "")).lower()
        case_name = str(r.get("name", f"case_{i+1}"))
        npz_str = str(r.get("npz_path", "")).strip()
        npz_path = Path(npz_str)
        if not npz_path.is_absolute():
            npz_path = (root / npz_str).resolve()
        if not npz_path.exists():
            log(f"[phase1_safety_exp] skip missing npz: {npz_path}")
            continue
        log(f"[phase1_safety_exp] case {i+1}/{len(chosen_rows)}: {mode}:{case_name}")
        station_case = StationGrid(npz_path)

        phase1_case_report = deepcopy(phase1_report)
        try:
            fv = safe_int(r.get("free_voxels", 0), 0)
            tv = fv + safe_int(r.get("blocked_voxels", 0), 0)
            phase1_case_report.setdefault("grid", {})
            phase1_case_report["grid"]["free_voxels"] = int(fv)
            phase1_case_report["grid"]["total_voxels"] = int(max(1, tv))
        except Exception:
            pass

        res = run_single_scenario(
            cfg=scenario_cfg,
            station=station_case,
            phase1_report=phase1_case_report,
            device_data=device_data,
            region_sequence=region_sequence,
            out_dir=out_dir,
            scenario_name=f"phase1_safety_{case_name}",
            write_full_outputs=False,
        )
        result_rows.append(
            {
                "mode": mode,
                "case_name": case_name,
                "d0": safe_float(r.get("d0", 0.0), 0.0),
                "k_gps": safe_float(r.get("k_gps", 0.0), 0.0),
                "d_ctrl": safe_float(r.get("d_ctrl", 0.0), 0.0),
                "static_d_eff": safe_float(r.get("static_d_eff", 0.0), 0.0),
                "phase1_free_pct": safe_float(r.get("free_pct", 0.0), 0.0),
                "estimated_global_coverage_ratio": float(res.get("estimated_global_coverage_ratio", 0.0)),
                "trackability_smoothed": float(res.get("trackability_smoothed", {}).get("trackability_score_0_100", 0.0)),
                "path_length_smoothed_m": float(res.get("path_length_smoothed_m", 0.0)),
                "segment_success_ratio": float(res.get("segment_success_ratio", 0.0)),
                "low_clearance_smoothed_ratio": float(res.get("trackability_smoothed", {}).get("low_clearance_ratio", 0.0)),
                "scenario_total_s": float(res.get("timing", {}).get("scenario_total_s", 0.0)),
            }
        )

    exp_dir = out_dir / "experiments"
    ensure_dir(exp_dir)
    out_csv = exp_dir / "phase1_safety_distance_effect_summary.csv"
    out_json = exp_dir / "phase1_safety_distance_effect_summary.json"
    out_png = exp_dir / "phase1_safety_distance_effect_compare.png"

    if result_rows:
        with out_csv.open("w", encoding="utf-8-sig", newline="") as f:
            keys = [
                "mode",
                "case_name",
                "d0",
                "k_gps",
                "d_ctrl",
                "static_d_eff",
                "phase1_free_pct",
                "estimated_global_coverage_ratio",
                "trackability_smoothed",
                "path_length_smoothed_m",
                "segment_success_ratio",
                "low_clearance_smoothed_ratio",
                "scenario_total_s",
            ]
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for rr in result_rows:
                w.writerow({k: rr.get(k, "") for k in keys})

    mode_stats: Dict[str, dict] = {}
    for mode in ("dynamic", "static"):
        sub = [r for r in result_rows if str(r.get("mode", "")).lower() == mode]
        if not sub:
            continue
        mode_stats[mode] = {
            "n": int(len(sub)),
            "coverage_mean": float(np.mean([float(r["estimated_global_coverage_ratio"]) for r in sub])),
            "trackability_mean": float(np.mean([float(r["trackability_smoothed"]) for r in sub])),
            "path_length_mean_m": float(np.mean([float(r["path_length_smoothed_m"]) for r in sub])),
            "low_clearance_mean": float(np.mean([float(r["low_clearance_smoothed_ratio"]) for r in sub])),
            "segment_success_mean": float(np.mean([float(r["segment_success_ratio"]) for r in sub])),
        }
    compare = {}
    if "dynamic" in mode_stats and "static" in mode_stats:
        compare = {
            "coverage_dynamic_minus_static": float(mode_stats["dynamic"]["coverage_mean"] - mode_stats["static"]["coverage_mean"]),
            "trackability_dynamic_minus_static": float(mode_stats["dynamic"]["trackability_mean"] - mode_stats["static"]["trackability_mean"]),
            "path_length_dynamic_minus_static_m": float(mode_stats["dynamic"]["path_length_mean_m"] - mode_stats["static"]["path_length_mean_m"]),
            "low_clearance_dynamic_minus_static": float(mode_stats["dynamic"]["low_clearance_mean"] - mode_stats["static"]["low_clearance_mean"]),
        }

    out_payload = {
        "enabled": True,
        "scenario_name": scenario_name,
        "sweep_summary_csv": str(sweep_csv),
        "rows": result_rows,
        "mode_stats": mode_stats,
        "mode_compare_dynamic_minus_static": compare,
        "notes": [
            "This is a supplemental phase1 dynamic/static safety-distance impact experiment on phase3 metrics.",
            "It does not replace primary phase3 conservative/balanced/aggressive experiments.",
        ],
    }
    write_json(out_json, out_payload)
    if result_rows:
        plot_phase1_safety_experiment_compare(out_png, result_rows)
    return out_payload


def run_single_scenario(
    cfg: dict,
    station: StationGrid,
    phase1_report: dict,
    device_data: Dict[str, DeviceData],
    region_sequence: Sequence[dict],
    out_dir: Path,
    scenario_name: str,
    write_full_outputs: bool,
) -> dict:
    t_s0 = time.perf_counter()
    timing: Dict[str, float] = {}
    device_to_region, expected_region_order, expected_device_order = build_region_maps(region_sequence)
    routes: Dict[str, CompressedDeviceRoute] = {}
    t0 = time.perf_counter()
    for d, dd in device_data.items():
        routes[d] = compress_device_route(dd, region=device_to_region.get(d, "unknown"), cfg=cfg)
    timing["compression_s"] = float(time.perf_counter() - t0)

    t0 = time.perf_counter()
    routes = apply_phase2_style_local_order(routes=routes, cfg=cfg)
    timing["phase2_style_local_order_s"] = float(time.perf_counter() - t0)

    t0 = time.perf_counter()
    routes = apply_local_refinement(routes=routes, station=station, cfg=cfg)
    timing["local_refinement_s"] = float(time.perf_counter() - t0)

    mission_cfg = cfg["mission"]
    start_xyz = np.array(mission_cfg["start_home_xyz"], dtype=np.float64)
    start_frame = str(mission_cfg.get("start_home_frame", "raw")).lower()
    angle_rad = float(phase1_report.get("rotation", {}).get("angle_rad", 0.0))
    if start_frame == "raw":
        start_xyz = rotate_xy(start_xyz[None, :], angle_rad)[0]
    start_xyz = nearest_free_world(
        station,
        start_xyz,
        max_k=int(cfg["mission"].get("home_snap_max_search_vox", 12)),
    )
    return_home = bool(mission_cfg.get("return_to_home", True))

    t0 = time.perf_counter()
    order, reverse_flags, est_order_cost, order_candidates = select_best_device_order(
        region_sequence=region_sequence,
        routes=routes,
        start_pt=start_xyz,
        return_home=return_home,
        station=station,
        ordering_cfg=cfg["ordering"],
    )
    timing["ordering_s"] = float(time.perf_counter() - t0)

    t0 = time.perf_counter()
    mission_caps, boundaries = build_mission_waypoints(
        order=order,
        reverse_flags=reverse_flags,
        routes=routes,
        start_pt=start_xyz,
        return_home=return_home,
    )
    mission_nav = build_mission_nav_points(mission_caps)
    timing["mission_assembly_s"] = float(time.perf_counter() - t0)

    t0 = time.perf_counter()
    path_raw, segments, wp_to_path, path_raw_len, seg_success = plan_full_mission_path(
        nav_pts=mission_nav,
        station=station,
        cfg=cfg["planning"],
    )
    timing["planning_s"] = float(time.perf_counter() - t0)

    t0 = time.perf_counter()
    shortcut_cfg = cfg.get("postprocess_shortcut", {})
    path_short = shortcut_path_line_of_sight(path_raw, station, shortcut_cfg)
    path_short_len = float(np.linalg.norm(path_short[1:] - path_short[:-1], axis=1).sum()) if path_short.shape[0] > 1 else 0.0
    wp_to_path_short = map_waypoints_to_path_indices(path_short, mission_nav)
    timing["shortcut_s"] = float(time.perf_counter() - t0)

    t0 = time.perf_counter()
    locked_idx = set(int(i) for i in wp_to_path_short)
    path_smooth = smooth_path_safe(
        path_pts=path_short,
        station=station,
        cfg=cfg["smoothing"],
        locked_indices=locked_idx,
    )
    path_smooth_len = float(np.linalg.norm(path_smooth[1:] - path_smooth[:-1], axis=1).sum()) if path_smooth.shape[0] > 1 else 0.0
    wp_to_path_smooth = map_waypoints_to_path_indices(path_smooth, mission_nav)
    timing["smoothing_s"] = float(time.perf_counter() - t0)

    t0 = time.perf_counter()
    tcfg = cfg["trackability"]
    tr_raw = compute_trackability_metrics(path_raw, station, tcfg)
    tr_short = compute_trackability_metrics(path_short, station, tcfg)
    tr_smooth = compute_trackability_metrics(path_smooth, station, tcfg)
    timing["trackability_eval_s"] = float(time.perf_counter() - t0)

    weighted_cov_n = 0
    weighted_cov_sum = 0.0
    retained_total = 0
    raw_total = 0
    for d in order:
        rt = routes[d]
        dd = device_data[d]
        weighted_cov_n += int(max(1, dd.sampled_surface_points))
        weighted_cov_sum += float(rt.estimated_coverage_ratio) * float(max(1, dd.sampled_surface_points))
        retained_total += int(rt.retained_count)
        raw_total += int(rt.raw_count)
    estimated_global_cov = float(weighted_cov_sum / max(1, weighted_cov_n))
    global_retention_ratio = float(retained_total / max(1, raw_total))

    order_match = analyze_order_match(
        order=order,
        region_order=expected_region_order,
        device_to_region=device_to_region,
        strict_target=mission_cfg.get("strict_device_order_target", expected_device_order),
    )
    seg_modes = {"straight": 0, "astar": 0, "fallback_direct": 0}
    astar_nodes_total = 0
    for s in segments:
        m = str(s.get("mode", ""))
        if m in seg_modes:
            seg_modes[m] += 1
        if m == "astar":
            astar_nodes_total += int(s.get("path_node_count", 0))

    order_symbolic = "O(C * D * 2^2 + C_conn)"
    strategy_name = str(cfg.get("ordering", {}).get("strategy", "region_constrained")).lower()
    if strategy_name in ("greedy_biased", "priority_regularized", "priority_regularized_greedy"):
        order_symbolic = "O(D^2 * 2 + C_conn)"
    complexity = {
        "symbolic": {
            "device_compression": "O(sum_i N_i log N_i)",
            "order_search": order_symbolic,
            "path_planning": "O(S * (L_line + Astar))",
            "shortcut": "O(P * H * line_check)",
            "smoothing": "O(P * I * W)",
            "trackability": "O(P)",
        },
        "variables": {
            "D_devices": int(len(order)),
            "C_order_candidates": int(order_candidates),
            "N_raw_viewpoints_total": int(raw_total),
            "N_retained_viewpoints_total": int(retained_total),
            "S_segments": int(len(segments)),
            "P_path_points_raw": int(path_raw.shape[0]),
            "P_path_points_shortcut": int(path_short.shape[0]),
            "P_path_points_smoothed": int(path_smooth.shape[0]),
            "Astar_segments": int(seg_modes["astar"]),
            "Astar_nodes_total": int(astar_nodes_total),
        },
        "timing_seconds": timing,
    }

    result = {
        "scenario": scenario_name,
        "ordering_strategy": str(cfg.get("ordering", {}).get("strategy", "region_constrained")),
        "start_home_xyz_used": [float(v) for v in start_xyz.tolist()],
        "start_home_frame_config": start_frame,
        "rotation_angle_deg": float(math.degrees(angle_rad)),
        "device_order": order,
        "device_reverse_flags": [bool(v) for v in reverse_flags],
        "order_candidate_count": int(order_candidates),
        "estimated_order_cost_m": float(est_order_cost),
        "retained_viewpoints_total": int(retained_total),
        "raw_viewpoints_total": int(raw_total),
        "global_retention_ratio": global_retention_ratio,
        "estimated_global_coverage_ratio": estimated_global_cov,
        "path_length_raw_m": float(path_raw_len),
        "path_length_shortcut_m": float(path_short_len),
        "path_length_smoothed_m": float(path_smooth_len),
        "segment_success_ratio": float(seg_success),
        "trackability_raw": tr_raw,
        "trackability_shortcut": tr_short,
        "trackability_smoothed": tr_smooth,
        "order_match": order_match,
        "boundaries": boundaries,
        "timing": timing,
        "complexity": complexity,
        "planning_mode_counts": seg_modes,
        "astar_nodes_total": int(astar_nodes_total),
        "path_points": {
            "raw": int(path_raw.shape[0]),
            "shortcut": int(path_short.shape[0]),
            "smoothed": int(path_smooth.shape[0]),
        },
        "mission_waypoint_to_smoothed_path_idx": [int(i) for i in wp_to_path_smooth],
    }
    result["_viz_path_smooth"] = path_smooth.copy()
    result["_viz_start_xyz"] = start_xyz.copy()

    if write_full_outputs:
        t0 = time.perf_counter()
        ensure_dir(out_dir)
        experiments_dir = out_dir / "experiments"
        ensure_dir(experiments_dir)

        order_meta = {b["device"]: {"region": b["region"], "order_index": b["order_index"]} for b in boundaries}
        write_capture_mission_csv(out_dir / "mission_waypoints_capture.csv", mission_caps, order_meta=order_meta)
        write_nav_csv(out_dir / "mission_waypoints_nav.csv", mission_nav)
        write_path_csv(out_dir / "path_waypoints.csv", path_raw)
        write_path_csv(out_dir / "path_waypoints_shortcut.csv", path_short)
        write_path_csv(out_dir / "path_waypoints_display.csv", path_smooth)
        write_segments_csv(out_dir / "path_segments.csv", segments)
        write_json(
            out_dir / "phase3_complexity_and_timing.json",
            {
                "scenario": scenario_name,
                "ordering_strategy": result.get("ordering_strategy"),
                "timing_s": timing,
                "complexity": complexity,
                "planning_mode_counts": seg_modes,
                "astar_nodes_total": int(astar_nodes_total),
                "path_points": result["path_points"],
            },
        )
        write_json(
            out_dir / "phase3_coverage_summary.json",
            {
                "estimated_global_coverage_ratio": float(estimated_global_cov),
                "global_retention_ratio": float(global_retention_ratio),
                "retained_viewpoints_total": int(retained_total),
                "raw_viewpoints_total": int(raw_total),
            },
        )

        seg_len_by_start_wp = {int(s["start_wp_id"]): float(s["length_m"]) for s in segments}
        summary_rows = []
        for b in boundaries:
            d = b["device"]
            dd = device_data[d]
            start_wp = int(b["start_wp_id"])
            end_wp = int(b["end_wp_id"])
            entry_trans = float(seg_len_by_start_wp.get(start_wp - 1, 0.0)) if start_wp > 0 else 0.0
            intra = float(sum(seg_len_by_start_wp.get(i, 0.0) for i in range(start_wp, end_wp)))
            exit_trans = float(seg_len_by_start_wp.get(end_wp, 0.0)) if end_wp < (len(mission_caps) - 1) else 0.0
            summary_rows.append(
                {
                    "device": d,
                    "device_type": b["device_type"],
                    "region": b["region"],
                    "order_index": b["order_index"],
                    "reverse": b["reverse"],
                    "raw_viewpoints": b["raw_viewpoints"],
                    "retained_viewpoints": b["retained_viewpoints"],
                    "retention_ratio": b["retention_ratio"],
                    "phase2_coverage_ratio": dd.phase2_coverage_ratio,
                    "estimated_phase3_coverage_ratio": b["estimated_coverage_ratio"],
                    "phase2_path_length_m": dd.phase2_path_length_m,
                    "phase2_path_segment_success_ratio": dd.phase2_path_segment_success_ratio,
                    "entry_transition_length_m": entry_trans,
                    "intra_device_length_m": intra,
                    "exit_transition_length_m": exit_trans,
                }
            )
        write_device_summary_csv(out_dir / "phase3_station_mission_summary.csv", summary_rows)

        vis_cfg = cfg.get("visualization", {})
        visualize_station_mission(
            out_png=out_dir / "station_mission_plan.png",
            out_gif=out_dir / "station_mission_rotate.gif",
            routes=routes,
            order=order,
            start_pt=start_xyz,
            path_raw=np.empty((0, 3), dtype=np.float64),
            path_smooth=path_smooth,
            vis_cfg=vis_cfg,
        )

        gcfg = cfg.get("global_cloud_visualization", {})
        root = Path(cfg["paths"]["project_root"]).resolve()
        las_path = (root / gcfg.get("station_baseline_las", "pointclouds/substation_baseline.las")).resolve()
        cloud_xyz, cloud_rgb = sample_las_rgb_cloud(
            las_path=las_path,
            max_points=int(gcfg.get("max_points", 120000)),
            chunk_size=int(gcfg.get("chunk_size", 1000000)),
            seed=int(cfg.get("random_seed", 42)) + 303,
            angle_rad=float(phase1_report.get("rotation", {}).get("angle_rad", 0.0)),
        )
        visualize_station_rgb_path(
            out_png=out_dir / "station_mission_rgb_path.png",
            out_gif=out_dir / "station_mission_rgb_path_rotate.gif",
            cloud_xyz=cloud_xyz,
            cloud_rgb=cloud_rgb,
            path_smooth=path_smooth,
            start_pt=start_xyz,
            vis_cfg=gcfg,
        )

        local_regions = [str(x) for x in cfg.get("local_refinement", {}).get("target_regions", [])]
        visualize_local_region_details(
            out_png=out_dir / "station_mission_local_refine_details.png",
            target_regions=local_regions,
            boundaries=boundaries,
            wp_to_path_idx_raw=wp_to_path,
            wp_to_path_idx_short=wp_to_path_short,
            wp_to_path_idx_smooth=wp_to_path_smooth,
            mission_wp=mission_nav,
            path_raw=path_raw,
            path_short=path_short,
            path_smooth=path_smooth,
            cloud_xyz=cloud_xyz,
            cloud_rgb=cloud_rgb,
            margin_m=float(cfg.get("local_refinement", {}).get("detail_margin_m", 8.0)),
            selection_mode="balanced",
        )
        visualize_local_region_details(
            out_png=out_dir / "station_mission_local_refine_details_smoothing_focus.png",
            target_regions=local_regions,
            boundaries=boundaries,
            wp_to_path_idx_raw=wp_to_path,
            wp_to_path_idx_short=wp_to_path_short,
            wp_to_path_idx_smooth=wp_to_path_smooth,
            mission_wp=mission_nav,
            path_raw=path_raw,
            path_short=path_short,
            path_smooth=path_smooth,
            cloud_xyz=cloud_xyz,
            cloud_rgb=cloud_rgb,
            margin_m=float(cfg.get("local_refinement", {}).get("detail_margin_m", 8.0)),
            selection_mode="smoothing_advantage",
        )

        sim_cfg = cfg.get("simulation", {})
        if bool(sim_cfg.get("enable", True)):
            create_station_simulation_gif(
                out_gif=out_dir / "station_mission_simulation.gif",
                cloud_xyz=cloud_xyz,
                cloud_rgb=cloud_rgb,
                path_smooth=path_smooth,
                mission_wp_path_idx=wp_to_path_smooth,
                start_pt=start_xyz,
                cfg=sim_cfg,
            )
        sim_fpv_cfg = cfg.get("simulation_fpv", {})
        if bool(sim_fpv_cfg.get("enable", True)):
            create_station_simulation_fpv_gif(
                out_gif=out_dir / "station_mission_simulation_fpv.gif",
                cloud_xyz=cloud_xyz,
                path_smooth=path_smooth,
                mission_caps=mission_caps,
                mission_wp_path_idx=wp_to_path_smooth,
                cfg=sim_fpv_cfg,
            )
        timing["output_and_visualization_s"] = float(time.perf_counter() - t0)

    timing["scenario_total_s"] = float(time.perf_counter() - t_s0)
    if write_full_outputs:
        write_json(
            out_dir / "phase3_complexity_and_timing.json",
            {
                "scenario": scenario_name,
                "ordering_strategy": result.get("ordering_strategy"),
                "timing_s": timing,
                "complexity": complexity,
                "planning_mode_counts": seg_modes,
                "astar_nodes_total": int(astar_nodes_total),
                "path_points": result["path_points"],
            },
        )

    return result


def build_output_summary(
    cfg: dict,
    phase1_report: dict,
    primary_result: dict,
    experiment_rows: Sequence[dict],
    phase1_safety_experiment: Optional[dict] = None,
) -> dict:
    free_ratio = None
    try:
        free_ratio = float(phase1_report.get("grid", {}).get("free_voxels", 0)) / float(
            max(1, phase1_report.get("grid", {}).get("total_voxels", 1))
        )
    except Exception:
        free_ratio = None
    summary = {
        "phase": "phase3_station_mission",
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "mission": {
            "primary_scenario": cfg["mission"].get("primary_scenario", "balanced"),
            "ordering_strategy": primary_result.get("ordering_strategy", str(cfg.get("ordering", {}).get("strategy", "region_constrained"))),
            "device_order": primary_result["device_order"],
            "device_reverse_flags": primary_result["device_reverse_flags"],
            "order_match": primary_result["order_match"],
            "return_to_home": bool(cfg["mission"].get("return_to_home", True)),
        },
        "coverage_estimation": {
            "global_retention_ratio": primary_result["global_retention_ratio"],
            "estimated_global_coverage_ratio": primary_result["estimated_global_coverage_ratio"],
            "retained_viewpoints_total": primary_result["retained_viewpoints_total"],
            "raw_viewpoints_total": primary_result["raw_viewpoints_total"],
        },
        "path": {
            "length_raw_m": primary_result["path_length_raw_m"],
            "length_shortcut_m": primary_result["path_length_shortcut_m"],
            "length_smoothed_m": primary_result["path_length_smoothed_m"],
            "segment_success_ratio": primary_result["segment_success_ratio"],
            "estimated_order_cost_m": primary_result["estimated_order_cost_m"],
            "planning_mode_counts": primary_result.get("planning_mode_counts", {}),
            "astar_nodes_total": primary_result.get("astar_nodes_total", 0),
            "path_points": primary_result.get("path_points", {}),
        },
        "trackability": {
            "raw": primary_result["trackability_raw"],
            "shortcut": primary_result.get("trackability_shortcut", {}),
            "smoothed": primary_result["trackability_smoothed"],
        },
        "scene_context": {
            "phase1_rotation_deg": float(math.degrees(float(phase1_report.get("rotation", {}).get("angle_rad", 0.0)))),
            "phase1_free_space_ratio": free_ratio,
            "start_home_xyz_used": primary_result["start_home_xyz_used"],
            "start_home_frame_config": primary_result["start_home_frame_config"],
        },
        "timing_s": primary_result.get("timing", {}),
        "complexity": primary_result.get("complexity", {}),
        "experiments": experiment_rows,
        "phase1_safety_sweep_experiment": phase1_safety_experiment or {"enabled": False},
    }
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Phase 3 station-level mission builder.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("phases/phase3_station_mission/configs/config.phase3.station_mission.json"),
    )
    args = parser.parse_args()

    t0 = time.perf_counter()
    cfg = load_json(args.config)
    root = Path(cfg["paths"]["project_root"]).resolve()
    phase2_out = (root / cfg["paths"]["phase2_output_dir"]).resolve()
    phase1_grid = (root / cfg["paths"]["phase1_station_grid_npz"]).resolve()
    phase1_report_path = (root / cfg["paths"]["phase1_report_json"]).resolve()
    out_dir = (root / cfg["paths"]["output_dir"]).resolve()
    ensure_dir(out_dir)
    ensure_dir(out_dir / "experiments")

    phase1_report = load_json(phase1_report_path)
    station = StationGrid(phase1_grid)
    region_sequence = cfg["mission"]["region_sequence"]
    needed = [str(d) for reg in region_sequence for d in reg.get("devices", [])]
    device_data = load_device_data(phase2_out=phase2_out, needed_devices=needed)

    log(f"Phase3 output dir: {out_dir}")
    log(f"Loaded devices: {len(device_data)}")

    exp_cfg = cfg.get("experiments", {})
    exp_enable = bool(exp_cfg.get("enable", True))
    scenarios = exp_cfg.get("scenarios", [])
    if (not exp_enable) or (not scenarios):
        scenarios = [{"name": cfg["mission"].get("primary_scenario", "default")}]
    primary_name = str(cfg["mission"].get("primary_scenario", scenarios[0]["name"]))
    names = [str(s.get("name", "")) for s in scenarios]
    if primary_name not in names:
        primary_name = names[0]

    experiment_rows: List[dict] = []
    scenario_results: Dict[str, dict] = {}
    primary_result: Optional[dict] = None
    for s in scenarios:
        sname = str(s.get("name", "scenario"))
        scenario_cfg = deep_update(cfg, s)
        log(f"Scenario: {sname}")
        res = run_single_scenario(
            cfg=scenario_cfg,
            station=station,
            phase1_report=phase1_report,
            device_data=device_data,
            region_sequence=region_sequence,
            out_dir=out_dir,
            scenario_name=sname,
            write_full_outputs=(sname == primary_name),
        )
        row = {
            "scenario": sname,
            "retained_viewpoints_total": int(res["retained_viewpoints_total"]),
            "global_retention_ratio": float(res["global_retention_ratio"]),
            "estimated_global_coverage_ratio": float(res["estimated_global_coverage_ratio"]),
            "path_length_raw_m": float(res["path_length_raw_m"]),
            "path_length_shortcut_m": float(res.get("path_length_shortcut_m", res["path_length_raw_m"])),
            "path_length_smoothed_m": float(res["path_length_smoothed_m"]),
            "segment_success_ratio": float(res["segment_success_ratio"]),
            "trackability_raw": float(res["trackability_raw"]["trackability_score_0_100"]),
            "trackability_shortcut": float(res.get("trackability_shortcut", res["trackability_raw"])["trackability_score_0_100"]),
            "trackability_smoothed": float(res["trackability_smoothed"]["trackability_score_0_100"]),
            "max_turn_raw_deg": float(res["trackability_raw"]["max_turn_angle_deg"]),
            "max_turn_shortcut_deg": float(res.get("trackability_shortcut", res["trackability_raw"])["max_turn_angle_deg"]),
            "max_turn_smoothed_deg": float(res["trackability_smoothed"]["max_turn_angle_deg"]),
            "low_clearance_raw_ratio": float(res["trackability_raw"]["low_clearance_ratio"]),
            "low_clearance_shortcut_ratio": float(res.get("trackability_shortcut", res["trackability_raw"])["low_clearance_ratio"]),
            "low_clearance_smoothed_ratio": float(res["trackability_smoothed"]["low_clearance_ratio"]),
            "region_order_match": bool(res["order_match"]["region_order_match"]),
            "strict_device_order_match": bool(res["order_match"]["strict_device_order_match"]),
            "estimated_order_cost_m": float(res["estimated_order_cost_m"]),
            "scenario_total_s": float(res.get("timing", {}).get("scenario_total_s", 0.0)),
            "ordering_strategy": str(res.get("ordering_strategy", cfg.get("ordering", {}).get("strategy", "region_constrained"))),
        }
        experiment_rows.append(row)
        scenario_results[sname] = res
        if sname == primary_name:
            primary_result = res

    if primary_result is None and experiment_rows:
        fallback_name = str(experiment_rows[0]["scenario"])
        log(f"Primary scenario '{primary_name}' not found in results; fallback to '{fallback_name}'")
        scenario_cfg = deep_update(cfg, {"name": fallback_name})
        primary_result = run_single_scenario(
            cfg=scenario_cfg,
            station=station,
            phase1_report=phase1_report,
            device_data=device_data,
            region_sequence=region_sequence,
            out_dir=out_dir,
            scenario_name=fallback_name,
            write_full_outputs=True,
        )
        scenario_results[fallback_name] = primary_result

    save_experiment_csv(out_dir / "experiments" / "phase3_experiment_summary.csv", experiment_rows)
    write_json(out_dir / "experiments" / "phase3_experiment_summary.json", {"rows": experiment_rows})
    core_names = {"conservative", "balanced", "aggressive"}
    rows_core = [r for r in experiment_rows if str(r.get("scenario", "")).lower() in core_names]
    plot_experiment_compare(
        out_dir / "experiments" / "phase3_experiment_compare.png",
        rows_core if rows_core else experiment_rows,
    )

    baseline_res = scenario_results.get(primary_name)
    phase2_style_res = scenario_results.get("phase2_style_local")
    if baseline_res is not None and phase2_style_res is not None:
        gcfg = cfg.get("global_cloud_visualization", {})
        las_path = (root / gcfg.get("station_baseline_las", "pointclouds/substation_baseline.las")).resolve()
        cloud_xyz, cloud_rgb = sample_las_rgb_cloud(
            las_path=las_path,
            max_points=int(gcfg.get("max_points", 120000)),
            chunk_size=int(gcfg.get("chunk_size", 1000000)),
            seed=int(cfg.get("random_seed", 42)) + 707,
            angle_rad=float(phase1_report.get("rotation", {}).get("angle_rad", 0.0)),
        )
        path_left = np.asarray(baseline_res.get("_viz_path_smooth", np.empty((0, 3), dtype=np.float64)), dtype=np.float64)
        path_right = np.asarray(phase2_style_res.get("_viz_path_smooth", np.empty((0, 3), dtype=np.float64)), dtype=np.float64)
        start_pt = np.asarray(baseline_res.get("_viz_start_xyz", cfg["mission"]["start_home_xyz"]), dtype=np.float64)
        visualize_station_rgb_path_comparison(
            out_png=out_dir / "experiments" / "station_mission_rgb_path_compare_phase2_style.png",
            out_gif=out_dir / "experiments" / "station_mission_rgb_path_compare_phase2_style_rotate.gif",
            cloud_xyz=cloud_xyz,
            cloud_rgb=cloud_rgb,
            path_left=path_left,
            path_right=path_right,
            start_pt=start_pt,
            vis_cfg=gcfg,
            left_title=f"Station RGB + Path ({primary_name})",
            right_title="Station RGB + Path (phase2_style_local)",
        )

    phase1_safety_exp = run_phase1_safety_sweep_experiment(
        cfg=cfg,
        phase1_report=phase1_report,
        device_data=device_data,
        region_sequence=region_sequence,
        out_dir=out_dir,
    )

    summary = build_output_summary(
        cfg=cfg,
        phase1_report=phase1_report,
        primary_result=primary_result,
        experiment_rows=experiment_rows,
        phase1_safety_experiment=phase1_safety_exp,
    )
    summary["runtime_s"] = float(time.perf_counter() - t0)
    write_json(out_dir / "phase3_station_mission_summary.json", summary)

    log("Phase3 station mission build complete")
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
