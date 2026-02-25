#!/usr/bin/env python3
"""Phase 2: build single-device inspection viewpoints and local paths.

Pipeline:
1) Discover segmented device LAS files (exclude baseline/origin/buildings by type).
2) Voxelize each device and generate shape-adaptive layered viewpoints.
3) Filter viewpoints by station-level feasible-space map (Phase 1 npz).
4) Reorder viewpoints into a trackable layer-wise route.
5) Evaluate per-device coverage using frustum + voxel occlusion checks.
6) Optionally build single-device local path (direct segment + fallback A* on station grid).
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import time
from matplotlib import animation
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import laspy
import matplotlib.pyplot as plt
import numpy as np


Voxel = Tuple[int, int, int]


def log(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8-sig") as f:
        return json.load(f)


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def infer_device_type(file_stem: str, type_keywords: Dict[str, List[str]]) -> str:
    s = file_stem.lower()
    for t, kws in type_keywords.items():
        if any(k.lower() in s for k in kws):
            return t
    return "default"


def discover_device_las(
    root: Path,
    pointcloud_dir: str,
    baseline_name: str,
    origin_name: str,
    exclude_keywords: List[str],
) -> List[Path]:
    base = (root / pointcloud_dir).resolve()
    files = sorted(base.glob("*.las"))
    out: List[Path] = []
    for p in files:
        name = p.name.lower()
        if name == baseline_name.lower() or name == origin_name.lower():
            continue
        if any(k.lower() in name for k in exclude_keywords):
            continue
        out.append(p)
    return out


@dataclass
class DeviceVoxel:
    name: str
    path: Path
    device_type: str
    voxel_size: float
    min_abs_idx: np.ndarray
    mask: np.ndarray
    surface_mask: np.ndarray
    ground_removed_layers: int = 0
    ground_removed_voxels: int = 0

    @property
    def shape(self) -> Tuple[int, int, int]:
        return tuple(int(v) for v in self.mask.shape)


@dataclass
class Viewpoint:
    vp_id: int
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
    theta: float


def clone_viewpoints(vps: List[Viewpoint]) -> List[Viewpoint]:
    return [Viewpoint(**asdict(v)) for v in vps]


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


def rotate_xy(points: np.ndarray, angle_rad: float) -> np.ndarray:
    if abs(angle_rad) < 1e-12:
        return points
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    out = points.copy()
    x = points[:, 0]
    y = points[:, 1]
    out[:, 0] = c * x - s * y
    out[:, 1] = s * x + c * y
    return out


def voxelize_las(las_path: Path, voxel_size: float, chunk_size: int, angle_rad: float) -> Tuple[np.ndarray, np.ndarray]:
    vox: Set[Voxel] = set()
    with laspy.open(las_path) as reader:
        for pts in reader.chunk_iterator(chunk_size):
            xyz = np.column_stack((pts.x, pts.y, pts.z)).astype(np.float64, copy=False)
            if xyz.shape[0] == 0:
                continue
            xyz = rotate_xy(xyz, angle_rad)
            idx = np.floor(xyz / voxel_size).astype(np.int32)
            uniq = np.unique(idx, axis=0)
            vox.update(map(tuple, uniq.tolist()))
    if not vox:
        return np.zeros((0, 3), dtype=np.int32), np.zeros((0, 3), dtype=np.int32)
    arr = np.array(list(vox), dtype=np.int32)
    mins = arr.min(axis=0).astype(np.int32)
    local = arr - mins[None, :]
    return mins, local


def sample_las_visual_cloud(
    las_path: Path,
    chunk_size: int,
    max_points: int,
    seed: int,
    angle_rad: float,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    if max_points <= 0:
        return np.empty((0, 3), dtype=np.float64), None
    rng = np.random.default_rng(seed)
    xyz_chunks: List[np.ndarray] = []
    rgb_chunks: List[np.ndarray] = []
    has_rgb = False
    with laspy.open(las_path) as reader:
        total = int(reader.header.point_count)
        n_chunks = max(1, int(math.ceil(total / max(1, chunk_size))))
        per_chunk = max(1, int(math.ceil(max_points / n_chunks)))
        for pts in reader.chunk_iterator(chunk_size):
            xyz = np.column_stack((pts.x, pts.y, pts.z)).astype(np.float64, copy=False)
            if xyz.shape[0] == 0:
                continue
            xyz = rotate_xy(xyz, angle_rad)
            take = min(per_chunk, xyz.shape[0])
            pick = rng.choice(xyz.shape[0], size=take, replace=False)
            xyz_chunks.append(xyz[pick])

            if hasattr(pts, "red") and hasattr(pts, "green") and hasattr(pts, "blue"):
                red = np.asarray(pts.red)[pick]
                green = np.asarray(pts.green)[pick]
                blue = np.asarray(pts.blue)[pick]
                max_c = float(
                    max(
                        float(red.max()) if red.size else 0.0,
                        float(green.max()) if green.size else 0.0,
                        float(blue.max()) if blue.size else 0.0,
                        1.0,
                    )
                )
                denom = 65535.0 if max_c > 255.0 else 255.0
                rgb = np.column_stack((red, green, blue)).astype(np.float64) / denom
                rgb_chunks.append(rgb)
                has_rgb = True
    if not xyz_chunks:
        return np.empty((0, 3), dtype=np.float64), None
    xyz = np.vstack(xyz_chunks)
    rgb = np.vstack(rgb_chunks) if has_rgb and rgb_chunks else None
    if xyz.shape[0] > max_points:
        keep = rng.choice(xyz.shape[0], size=max_points, replace=False)
        xyz = xyz[keep]
        if rgb is not None:
            rgb = rgb[keep]
    return xyz, rgb


def build_device_voxel(
    device_name: str,
    device_type: str,
    las_path: Path,
    voxel_size: float,
    chunk_size: int,
    angle_rad: float,
    ground_filter_cfg: Optional[dict] = None,
) -> Optional[DeviceVoxel]:
    min_abs_idx, local = voxelize_las(las_path, voxel_size, chunk_size, angle_rad=angle_rad)
    if local.shape[0] == 0:
        return None
    shape = local.max(axis=0) + 1
    mask = np.zeros(tuple(int(v) for v in shape.tolist()), dtype=np.bool_)
    mask[local[:, 0], local[:, 1], local[:, 2]] = True
    removed_layers = 0
    removed_voxels = 0

    gf = ground_filter_cfg or {}
    if bool(gf.get("enable", False)):
        apply_types = {str(t) for t in gf.get("apply_types", [])}
        apply_names = {str(t).lower() for t in gf.get("apply_device_names", [])}
        if (device_type in apply_types) or (device_name.lower() in apply_names):
            # Optional strict removal for known thin-floor artifacts.
            force_type_layers = gf.get("force_remove_bottom_layers_by_type", {})
            force_name_layers = gf.get("force_remove_bottom_layers_by_name", {})
            force_layers = 0
            if device_type in force_type_layers:
                force_layers = int(force_type_layers[device_type])
            if device_name.lower() in {str(k).lower() for k in force_name_layers.keys()}:
                for k, vv in force_name_layers.items():
                    if str(k).lower() == device_name.lower():
                        force_layers = int(vv)
                        break
            if force_layers > 0:
                nz = np.where(mask.any(axis=(0, 1)))[0]
                if nz.size > 0:
                    rm_layers = nz[: min(force_layers, nz.size)].tolist()
                    rm_n = int(sum(int(mask[:, :, z].sum()) for z in rm_layers))
                    for z in rm_layers:
                        mask[:, :, z] = False
                    removed_layers += len(rm_layers)
                    removed_voxels += rm_n

            max_strip_m = float(gf.get("max_strip_m", 0.8))
            min_ratio = float(gf.get("min_layer_footprint_ratio", 0.72))
            min_remove = int(gf.get("min_removed_voxels", 120))
            max_layers = max(1, int(round(max_strip_m / max(1e-6, voxel_size))))
            sx, sy, sz = mask.shape
            footprint = int(mask.any(axis=2).sum())
            cand_layers: List[int] = []
            for z in range(0, min(sz, max_layers + 1)):
                layer_n = int(mask[:, :, z].sum())
                ratio = float(layer_n / max(1, footprint))
                if ratio >= min_ratio:
                    cand_layers.append(z)
                elif cand_layers:
                    break
            if cand_layers:
                rm_n = int(sum(int(mask[:, :, z].sum()) for z in cand_layers))
                if rm_n >= min_remove:
                    for z in cand_layers:
                        mask[:, :, z] = False
                    removed_layers += len(cand_layers)
                    removed_voxels += rm_n

            # Final stage: world-space bottom trimming at processing stage
            # to eliminate residual thin-floor artifacts before all downstream tasks.
            proc_default = float(gf.get("process_trim_raise_m_default", 0.0))
            proc_by_type = gf.get("process_trim_raise_m_by_type", {})
            proc_by_name = gf.get("process_trim_raise_m_by_name", {})
            proc_raise_m = proc_default
            if device_type in proc_by_type:
                proc_raise_m = float(proc_by_type[device_type])
            for k, v in proc_by_name.items():
                if str(k).lower() == device_name.lower():
                    proc_raise_m = float(v)
                    break
            if proc_raise_m > 1e-6 and bool(mask.any()):
                nz = np.where(mask.any(axis=(0, 1)))[0]
                if nz.size > 0:
                    zc = (float(min_abs_idx[2]) + nz.astype(np.float64) + 0.5) * float(voxel_size)
                    z_base = float(np.min(zc))
                    rm_mask = zc < (z_base + proc_raise_m - 1e-9)
                    rm_layers = nz[rm_mask].tolist()
                    if len(rm_layers) >= int(nz.size):
                        rm_layers = rm_layers[:-1]
                    if rm_layers:
                        rm_n = int(sum(int(mask[:, :, z].sum()) for z in rm_layers))
                        for z in rm_layers:
                            mask[:, :, z] = False
                        removed_layers += len(rm_layers)
                        removed_voxels += rm_n

            # Optional: bottom-band outlier cleanup for specific devices
            # to remove sparse residual ground-adjacent voxels.
            outlier_by_name = gf.get("bottom_outlier_filter_by_name", {})
            out_cfg = None
            for k, v in outlier_by_name.items():
                if str(k).lower() == device_name.lower():
                    out_cfg = v
                    break
            if out_cfg and bool(mask.any()):
                z_band_m = float(out_cfg.get("z_band_m", 1.0))
                min_neighbors = int(out_cfg.get("min_neighbors", 3))
                nz = np.where(mask.any(axis=(0, 1)))[0]
                if nz.size > 0:
                    z0 = int(nz.min())
                    band_layers = max(1, int(math.ceil(z_band_m / max(1e-6, voxel_size))))
                    z1 = min(mask.shape[2] - 1, z0 + band_layers)
                    band = np.zeros_like(mask, dtype=np.bool_)
                    band[:, :, z0 : z1 + 1] = True
                    sx, sy, sz = mask.shape
                    p = np.pad(mask.astype(np.uint8), 1, mode="constant")
                    nb = np.zeros((sx, sy, sz), dtype=np.int16)
                    for dx in (0, 1, 2):
                        for dy in (0, 1, 2):
                            for dz in (0, 1, 2):
                                if dx == 1 and dy == 1 and dz == 1:
                                    continue
                                nb += p[dx : dx + sx, dy : dy + sy, dz : dz + sz]
                    rm = band & mask & (nb < min_neighbors)
                    rm_n = int(rm.sum())
                    if rm_n > 0:
                        rm_layers = int(np.unique(np.where(rm)[2]).size)
                        mask[rm] = False
                        removed_layers += rm_layers
                        removed_voxels += rm_n

    if not bool(mask.any()):
        return None

    p = np.pad(mask, 1, mode="constant", constant_values=False)
    core = p[1:-1, 1:-1, 1:-1]
    all6 = (
        p[2:, 1:-1, 1:-1]
        & p[:-2, 1:-1, 1:-1]
        & p[1:-1, 2:, 1:-1]
        & p[1:-1, :-2, 1:-1]
        & p[1:-1, 1:-1, 2:]
        & p[1:-1, 1:-1, :-2]
    )
    surface = core & (~all6)
    return DeviceVoxel(
        name=device_name,
        path=las_path,
        device_type=device_type,
        voxel_size=voxel_size,
        min_abs_idx=min_abs_idx,
        mask=mask,
        surface_mask=surface,
        ground_removed_layers=int(removed_layers),
        ground_removed_voxels=int(removed_voxels),
    )


def label_components_2d(binary: np.ndarray) -> Tuple[np.ndarray, int]:
    h, w = binary.shape
    labels = np.full((h, w), -1, dtype=np.int32)
    comp_id = 0
    neigh = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for i in range(h):
        for j in range(w):
            if not binary[i, j] or labels[i, j] >= 0:
                continue
            stack = [(i, j)]
            labels[i, j] = comp_id
            while stack:
                x, y = stack.pop()
                for dx, dy in neigh:
                    nx, ny = x + dx, y + dy
                    if nx < 0 or nx >= h or ny < 0 or ny >= w:
                        continue
                    if not binary[nx, ny] or labels[nx, ny] >= 0:
                        continue
                    labels[nx, ny] = comp_id
                    stack.append((nx, ny))
            comp_id += 1
    return labels, comp_id


def boundary_2d(occ2d: np.ndarray) -> np.ndarray:
    p = np.pad(occ2d, 1, mode="constant", constant_values=False)
    core = p[1:-1, 1:-1]
    all4 = p[2:, 1:-1] & p[:-2, 1:-1] & p[1:-1, 2:] & p[1:-1, :-2]
    return core & (~all4)


def compute_outward_dir_2d(occ2d: np.ndarray, ix: int, iy: int, cx: float, cy: float) -> np.ndarray:
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    v = np.zeros((2,), dtype=np.float64)
    h, w = occ2d.shape
    for dx, dy in dirs:
        nx, ny = ix + dx, iy + dy
        if nx < 0 or nx >= h or ny < 0 or ny >= w or not occ2d[nx, ny]:
            vv = np.array([dx, dy], dtype=np.float64)
            nrm = np.linalg.norm(vv)
            if nrm > 1e-9:
                v += vv / nrm
    n = np.linalg.norm(v)
    if n < 1e-9:
        v = np.array([ix - cx, iy - cy], dtype=np.float64)
        n = np.linalg.norm(v)
        if n < 1e-9:
            return np.array([1.0, 0.0], dtype=np.float64)
    return v / n


def order_boundary_points_nn(pts: np.ndarray) -> np.ndarray:
    if pts.shape[0] <= 2:
        return pts
    used = np.zeros((pts.shape[0],), dtype=np.bool_)
    out = np.zeros_like(pts)
    cur = int(np.argmin(pts[:, 0] + pts[:, 1]))
    for i in range(pts.shape[0]):
        out[i] = pts[cur]
        used[cur] = True
        if i == pts.shape[0] - 1:
            break
        rest = np.where(~used)[0]
        d = np.sum((pts[rest] - pts[cur][None, :]) ** 2, axis=1)
        cur = int(rest[int(np.argmin(d))])
    return out


def uniform_sample_closed_polyline(pts: np.ndarray, target_n: int) -> np.ndarray:
    if pts.shape[0] == 0 or target_n <= 0:
        return np.zeros((0, 2), dtype=np.float64)
    if pts.shape[0] <= target_n:
        return pts.astype(np.float64)

    p = pts.astype(np.float64)
    q = np.vstack([p, p[0]])
    seg = np.linalg.norm(q[1:] - q[:-1], axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    if total < 1e-9:
        idx = np.linspace(0, p.shape[0] - 1, num=target_n, endpoint=True).astype(np.int32)
        return p[idx]

    ts = np.linspace(0.0, total, num=target_n, endpoint=False)
    out = np.zeros((target_n, 2), dtype=np.float64)
    j = 0
    for i, t in enumerate(ts):
        while j + 1 < cum.size and cum[j + 1] < t:
            j += 1
        j2 = min(j + 1, p.shape[0])
        a = q[j]
        b = q[j2]
        denom = max(cum[j + 1] - cum[j], 1e-9)
        u = (t - cum[j]) / denom
        out[i] = a * (1.0 - u) + b * u
    return out


def regularize_viewpoint_spacing(
    vps: List[Viewpoint],
    min_spacing_m: float,
    max_spacing_m: float,
    fill_large_gap: bool = True,
    max_add_ratio: float = 0.35,
) -> List[Viewpoint]:
    if len(vps) <= 2:
        return vps
    min_s = max(0.05, float(min_spacing_m))
    max_s = max(min_s * 1.2, float(max_spacing_m))

    groups: Dict[Tuple[int, int], List[Viewpoint]] = {}
    for v in vps:
        groups.setdefault((int(v.layer_k), int(v.component_id)), []).append(v)

    out: List[Viewpoint] = []
    for key in sorted(groups.keys()):
        g = sorted(groups[key], key=lambda t: t.theta)
        if len(g) <= 2:
            out.extend(g)
            continue
        kept = [g[0]]
        for v in g[1:]:
            prev = kept[-1]
            d = math.dist((v.x, v.y, v.z), (prev.x, prev.y, prev.z))
            if d >= min_s:
                kept.append(v)
        if len(kept) >= 2:
            first = kept[0]
            last = kept[-1]
            if math.dist((first.x, first.y, first.z), (last.x, last.y, last.z)) < min_s * 0.8 and len(kept) > 2:
                kept = kept[:-1]

        if (not fill_large_gap) or len(kept) <= 2:
            out.extend(kept)
            continue

        add_cap = int(max(0, round(len(kept) * max(0.0, max_add_ratio))))
        add_cnt = 0
        filled: List[Viewpoint] = []
        for i in range(len(kept)):
            a = kept[i]
            b = kept[(i + 1) % len(kept)]
            filled.append(a)
            d = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
            if d > max_s and add_cnt < add_cap:
                n_add = int(min(add_cap - add_cnt, max(0, d // max_s)))
                for k in range(1, n_add + 1):
                    t = k / (n_add + 1)
                    nv = Viewpoint(
                        vp_id=-1,
                        device=a.device,
                        device_type=a.device_type,
                        x=float(a.x * (1.0 - t) + b.x * t),
                        y=float(a.y * (1.0 - t) + b.y * t),
                        z=float(a.z * (1.0 - t) + b.z * t),
                        yaw_deg=float(a.yaw_deg * (1.0 - t) + b.yaw_deg * t),
                        pitch_deg=float(a.pitch_deg),
                        roll_deg=float(a.roll_deg),
                        target_x=float(a.target_x * (1.0 - t) + b.target_x * t),
                        target_y=float(a.target_y * (1.0 - t) + b.target_y * t),
                        target_z=float(a.target_z * (1.0 - t) + b.target_z * t),
                        layer_k=int(round(a.layer_k * (1.0 - t) + b.layer_k * t)),
                        component_id=a.component_id,
                        theta=float(a.theta * (1.0 - t) + b.theta * t),
                    )
                    filled.append(nv)
                    add_cnt += 1
        out.extend(filled)

    for i, v in enumerate(out):
        v.vp_id = i
    return out


def cap_viewpoints_preserve_layers(vps: List[Viewpoint], cap: int) -> List[Viewpoint]:
    if cap <= 0 or len(vps) <= cap:
        return vps
    by_layer: Dict[int, List[Viewpoint]] = {}
    for v in vps:
        by_layer.setdefault(int(v.layer_k), []).append(v)
    layers = sorted(by_layer.keys())
    sizes = np.array([len(by_layer[k]) for k in layers], dtype=np.float64)
    alloc = np.maximum(1, np.floor((sizes / max(1.0, sizes.sum())) * cap).astype(np.int32))
    while int(alloc.sum()) > cap:
        i = int(np.argmax(alloc))
        if alloc[i] > 1:
            alloc[i] -= 1
        else:
            break
    while int(alloc.sum()) < cap:
        i = int(np.argmax(sizes - alloc))
        alloc[i] += 1

    out: List[Viewpoint] = []
    for k, n_keep in zip(layers, alloc.tolist()):
        g = sorted(by_layer[k], key=lambda t: t.theta)
        if len(g) <= n_keep:
            out.extend(g)
            continue
        idx = np.linspace(0, len(g) - 1, num=int(n_keep), endpoint=True).astype(np.int32)
        out.extend([g[i] for i in idx.tolist()])
    out.sort(key=lambda t: (t.layer_k, t.theta))
    for i, v in enumerate(out):
        v.vp_id = i
    return out


def local_idx_to_world(min_abs_idx: np.ndarray, idx_local: np.ndarray, voxel_size: float) -> np.ndarray:
    arr = np.asarray(idx_local, dtype=np.int32)
    abs_idx = min_abs_idx + arr
    return (abs_idx.astype(np.float64) + 0.5) * voxel_size


def pick_standoff_m(device_name: str, device_type: str, standoff_cfg: dict, phase1_deff_map: Dict[str, float]) -> float:
    s_min = float(standoff_cfg.get("min_m", 2.2))
    s_max = float(standoff_cfg.get("max_m", 6.0))
    margin = float(standoff_cfg.get("boundary_margin_m", 0.2))
    use_deff = bool(standoff_cfg.get("use_phase1_deff", True))
    fallback = float(standoff_cfg.get("fallback_m", 3.5))
    per_type = standoff_cfg.get("per_type_m", {})
    if use_deff and device_name in phase1_deff_map:
        # stay just outside no-fly boundary: deff + margin
        s = float(phase1_deff_map[device_name]) + margin
    elif device_type in per_type:
        s = float(per_type[device_type])
    else:
        s = fallback
    return float(np.clip(s, s_min, s_max))

def generate_viewpoints_for_device(dv: DeviceVoxel, station: StationGrid, cfg: dict, standoff_m: float, seed: int) -> List[Viewpoint]:
    rng = np.random.default_rng(seed)
    cam_cfg = cfg["camera"]
    gen_cfg = cfg["generation"]
    feas_cfg = cfg["feasibility"]
    vfov = math.radians(float(cam_cfg["vfov_deg"]))
    hfov = math.radians(float(cam_cfg["hfov_deg"]))

    overlap_h = float(gen_cfg.get("horizontal_overlap_ratio", 0.7))
    overlap_v = float(gen_cfg.get("vertical_overlap_ratio", 0.65))
    min_layer = float(gen_cfg.get("min_layer_spacing_m", 0.6))
    max_layer = float(gen_cfg.get("max_layer_spacing_m", 3.0))
    layer_window = int(gen_cfg.get("layer_window_vox", 1))
    min_pts_comp = int(gen_cfg.get("min_points_per_component", 4))
    max_per_layer = int(gen_cfg.get("max_viewpoints_per_layer", 120))
    min_arc_step = float(gen_cfg.get("min_arc_step_m", 0.9))
    max_arc_step = float(gen_cfg.get("max_arc_step_m", 2.8))
    dens_cfg = gen_cfg.get("density_regularization", {})
    dens_enable = bool(dens_cfg.get("enable", True))
    dens_min = float(dens_cfg.get("min_spacing_m", 0.9))
    dens_max = float(dens_cfg.get("max_spacing_m", 2.8))
    dens_fill = bool(dens_cfg.get("fill_large_gap", True))
    dens_max_add_ratio = float(dens_cfg.get("max_add_ratio", 0.35))
    name_l = dv.name.lower()
    dens_scale_cfg = gen_cfg.get("density_scale", {})
    dens_scale = float(dens_scale_cfg.get("default", 1.0))
    dens_scale *= float(dens_scale_cfg.get("by_type", {}).get(dv.device_type, 1.0))
    for k, v in dens_scale_cfg.get("by_device", {}).items():
        if str(k).lower() == name_l:
            dens_scale *= float(v)
    layer_scale_cfg = gen_cfg.get("layer_spacing_scale", {})
    layer_scale = float(layer_scale_cfg.get("default", 1.0))
    layer_scale *= float(layer_scale_cfg.get("by_type", {}).get(dv.device_type, 1.0))
    for k, v in layer_scale_cfg.get("by_device", {}).items():
        if str(k).lower() == name_l:
            layer_scale *= float(v)

    z_occ = np.where(dv.mask.any(axis=(0, 1)))[0]
    if z_occ.size == 0:
        return []
    zmin, zmax = int(z_occ.min()), int(z_occ.max())
    v_cov = 2.0 * standoff_m * math.tan(0.5 * vfov)
    layer_step = float(np.clip(v_cov * overlap_v * max(0.2, layer_scale), min_layer, max_layer))
    layer_step_k = max(1, int(round(layer_step / dv.voxel_size)))
    layers = list(range(zmin, zmax + 1, layer_step_k))
    if layers[-1] != zmax:
        layers.append(zmax)

    vps: List[Viewpoint] = []
    vp_id = 0
    for lk in layers:
        k0 = max(zmin, lk - layer_window)
        k1 = min(zmax, lk + layer_window)
        occ2d = dv.mask[:, :, k0 : k1 + 1].any(axis=2)
        if int(occ2d.sum()) < 8:
            continue
        bnd = boundary_2d(occ2d)
        if int(bnd.sum()) < 4:
            continue
        labels, n_comp = label_components_2d(occ2d)
        for comp in range(n_comp):
            comp_occ = occ2d & (labels == comp)
            if int(comp_occ.sum()) < 4:
                continue
            comp_bnd = bnd & comp_occ
            bnd_idx = np.argwhere(comp_bnd)
            if bnd_idx.shape[0] < 3:
                continue
            cx, cy = np.mean(np.argwhere(comp_occ), axis=0)
            perimeter = float(bnd_idx.shape[0] * dv.voxel_size)
            h_cov = 2.0 * standoff_m * math.tan(0.5 * hfov)
            raw_n = int(math.ceil(perimeter / max(h_cov * overlap_h, 1e-6)))
            raw_n = int(max(min_pts_comp, round(raw_n * max(0.2, dens_scale))))
            n_lo = int(max(2, math.ceil(perimeter / max(max_arc_step, 1e-6))))
            n_hi = int(max(2, math.floor(perimeter / max(min_arc_step, 1e-6))))
            target_n = int(np.clip(raw_n, max(min_pts_comp, n_lo), max(min_pts_comp, min(max_per_layer, max(n_hi, min_pts_comp)))))

            ring = order_boundary_points_nn(bnd_idx.astype(np.float64))
            sampled_ring = uniform_sample_closed_polyline(ring, target_n=target_n)
            sampled_ring_i = np.rint(sampled_ring).astype(np.int32)

            for ixy in sampled_ring_i.tolist():
                ix = int(np.clip(ixy[0], 0, occ2d.shape[0] - 1))
                iy = int(np.clip(ixy[1], 0, occ2d.shape[1] - 1))
                if not comp_bnd[ix, iy]:
                    dxy = bnd_idx - np.array([ix, iy], dtype=np.int32)[None, :]
                    j = int(np.argmin(np.sum(dxy * dxy, axis=1)))
                    ix, iy = int(bnd_idx[j, 0]), int(bnd_idx[j, 1])
                theta = float(math.atan2(iy - cy, ix - cx))
                col = np.where(dv.surface_mask[ix, iy, :])[0]
                if col.size == 0:
                    col = np.where(dv.mask[ix, iy, :])[0]
                if col.size == 0:
                    continue
                kz = int(col[np.argmin(np.abs(col - lk))])
                target_local = np.array([ix, iy, kz], dtype=np.int32)
                target_world = local_idx_to_world(dv.min_abs_idx, target_local, dv.voxel_size)
                d2 = compute_outward_dir_2d(occ2d, ix, iy, cx, cy)
                dir3 = np.array([d2[0], d2[1], 0.0], dtype=np.float64)
                vp_world = target_world + dir3 * standoff_m
                if bool(feas_cfg.get("enforce_station_free", True)):
                    if not station.is_free_world(vp_world):
                        extra = float(feas_cfg.get("search_extra_m", 2.0))
                        inward = float(feas_cfg.get("search_inward_m", 0.6))
                        step = max(float(feas_cfg.get("search_step_m", 0.2)), 1e-3)
                        r_min = max(0.4, standoff_m - max(0.0, inward))
                        r_max = standoff_m + max(0.0, extra)
                        radii = [standoff_m]
                        k = 1
                        while True:
                            ru = standoff_m + k * step
                            rd = standoff_m - k * step
                            add_any = False
                            if rd >= r_min - 1e-9:
                                radii.append(rd)
                                add_any = True
                            if ru <= r_max + 1e-9:
                                radii.append(ru)
                                add_any = True
                            if not add_any:
                                break
                            k += 1
                        best_r = None
                        best_err = float("inf")
                        for r in radii:
                            trial = target_world + dir3 * r
                            if station.is_free_world(trial):
                                err = abs(r - standoff_m)
                                if err < best_err:
                                    best_err = err
                                    best_r = r
                                    vp_world = trial
                        if (best_r is None) and (not bool(feas_cfg.get("allow_infeasible_if_none", False))):
                            continue
                yaw = math.degrees(math.atan2(target_world[1] - vp_world[1], target_world[0] - vp_world[0]))
                vps.append(
                    Viewpoint(
                        vp_id=vp_id,
                        device=dv.name,
                        device_type=dv.device_type,
                        x=float(vp_world[0]),
                        y=float(vp_world[1]),
                        z=float(vp_world[2]),
                        yaw_deg=float(yaw),
                        pitch_deg=float(cam_cfg.get("pitch_deg", 0.0)),
                        roll_deg=float(cam_cfg.get("roll_deg", 0.0)),
                        target_x=float(target_world[0]),
                        target_y=float(target_world[1]),
                        target_z=float(target_world[2]),
                        layer_k=int(kz),
                        component_id=int(comp),
                        theta=float(theta),
                    )
                )
                vp_id += 1

    if not vps:
        return []
    keep: List[Viewpoint] = []
    cell = float(gen_cfg.get("dedup_radius_m", 0.45))
    keyset: Set[Tuple[int, int, int]] = set()
    for v in vps:
        k = (int(round(v.x / cell)), int(round(v.y / cell)), int(round(v.z / cell)))
        if k in keyset:
            continue
        keyset.add(k)
        keep.append(v)
    rng.shuffle(keep)
    keep.sort(key=lambda t: (t.layer_k, t.component_id, t.theta))
    if dens_enable:
        keep = regularize_viewpoint_spacing(
            keep,
            min_spacing_m=dens_min,
            max_spacing_m=dens_max,
            fill_large_gap=dens_fill,
            max_add_ratio=dens_max_add_ratio,
        )
    max_by_name = gen_cfg.get("max_viewpoints_per_device_by_name", {})
    max_by_type = gen_cfg.get("max_viewpoints_per_device_by_type", {})
    cap_n = None
    if dv.device_type in max_by_type:
        cap_n = int(max_by_type[dv.device_type])
    if dv.name in max_by_name:
        cap_n = int(max_by_name[dv.name])
    if cap_n is not None:
        keep = cap_viewpoints_preserve_layers(keep, cap=max(4, cap_n))
    for i, v in enumerate(keep):
        v.vp_id = i
    return keep


def _path_length(pts: List[Viewpoint]) -> float:
    if len(pts) <= 1:
        return 0.0
    arr = np.array([[p.x, p.y, p.z] for p in pts], dtype=np.float64)
    d = np.linalg.norm(arr[1:] - arr[:-1], axis=1)
    return float(d.sum())


def tsp_nn_order(arr: np.ndarray, start_idx: int) -> List[int]:
    n = arr.shape[0]
    if n <= 1:
        return [0] if n == 1 else []
    used = np.zeros((n,), dtype=np.bool_)
    order = [int(start_idx)]
    used[int(start_idx)] = True
    for _ in range(n - 1):
        cur = order[-1]
        rest = np.where(~used)[0]
        d = np.linalg.norm(arr[rest] - arr[cur][None, :], axis=1)
        nxt = int(rest[int(np.argmin(d))])
        order.append(nxt)
        used[nxt] = True
    return order


def two_opt_order(order: List[int], arr: np.ndarray, max_iter: int = 1) -> List[int]:
    n = len(order)
    if n < 6:
        return order
    for _ in range(max_iter):
        improved = False
        for i in range(1, n - 2):
            for j in range(i + 1, n - 1):
                a, b = order[i - 1], order[i]
                c, d = order[j], order[j + 1]
                before = float(np.linalg.norm(arr[a] - arr[b]) + np.linalg.norm(arr[c] - arr[d]))
                after = float(np.linalg.norm(arr[a] - arr[c]) + np.linalg.norm(arr[b] - arr[d]))
                if after + 1e-9 < before:
                    order[i : j + 1] = reversed(order[i : j + 1])
                    improved = True
        if not improved:
            break
    return order


def reorder_viewpoints_tsp_global(vps: List[Viewpoint], two_opt_iter: int = 1) -> List[Viewpoint]:
    if len(vps) <= 1:
        return vps
    pts = np.array([[v.x, v.y, v.z] for v in vps], dtype=np.float64)
    start_idx = int(np.argmin(pts[:, 2]))
    order = tsp_nn_order(pts, start_idx=start_idx)
    order = two_opt_order(order, pts, max_iter=max(0, int(two_opt_iter)))
    out = [vps[i] for i in order]
    for i, v in enumerate(out):
        v.vp_id = i
    return out


def reorder_viewpoints(vps: List[Viewpoint], reorder_cfg: Optional[dict] = None) -> List[Viewpoint]:
    if len(vps) <= 1:
        return vps
    reorder_cfg = reorder_cfg or {}
    band_h = float(reorder_cfg.get("band_height_m", 2.5))
    tsp_iter = int(reorder_cfg.get("tsp_2opt_iter", 1))
    max_band_2opt = int(reorder_cfg.get("max_band_points_for_2opt", 240))

    z_all = np.array([v.z for v in vps], dtype=np.float64)
    z0 = float(np.min(z_all))
    by_band: Dict[int, List[Viewpoint]] = {}
    for v in vps:
        bid = int(math.floor((float(v.z) - z0) / max(0.2, band_h)))
        by_band.setdefault(bid, []).append(v)

    band_ids = sorted(by_band.keys())
    out: List[Viewpoint] = []
    prev_pos: Optional[np.ndarray] = None

    for bid in band_ids:
        cur = by_band[bid]
        if len(cur) == 1:
            out.extend(cur)
            prev_pos = np.array([cur[0].x, cur[0].y, cur[0].z], dtype=np.float64)
            continue
        pts = np.array([[p.x, p.y, p.z] for p in cur], dtype=np.float64)
        if prev_pos is None:
            start_idx = int(np.argmin(pts[:, 2]))
        else:
            start_idx = int(np.argmin(np.linalg.norm(pts - prev_pos[None, :], axis=1)))
        order = tsp_nn_order(pts, start_idx=start_idx)
        if len(order) <= max_band_2opt:
            order = two_opt_order(order, pts, max_iter=max(0, tsp_iter))
        seq = [cur[i] for i in order]
        out.extend(seq)
        prev_pos = np.array([seq[-1].x, seq[-1].y, seq[-1].z], dtype=np.float64)

    for i, v in enumerate(out):
        v.vp_id = i
    return out


def point_in_frustum_mask(points: np.ndarray, vp: Viewpoint, hfov_deg: float, vfov_deg: float, near_m: float, far_m: float) -> np.ndarray:
    yaw = math.radians(vp.yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    d = points - np.array([vp.x, vp.y, vp.z], dtype=np.float64)[None, :]
    x_cam = c * d[:, 0] + s * d[:, 1]
    y_cam = -s * d[:, 0] + c * d[:, 1]
    z_cam = d[:, 2]
    tan_h = math.tan(math.radians(hfov_deg) * 0.5)
    tan_v = math.tan(math.radians(vfov_deg) * 0.5)
    m = (x_cam >= near_m) & (x_cam <= far_m)
    m &= np.abs(y_cam) <= (x_cam * tan_h)
    m &= np.abs(z_cam) <= (x_cam * tan_v)
    return m


def is_occluded_by_device(dv: DeviceVoxel, vp_world: np.ndarray, pt_world: np.ndarray, step_voxel: float = 0.6) -> bool:
    vs = dv.voxel_size
    p0 = (vp_world / vs) - dv.min_abs_idx.astype(np.float64)
    p1 = (pt_world / vs) - dv.min_abs_idx.astype(np.float64)
    delta = p1 - p0
    n = max(2, int(math.ceil(float(np.max(np.abs(delta))) / max(step_voxel, 1e-6))))
    sx, sy, sz = dv.mask.shape
    for i in range(1, n):
        t = i / n
        if t > 0.96:
            break
        pp = p0 * (1.0 - t) + p1 * t
        ix, iy, iz = int(math.floor(pp[0])), int(math.floor(pp[1])), int(math.floor(pp[2]))
        if ix < 0 or iy < 0 or iz < 0 or ix >= sx or iy >= sy or iz >= sz:
            continue
        if dv.mask[ix, iy, iz]:
            return True
    return False

def evaluate_coverage(dv: DeviceVoxel, ordered_vps: List[Viewpoint], cfg: dict, seed: int, return_details: bool = False) -> dict:
    cov_cfg = cfg["coverage"]
    if not bool(cov_cfg.get("enable", True)) or not ordered_vps:
        return {
            "enabled": False,
            "sampled_surface_points": 0,
            "covered_surface_points": 0,
            "coverage_ratio": 0.0,
            "surface_world": np.empty((0, 3), dtype=np.float64),
            "covered_mask": np.zeros((0,), dtype=np.bool_),
        }

    rng = np.random.default_rng(seed)
    surf_idx = np.argwhere(dv.surface_mask).astype(np.int32)
    n_sample = int(cov_cfg.get("sample_surface_points", 3000))
    if surf_idx.shape[0] > n_sample:
        pick = rng.choice(surf_idx.shape[0], size=n_sample, replace=False)
        surf_idx = surf_idx[pick]
    surface_world = local_idx_to_world(dv.min_abs_idx, surf_idx, dv.voxel_size)
    covered = np.zeros((surface_world.shape[0],), dtype=np.bool_)

    max_checks = int(cov_cfg.get("max_points_test_per_viewpoint", 800))
    step_vox = float(cov_cfg.get("occlusion_step_voxel", 0.6))
    cam = cfg["camera"]
    for vp in ordered_vps:
        if covered.all():
            break
        m = point_in_frustum_mask(
            surface_world,
            vp,
            hfov_deg=float(cam["hfov_deg"]),
            vfov_deg=float(cam["vfov_deg"]),
            near_m=float(cam.get("near_m", 0.3)),
            far_m=float(cam.get("far_m", 30.0)),
        )
        cand = np.where(m & (~covered))[0]
        if cand.size == 0:
            continue
        if cand.size > max_checks:
            d = np.linalg.norm(surface_world[cand] - np.array([vp.x, vp.y, vp.z])[None, :], axis=1)
            order = np.argsort(d)
            cand = cand[order[:max_checks]]
        vpw = np.array([vp.x, vp.y, vp.z], dtype=np.float64)
        for idx in cand.tolist():
            if not is_occluded_by_device(dv, vpw, surface_world[idx], step_voxel=step_vox):
                covered[idx] = True

    cov = float(covered.mean()) if covered.size else 0.0
    out = {
        "enabled": True,
        "sampled_surface_points": int(covered.size),
        "covered_surface_points": int(covered.sum()),
        "coverage_ratio": cov,
        "surface_world": surface_world if return_details else np.empty((0, 3), dtype=np.float64),
        "covered_mask": covered if return_details else np.zeros((0,), dtype=np.bool_),
    }
    return out


def build_single_device_path(ordered_vps: List[Viewpoint], station: StationGrid, cfg: dict) -> dict:
    plan_cfg = cfg["planning"]
    if not bool(plan_cfg.get("enable_single_device_path_planning", True)) or len(ordered_vps) <= 1:
        pts = np.array([[v.x, v.y, v.z] for v in ordered_vps], dtype=np.float64)
        return {
            "enabled": False,
            "path_points": pts,
            "segments": [],
            "total_length_m": float(np.linalg.norm(pts[1:] - pts[:-1], axis=1).sum()) if pts.shape[0] > 1 else 0.0,
        }

    con = int(plan_cfg.get("voxel_connectivity", 26))
    roi_margin = float(plan_cfg.get("astar_roi_margin_m", 8.0))
    max_exp = int(plan_cfg.get("astar_max_expansions", 180000))

    out_pts: List[np.ndarray] = [np.array([ordered_vps[0].x, ordered_vps[0].y, ordered_vps[0].z], dtype=np.float64)]
    segs = []
    seg_lengths: List[float] = []
    ok_cnt = 0
    for i in range(len(ordered_vps) - 1):
        a = np.array([ordered_vps[i].x, ordered_vps[i].y, ordered_vps[i].z], dtype=np.float64)
        b = np.array([ordered_vps[i + 1].x, ordered_vps[i + 1].y, ordered_vps[i + 1].z], dtype=np.float64)
        if station.line_free_world(a, b):
            seg_len = float(np.linalg.norm(b - a))
            out_pts.append(b)
            segs.append({"i": i, "mode": "straight", "ok": True, "length_m": seg_len})
            seg_lengths.append(seg_len)
            ok_cnt += 1
            continue
        astar_path = station.astar_world(start_w=a, goal_w=b, margin_m=roi_margin, connectivity=con, max_expansions=max_exp)
        if astar_path is not None and astar_path.shape[0] >= 2:
            seg_len = float(np.linalg.norm(astar_path[1:] - astar_path[:-1], axis=1).sum())
            for p in astar_path[1:]:
                out_pts.append(p.astype(np.float64))
            segs.append({"i": i, "mode": "astar", "ok": True, "nodes": int(astar_path.shape[0]), "length_m": seg_len})
            seg_lengths.append(seg_len)
            ok_cnt += 1
        else:
            seg_len = float(np.linalg.norm(b - a))
            out_pts.append(b)
            segs.append({"i": i, "mode": "fallback_direct", "ok": False, "length_m": seg_len})
            seg_lengths.append(seg_len)

    path = np.vstack(out_pts) if out_pts else np.empty((0, 3), dtype=np.float64)
    total_len = float(np.linalg.norm(path[1:] - path[:-1], axis=1).sum()) if path.shape[0] > 1 else 0.0
    seg_arr = np.array(seg_lengths, dtype=np.float64) if seg_lengths else np.empty((0,), dtype=np.float64)
    return {
        "enabled": True,
        "path_points": path,
        "segments": segs,
        "segment_success_ratio": float(ok_cnt / max(1, len(segs))),
        "total_length_m": total_len,
        "avg_segment_length_m": float(seg_arr.mean()) if seg_arr.size else 0.0,
        "min_segment_length_m": float(seg_arr.min()) if seg_arr.size else 0.0,
        "max_segment_length_m": float(seg_arr.max()) if seg_arr.size else 0.0,
    }


def write_capture_csv(path: Path, vps: List[Viewpoint]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["viewpoint_id", "device", "device_type", "role", "x", "y", "z", "yaw_deg", "pitch_deg", "roll_deg", "target_x", "target_y", "target_z", "layer_k", "component_id"])
        for v in vps:
            w.writerow([v.vp_id, v.device, v.device_type, "inspect_capture", f"{v.x:.4f}", f"{v.y:.4f}", f"{v.z:.4f}", f"{v.yaw_deg:.2f}", f"{v.pitch_deg:.2f}", f"{v.roll_deg:.2f}", f"{v.target_x:.4f}", f"{v.target_y:.4f}", f"{v.target_z:.4f}", v.layer_k, v.component_id])


def write_nav_csv(path: Path, vps: List[Viewpoint]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["waypoint_id", "x", "y", "z", "source_viewpoint_id"])
        for i, v in enumerate(vps):
            w.writerow([i, f"{v.x:.4f}", f"{v.y:.4f}", f"{v.z:.4f}", v.vp_id])


def write_path_csv(path: Path, path_pts: np.ndarray) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["path_id", "x", "y", "z"])
        for i, p in enumerate(path_pts.tolist()):
            w.writerow([i, f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}"])


def simplify_path_for_display(path_pts: np.ndarray, min_step_m: float, max_points: int) -> np.ndarray:
    if path_pts.shape[0] <= 2:
        return path_pts
    keep = [path_pts[0]]
    last = path_pts[0]
    min_step = max(1e-3, float(min_step_m))
    for i in range(1, path_pts.shape[0] - 1):
        p = path_pts[i]
        if float(np.linalg.norm(p - last)) >= min_step:
            keep.append(p)
            last = p
    keep.append(path_pts[-1])
    out = np.vstack(keep)
    if out.shape[0] > max_points:
        idx = np.linspace(0, out.shape[0] - 1, num=max_points, endpoint=True).astype(np.int32)
        out = out[idx]
    return out


def smooth_path_for_display(path_pts: np.ndarray, window_size: int) -> np.ndarray:
    if path_pts.shape[0] <= 2 or window_size <= 1:
        return path_pts
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


def trim_visual_ground_points(
    dv: DeviceVoxel,
    device_name: str,
    device_type: str,
    xyz: Optional[np.ndarray],
    rgb: Optional[np.ndarray],
    ground_filter_cfg: Optional[dict],
    purpose: str = "plan",
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    if xyz is None or xyz.shape[0] == 0:
        return xyz, rgb
    gf = ground_filter_cfg or {}
    if not bool(gf.get("enable", False)):
        return xyz, rgb
    apply_types = {str(t) for t in gf.get("apply_types", [])}
    apply_names = {str(t).lower() for t in gf.get("apply_device_names", [])}
    if (device_type not in apply_types) and (device_name.lower() not in apply_names):
        return xyz, rgb
    if not bool(gf.get("visual_trim_enable", True)):
        return xyz, rgb

    nz = np.where(dv.mask.any(axis=(0, 1)))[0]
    if nz.size == 0:
        return xyz, rgb
    k_min = int(nz.min())
    z_min = float((float(dv.min_abs_idx[2]) + float(k_min) + 0.5) * float(dv.voxel_size))
    extra = float(gf.get("visual_trim_below_min_z_m", 0.05))
    raise_default = float(gf.get("visual_trim_raise_m_default", 0.0))
    raise_by_type = gf.get("visual_trim_raise_m_by_type", {})
    raise_by_name = gf.get("visual_trim_raise_m_by_name", {})
    raise_m = raise_default
    if device_type in raise_by_type:
        raise_m = float(raise_by_type[device_type])
    dev_l = device_name.lower()
    for k, v in raise_by_name.items():
        if str(k).lower() == dev_l:
            raise_m = float(v)
            break
    if purpose == "coverage":
        extra_default = float(gf.get("visual_trim_raise_m_coverage_extra_default", 0.0))
        extra_by_type = gf.get("visual_trim_raise_m_coverage_extra_by_type", {})
        extra_by_name = gf.get("visual_trim_raise_m_coverage_extra_by_name", {})
        extra_m = extra_default
        if device_type in extra_by_type:
            extra_m = float(extra_by_type[device_type])
        for k, v in extra_by_name.items():
            if str(k).lower() == dev_l:
                extra_m = float(v)
                break
        raise_m += extra_m
    z_cut = z_min - extra + raise_m
    keep = xyz[:, 2] >= z_cut
    if not np.any(keep):
        return xyz, rgb
    xyz2 = xyz[keep]
    rgb2 = rgb[keep] if (rgb is not None and rgb.shape[0] == xyz.shape[0]) else rgb
    return xyz2, rgb2


def visualize_device_plan(
    dv: DeviceVoxel,
    ordered_vps: List[Viewpoint],
    path_pts: np.ndarray,
    out_png: Path,
    max_surface_plot_points: int,
    seed: int,
    base_cloud_xyz: Optional[np.ndarray] = None,
    base_cloud_rgb: Optional[np.ndarray] = None,
    path_display_min_step_m: float = 0.6,
    path_display_max_points: int = 1800,
    path_display_smooth_window: int = 5,
    preserve_all_path_points: bool = False,
) -> np.ndarray:
    rng = np.random.default_rng(seed)
    surf = np.argwhere(dv.surface_mask).astype(np.int32)
    if surf.shape[0] > max_surface_plot_points:
        pick = rng.choice(surf.shape[0], size=max_surface_plot_points, replace=False)
        surf = surf[pick]
    surf_w = local_idx_to_world(dv.min_abs_idx, surf, dv.voxel_size)
    if preserve_all_path_points:
        path_plot = path_pts.copy()
    else:
        path_plot = simplify_path_for_display(path_pts, min_step_m=path_display_min_step_m, max_points=path_display_max_points)
        path_plot = smooth_path_for_display(path_plot, window_size=path_display_smooth_window)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    if base_cloud_xyz is not None and base_cloud_xyz.shape[0] > 0:
        if base_cloud_rgb is not None and base_cloud_rgb.shape[0] == base_cloud_xyz.shape[0]:
            ax.scatter(
                base_cloud_xyz[:, 0],
                base_cloud_xyz[:, 1],
                base_cloud_xyz[:, 2],
                c=base_cloud_rgb,
                s=0.8,
                alpha=0.24,
                depthshade=False,
            )
        else:
            ax.scatter(base_cloud_xyz[:, 0], base_cloud_xyz[:, 1], base_cloud_xyz[:, 2], c="silver", s=0.8, alpha=0.24, depthshade=False)
    elif surf_w.shape[0] > 0:
        ax.scatter(surf_w[:, 0], surf_w[:, 1], surf_w[:, 2], c="lightgray", s=0.8, alpha=0.28, depthshade=False)
    if ordered_vps:
        vp_arr = np.array([[v.x, v.y, v.z] for v in ordered_vps], dtype=np.float64)
        ax.scatter(vp_arr[:, 0], vp_arr[:, 1], vp_arr[:, 2], c="red", s=11.0, alpha=0.85, depthshade=False)
    if path_plot.shape[0] > 1:
        ax.plot(path_plot[:, 0], path_plot[:, 1], path_plot[:, 2], c="deepskyblue", linewidth=1.8, alpha=0.9)
    ax.set_title(f"{dv.name}: viewpoints + local path")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.view_init(elev=22.0, azim=25.0)
    fig.tight_layout()
    fig.savefig(out_png, dpi=220)
    plt.close(fig)
    return path_plot


def visualize_device_plan_gif(
    dv: DeviceVoxel,
    ordered_vps: List[Viewpoint],
    path_pts: np.ndarray,
    out_gif: Path,
    base_cloud_xyz: Optional[np.ndarray],
    base_cloud_rgb: Optional[np.ndarray],
    gif_frames: int,
    gif_fps: int,
) -> None:
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    if base_cloud_xyz is not None and base_cloud_xyz.shape[0] > 0:
        if base_cloud_rgb is not None and base_cloud_rgb.shape[0] == base_cloud_xyz.shape[0]:
            ax.scatter(
                base_cloud_xyz[:, 0],
                base_cloud_xyz[:, 1],
                base_cloud_xyz[:, 2],
                c=base_cloud_rgb,
                s=0.8,
                alpha=0.24,
                depthshade=False,
            )
        else:
            ax.scatter(base_cloud_xyz[:, 0], base_cloud_xyz[:, 1], base_cloud_xyz[:, 2], c="silver", s=0.8, alpha=0.24, depthshade=False)
    if ordered_vps:
        vp_arr = np.array([[v.x, v.y, v.z] for v in ordered_vps], dtype=np.float64)
        ax.scatter(vp_arr[:, 0], vp_arr[:, 1], vp_arr[:, 2], c="red", s=11.0, alpha=0.85, depthshade=False)
    if path_pts.shape[0] > 1:
        ax.plot(path_pts[:, 0], path_pts[:, 1], path_pts[:, 2], c="deepskyblue", linewidth=1.8, alpha=0.9)
    ax.set_title(f"{dv.name}: viewpoints + local path (rotate)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.view_init(elev=22.0, azim=25.0)
    fig.tight_layout()

    frames = max(16, int(gif_frames))
    azims = np.linspace(25.0, 385.0, frames, endpoint=False)

    def _update(i: int):
        ax.view_init(elev=22.0, azim=float(azims[i]))
        return []

    ani = animation.FuncAnimation(fig, _update, frames=frames, interval=max(1, int(1000 / max(1, gif_fps))), blit=False)
    try:
        writer = animation.PillowWriter(fps=max(1, int(gif_fps)))
        ani.save(out_gif, writer=writer, dpi=120)
    except Exception as e:
        log(f"[warn] device plan gif save failed for {dv.name}: {e}")
    plt.close(fig)


def visualize_generation_diagnostics(
    dv: DeviceVoxel,
    raw_vps: List[Viewpoint],
    ordered_vps: List[Viewpoint],
    out_png: Path,
) -> None:
    if not raw_vps:
        return
    raw_arr = np.array([[v.x, v.y, v.z, v.target_x, v.target_y, v.target_z, v.layer_k] for v in raw_vps], dtype=np.float64)
    ord_arr = np.array([[v.x, v.y, v.z, v.layer_k] for v in ordered_vps], dtype=np.float64) if ordered_vps else np.empty((0, 4), dtype=np.float64)

    z_occ = np.where(dv.mask.any(axis=(0, 1)))[0].astype(np.int32)
    occ_counts = np.array([int(dv.mask[:, :, k].sum()) for k in z_occ], dtype=np.int32)
    vp_layers = raw_arr[:, 6].astype(np.int32)
    uniq_layers, vp_counts = np.unique(vp_layers, return_counts=True)
    dists = np.linalg.norm(raw_arr[:, :3] - raw_arr[:, 3:6], axis=1)

    fig, axs = plt.subplots(2, 2, figsize=(12, 9))

    sc = axs[0, 0].scatter(raw_arr[:, 0], raw_arr[:, 1], c=raw_arr[:, 6], s=10, cmap="viridis", alpha=0.75)
    axs[0, 0].set_title("Generated viewpoints (XY, color=layer)")
    axs[0, 0].set_xlabel("x")
    axs[0, 0].set_ylabel("y")
    cb = fig.colorbar(sc, ax=axs[0, 0], fraction=0.045, pad=0.04)
    cb.set_label("layer_k")

    axs[0, 1].plot(z_occ.tolist(), occ_counts.tolist(), color="dimgray", linewidth=1.4, label="occupied voxels per z")
    axs[0, 1].plot(uniq_layers.tolist(), vp_counts.tolist(), color="tab:red", linewidth=1.4, marker="o", markersize=3, label="viewpoints per layer")
    axs[0, 1].set_title("Layer occupancy vs sampled viewpoints")
    axs[0, 1].set_xlabel("layer_k")
    axs[0, 1].set_ylabel("count")
    axs[0, 1].legend(loc="upper right")

    axs[1, 0].hist(dists.tolist(), bins=16, color="tab:blue", alpha=0.8, edgecolor="white")
    axs[1, 0].set_title("Viewpoint standoff distribution")
    axs[1, 0].set_xlabel("distance viewpoint->target (m)")
    axs[1, 0].set_ylabel("count")

    if ord_arr.shape[0] > 0:
        axs[1, 1].plot(ord_arr[:, 0], ord_arr[:, 1], color="deepskyblue", linewidth=1.1, alpha=0.8)
        axs[1, 1].scatter(ord_arr[:, 0], ord_arr[:, 1], c=ord_arr[:, 3], s=7, cmap="plasma", alpha=0.8)
    axs[1, 1].set_title("Ordered traversal preview (XY)")
    axs[1, 1].set_xlabel("x")
    axs[1, 1].set_ylabel("y")

    fig.suptitle(f"{dv.name}: viewpoint generation diagnostics", y=0.98)
    fig.tight_layout()
    fig.savefig(out_png, dpi=220)
    plt.close(fig)


def visualize_coverage_map(
    device_name: str,
    ordered_vps: List[Viewpoint],
    base_cloud_xyz: Optional[np.ndarray],
    base_cloud_rgb: Optional[np.ndarray],
    surface_world: np.ndarray,
    covered_mask: np.ndarray,
    out_png: Path,
    out_gif: Path,
    max_points: int,
    gif_frames: int,
    gif_fps: int,
    seed: int,
) -> None:
    rng = np.random.default_rng(seed)
    surf = surface_world
    cov = covered_mask
    if surf.shape[0] > max_points:
        pick = rng.choice(surf.shape[0], size=max_points, replace=False)
        surf = surf[pick]
        cov = cov[pick]

    covered_pts = surf[cov] if surf.size else np.empty((0, 3), dtype=np.float64)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    if base_cloud_xyz is not None and base_cloud_xyz.shape[0] > 0:
        ax.scatter(base_cloud_xyz[:, 0], base_cloud_xyz[:, 1], base_cloud_xyz[:, 2], c="black", s=0.55, alpha=0.26, depthshade=False)
    if covered_pts.shape[0] > 0:
        ax.scatter(covered_pts[:, 0], covered_pts[:, 1], covered_pts[:, 2], c="#20c463", s=3.2, alpha=0.9, depthshade=False)
    ax.set_title(f"{device_name}: covered surface region")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.view_init(elev=22.0, azim=25.0)
    fig.tight_layout()
    fig.savefig(out_png, dpi=220)

    frames = max(16, int(gif_frames))
    azims = np.linspace(25.0, 385.0, frames, endpoint=False)

    def _update(i: int):
        ax.view_init(elev=22.0, azim=float(azims[i]))
        return []

    ani = animation.FuncAnimation(fig, _update, frames=frames, interval=max(1, int(1000 / max(1, gif_fps))), blit=False)
    try:
        writer = animation.PillowWriter(fps=max(1, int(gif_fps)))
        ani.save(out_gif, writer=writer, dpi=120)
    except Exception as e:
        log(f"[warn] coverage gif save failed for {device_name}: {e}")
    plt.close(fig)


def visualize_reorder_baseline_compare(
    device_name: str,
    base_cloud_xyz: Optional[np.ndarray],
    base_cloud_rgb: Optional[np.ndarray],
    ours_vps: List[Viewpoint],
    ours_path_plot: np.ndarray,
    baseline_vps: List[Viewpoint],
    baseline_path_plot: np.ndarray,
    out_png: Path,
) -> None:
    if (not ours_vps) or (not baseline_vps):
        return
    fig = plt.figure(figsize=(16, 7))
    axs = [fig.add_subplot(1, 2, 1, projection="3d"), fig.add_subplot(1, 2, 2, projection="3d")]
    for ax, title, vps, pth in [
        (axs[0], "Proposed method", ours_vps, ours_path_plot),
        (axs[1], "Baseline (global TSP-NN + 2-opt)", baseline_vps, baseline_path_plot),
    ]:
        if base_cloud_xyz is not None and base_cloud_xyz.shape[0] > 0:
            if base_cloud_rgb is not None and base_cloud_rgb.shape[0] == base_cloud_xyz.shape[0]:
                ax.scatter(
                    base_cloud_xyz[:, 0],
                    base_cloud_xyz[:, 1],
                    base_cloud_xyz[:, 2],
                    c=base_cloud_rgb,
                    s=0.8,
                    alpha=0.24,
                    depthshade=False,
                )
            else:
                ax.scatter(base_cloud_xyz[:, 0], base_cloud_xyz[:, 1], base_cloud_xyz[:, 2], c="silver", s=0.8, alpha=0.24, depthshade=False)
        vp_arr = np.array([[v.x, v.y, v.z] for v in vps], dtype=np.float64)
        ax.scatter(vp_arr[:, 0], vp_arr[:, 1], vp_arr[:, 2], c="red", s=11.0, alpha=0.85, depthshade=False)
        if pth is not None and pth.shape[0] > 1:
            ax.plot(pth[:, 0], pth[:, 1], pth[:, 2], c="deepskyblue", linewidth=1.8, alpha=0.9)
        ax.set_title(title)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.view_init(elev=22.0, azim=25.0)
    fig.suptitle(f"{device_name}: reorder baseline compare")
    fig.tight_layout()
    fig.savefig(out_png, dpi=220)
    plt.close(fig)


def load_phase1_deff_map(path: Optional[Path]) -> Dict[str, float]:
    if path is None or (not path.exists()):
        return {}
    try:
        rep = load_json(path)
    except Exception:
        return {}
    out: Dict[str, float] = {}
    for d in rep.get("safety", {}).get("devices", []):
        name = str(d.get("name", "")).strip()
        if not name:
            continue
        out[name] = float(d.get("d_eff", 0.0))
    return out


def load_phase1_rotation_angle(path: Optional[Path]) -> float:
    if path is None or (not path.exists()):
        return 0.0
    try:
        rep = load_json(path)
    except Exception:
        return 0.0
    return float(rep.get("rotation", {}).get("angle_rad", 0.0))

def main() -> None:
    parser = argparse.ArgumentParser(description="Phase 2 single-device viewpoint generation and evaluation.")
    parser.add_argument("--config", type=Path, default=Path("phases/phase2_single_device/configs/config.phase2.single_device.json"))
    args = parser.parse_args()

    cfg = load_json(args.config)
    root = Path(cfg["paths"]["project_root"]).resolve()
    out_dir = (root / cfg["paths"]["output_dir"]).resolve()
    ensure_dir(out_dir)

    station_npz = (root / cfg["paths"]["station_grid_npz"]).resolve()
    station = StationGrid(station_npz)
    phase1_report = (root / cfg["paths"]["phase1_report_json"]).resolve()
    deff_map = load_phase1_deff_map(phase1_report)
    angle_rad = load_phase1_rotation_angle(phase1_report)
    log(f"Output dir: {out_dir}")
    log(f"Station grid: {station_npz}")
    log(f"Use rotation angle from phase1 report: {math.degrees(angle_rad):.3f} deg")

    device_files = discover_device_las(
        root=root,
        pointcloud_dir=str(cfg["paths"]["pointcloud_dir"]),
        baseline_name=str(cfg["paths"]["baseline_name"]),
        origin_name=str(cfg["paths"]["origin_name"]),
        exclude_keywords=[str(x) for x in cfg.get("filters", {}).get("exclude_name_keywords", [])],
    )
    type_keywords = cfg["device_type_keywords"]
    exclude_types = {str(t) for t in cfg.get("filters", {}).get("exclude_types", [])}
    include_keywords = [str(k).lower() for k in cfg.get("filters", {}).get("include_name_keywords", [])]

    selected: List[Tuple[Path, str]] = []
    for p in device_files:
        dt = infer_device_type(p.stem, type_keywords)
        if dt in exclude_types:
            continue
        if include_keywords and not any(k in p.stem.lower() for k in include_keywords):
            continue
        selected.append((p, dt))
    log(f"Selected device models: {len(selected)}")

    summary_rows = []
    total_cov_n = 0
    total_cov_ok = 0

    for i, (las_path, device_type) in enumerate(selected, start=1):
        t_dev0 = time.perf_counter()
        name = las_path.stem
        log(f"Device {i}/{len(selected)}: {name} ({device_type})")
        dv = build_device_voxel(
            device_name=name,
            device_type=device_type,
            las_path=las_path,
            voxel_size=float(cfg["generation"]["voxel_size_m"]),
            chunk_size=int(cfg["io"]["chunk_size"]),
            angle_rad=angle_rad,
            ground_filter_cfg=cfg.get("ground_filter", {}),
        )
        if dv is None:
            log(f"  skip {name}: empty voxel result")
            continue

        standoff = pick_standoff_m(name, device_type, cfg["generation"]["standoff"], deff_map)
        t_gen0 = time.perf_counter()
        raw_vps = generate_viewpoints_for_device(dv=dv, station=station, cfg=cfg, standoff_m=standoff, seed=int(cfg["random_seed"]) + i * 11)
        t_reorder0 = time.perf_counter()
        ordered_vps = reorder_viewpoints(raw_vps, reorder_cfg=cfg.get("reorder", {}))
        t_reorder = float(time.perf_counter() - t_reorder0)
        t_gen = float(time.perf_counter() - t_gen0)
        vp_cnt = max(1, len(ordered_vps))
        t_per_vp_ms = float((t_gen / vp_cnt) * 1000.0)

        t_cov0 = time.perf_counter()
        cov = evaluate_coverage(dv=dv, ordered_vps=ordered_vps, cfg=cfg, seed=int(cfg["random_seed"]) + i * 23, return_details=True)
        t_cov = float(time.perf_counter() - t_cov0)
        t_plan0 = time.perf_counter()
        path_res = build_single_device_path(ordered_vps=ordered_vps, station=station, cfg=cfg)
        t_plan = float(time.perf_counter() - t_plan0)
        n_pairs = max(1, len(ordered_vps) - 1)
        t_plan_pair_ms = float((t_plan / n_pairs) * 1000.0)

        vis_cfg = cfg.get("visualization", {})
        cov_vis_cfg = cfg.get("coverage_visualization", {})
        base_cloud_n = int(max(vis_cfg.get("max_base_cloud_plot_points", 24000), cov_vis_cfg.get("max_base_cloud_plot_points", 55000)))
        base_cloud_xyz, base_cloud_rgb = sample_las_visual_cloud(
            las_path=las_path,
            chunk_size=int(cfg["io"]["chunk_size"]),
            max_points=base_cloud_n,
            seed=int(cfg["random_seed"]) + i * 29,
            angle_rad=angle_rad,
        )
        base_cloud_plan_xyz, base_cloud_plan_rgb = trim_visual_ground_points(
            dv=dv,
            device_name=name,
            device_type=device_type,
            xyz=base_cloud_xyz,
            rgb=base_cloud_rgb,
            ground_filter_cfg=cfg.get("ground_filter", {}),
            purpose="plan",
        )
        base_cloud_cov_xyz, base_cloud_cov_rgb = trim_visual_ground_points(
            dv=dv,
            device_name=name,
            device_type=device_type,
            xyz=base_cloud_xyz,
            rgb=base_cloud_rgb,
            ground_filter_cfg=cfg.get("ground_filter", {}),
            purpose="coverage",
        )

        dev_dir = out_dir / name
        ensure_dir(dev_dir)
        write_capture_csv(dev_dir / "viewpoints_capture_raw.csv", raw_vps)
        write_capture_csv(dev_dir / "viewpoints_capture_ordered.csv", ordered_vps)
        write_nav_csv(dev_dir / "viewpoints_nav_ordered.csv", ordered_vps)
        write_path_csv(dev_dir / "path_waypoints.csv", path_res["path_points"])
        display_source = str(vis_cfg.get("display_path_source", "viewpoint_order"))
        path_for_plot = path_res["path_points"]
        if display_source == "viewpoint_order" and ordered_vps:
            path_for_plot = np.array([[v.x, v.y, v.z] for v in ordered_vps], dtype=np.float64)
        preserve_all_plot_path = bool(vis_cfg.get("display_preserve_waypoints", True)) and display_source == "viewpoint_order"
        path_plot = visualize_device_plan(
            dv=dv,
            ordered_vps=ordered_vps,
            path_pts=path_for_plot,
            out_png=dev_dir / "device_plan.png",
            max_surface_plot_points=int(cfg["visualization"]["max_surface_plot_points"]),
            seed=int(cfg["random_seed"]) + i * 31,
            base_cloud_xyz=base_cloud_plan_xyz,
            base_cloud_rgb=base_cloud_plan_rgb,
            path_display_min_step_m=float(vis_cfg.get("path_display_min_step_m", 0.6)),
            path_display_max_points=int(vis_cfg.get("path_display_max_points", 1800)),
            path_display_smooth_window=int(vis_cfg.get("path_display_smooth_window", 5)),
            preserve_all_path_points=preserve_all_plot_path,
        )
        plan_gif_cfg = cfg.get("visualization", {})
        if bool(plan_gif_cfg.get("device_plan_gif_enable", True)):
            visualize_device_plan_gif(
                dv=dv,
                ordered_vps=ordered_vps,
                path_pts=path_plot,
                out_gif=dev_dir / "device_plan_rotate.gif",
                base_cloud_xyz=base_cloud_plan_xyz,
                base_cloud_rgb=base_cloud_plan_rgb,
                gif_frames=int(plan_gif_cfg.get("device_plan_gif_frames", 48)),
                gif_fps=int(plan_gif_cfg.get("device_plan_gif_fps", 10)),
            )
        write_path_csv(dev_dir / "path_waypoints_display.csv", path_plot)
        visualize_generation_diagnostics(
            dv=dv,
            raw_vps=raw_vps,
            ordered_vps=ordered_vps,
            out_png=dev_dir / "generation_diagnostics.png",
        )
        if bool(cov_vis_cfg.get("enable", True)) and cov.get("surface_world", np.empty((0, 3))).shape[0] > 0:
            visualize_coverage_map(
                device_name=name,
                ordered_vps=ordered_vps,
                base_cloud_xyz=base_cloud_cov_xyz,
                base_cloud_rgb=base_cloud_cov_rgb,
                surface_world=cov["surface_world"],
                covered_mask=cov["covered_mask"],
                out_png=dev_dir / "coverage_map.png",
                out_gif=dev_dir / "coverage_map_rotate.gif",
                max_points=int(cov_vis_cfg.get("max_plot_points", 2800)),
                gif_frames=int(cov_vis_cfg.get("gif_frames", 48)),
                gif_fps=int(cov_vis_cfg.get("gif_fps", 10)),
                seed=int(cfg["random_seed"]) + i * 37,
            )

        baseline_compare = None
        cmp_cfg = cfg.get("reorder_baseline_compare", {})
        cmp_enable = bool(cmp_cfg.get("enable", False))
        cmp_names = {str(x).lower() for x in cmp_cfg.get("device_names", ["main_transformer"])}
        if cmp_enable and (name.lower() in cmp_names):
            t_br0 = time.perf_counter()
            tsp_vps = reorder_viewpoints_tsp_global(
                clone_viewpoints(raw_vps),
                two_opt_iter=int(cmp_cfg.get("tsp_2opt_iter", 1)),
            )
            t_br = float(time.perf_counter() - t_br0)
            t_bp0 = time.perf_counter()
            tsp_path = build_single_device_path(ordered_vps=tsp_vps, station=station, cfg=cfg)
            t_bp = float(time.perf_counter() - t_bp0)
            tsp_pairs = max(1, len(tsp_vps) - 1)
            tsp_pair_plan_ms = float((t_bp / tsp_pairs) * 1000.0)
            tsp_cov = evaluate_coverage(
                dv=dv,
                ordered_vps=tsp_vps,
                cfg=cfg,
                seed=int(cfg["random_seed"]) + i * 41,
                return_details=False,
            )
            display_source_cmp = str(vis_cfg.get("display_path_source", "viewpoint_order"))
            tsp_path_for_plot = tsp_path["path_points"]
            if display_source_cmp == "viewpoint_order" and tsp_vps:
                tsp_path_for_plot = np.array([[v.x, v.y, v.z] for v in tsp_vps], dtype=np.float64)
            preserve_all_plot_path_cmp = bool(vis_cfg.get("display_preserve_waypoints", True)) and display_source_cmp == "viewpoint_order"
            if preserve_all_plot_path_cmp:
                tsp_path_plot = tsp_path_for_plot.copy()
            else:
                tsp_path_plot = simplify_path_for_display(
                    tsp_path_for_plot,
                    min_step_m=float(vis_cfg.get("path_display_min_step_m", 0.6)),
                    max_points=int(vis_cfg.get("path_display_max_points", 1800)),
                )
                tsp_path_plot = smooth_path_for_display(
                    tsp_path_plot,
                    window_size=int(vis_cfg.get("path_display_smooth_window", 5)),
                )
            ours_pairs = max(1, len(ordered_vps) - 1)
            baseline_compare = {
                "baseline": "global_tsp_nn_2opt",
                "proposed": {
                    "ordered_viewpoints": int(len(ordered_vps)),
                    "coverage_ratio": float(cov.get("coverage_ratio", 0.0)),
                    "path_length_m": float(path_res.get("total_length_m", 0.0)),
                    "avg_path_length_per_viewpoint_pair_m": float(path_res.get("total_length_m", 0.0) / ours_pairs),
                    "avg_pair_path_length_m": float(path_res.get("avg_segment_length_m", 0.0)),
                    "min_pair_path_length_m": float(path_res.get("min_segment_length_m", 0.0)),
                    "max_pair_path_length_m": float(path_res.get("max_segment_length_m", 0.0)),
                    "reorder_time_s": float(t_reorder),
                    "path_planning_time_s": float(t_plan),
                    "avg_path_planning_time_per_viewpoint_pair_ms": float(t_plan_pair_ms),
                    "segment_success_ratio": float(path_res.get("segment_success_ratio", 1.0 if len(ordered_vps) <= 1 else 0.0)),
                },
                "baseline_global_tsp_nn_2opt": {
                    "ordered_viewpoints": int(len(tsp_vps)),
                    "coverage_ratio": float(tsp_cov.get("coverage_ratio", 0.0)),
                    "path_length_m": float(tsp_path.get("total_length_m", 0.0)),
                    "avg_path_length_per_viewpoint_pair_m": float(tsp_path.get("total_length_m", 0.0) / tsp_pairs),
                    "avg_pair_path_length_m": float(tsp_path.get("avg_segment_length_m", 0.0)),
                    "min_pair_path_length_m": float(tsp_path.get("min_segment_length_m", 0.0)),
                    "max_pair_path_length_m": float(tsp_path.get("max_segment_length_m", 0.0)),
                    "reorder_time_s": float(t_br),
                    "path_planning_time_s": float(t_bp),
                    "avg_path_planning_time_per_viewpoint_pair_ms": float(tsp_pair_plan_ms),
                    "segment_success_ratio": float(tsp_path.get("segment_success_ratio", 1.0 if len(tsp_vps) <= 1 else 0.0)),
                },
            }
            with (dev_dir / "reorder_baseline_compare.json").open("w", encoding="utf-8") as f:
                json.dump(
                    {
                        "device": name,
                        **baseline_compare,
                    },
                    f,
                    ensure_ascii=False,
                    indent=2,
                )
            if bool(cmp_cfg.get("write_compare_png", True)):
                visualize_reorder_baseline_compare(
                    device_name=name,
                    base_cloud_xyz=base_cloud_plan_xyz,
                    base_cloud_rgb=base_cloud_plan_rgb,
                    ours_vps=ordered_vps,
                    ours_path_plot=path_plot,
                    baseline_vps=tsp_vps,
                    baseline_path_plot=tsp_path_plot,
                    out_png=dev_dir / "reorder_baseline_compare.png",
                )

        t_dev = float(time.perf_counter() - t_dev0)

        report = {
            "device": name,
            "type": device_type,
            "standoff_m": standoff,
            "voxel_size_m": dv.voxel_size,
            "device_shape": list(dv.shape),
            "preprocess": {
                "ground_removed_layers": int(dv.ground_removed_layers),
                "ground_removed_voxels": int(dv.ground_removed_voxels),
            },
            "raw_viewpoints": len(raw_vps),
            "ordered_viewpoints": len(ordered_vps),
            "timing": {
                "viewpoint_generation_time_s": t_gen,
                "avg_generation_time_per_viewpoint_ms": t_per_vp_ms,
                "viewpoint_reorder_time_s": t_reorder,
                "coverage_eval_time_s": t_cov,
                "path_planning_time_s": t_plan,
                "avg_path_planning_time_per_viewpoint_pair_ms": t_plan_pair_ms,
                "device_total_runtime_s": t_dev,
            },
            "coverage": {
                "enabled": bool(cov["enabled"]),
                "sampled_surface_points": int(cov["sampled_surface_points"]),
                "covered_surface_points": int(cov["covered_surface_points"]),
                "coverage_ratio": float(cov["coverage_ratio"]),
            },
            "planning": {
                "enabled": bool(cfg["planning"].get("enable_single_device_path_planning", True)),
                "segment_success_ratio": float(path_res.get("segment_success_ratio", 1.0 if len(ordered_vps) <= 1 else 0.0)),
                "total_length_m": float(path_res["total_length_m"]),
                "avg_segment_length_m": float(path_res.get("avg_segment_length_m", 0.0)),
                "min_segment_length_m": float(path_res.get("min_segment_length_m", 0.0)),
                "max_segment_length_m": float(path_res.get("max_segment_length_m", 0.0)),
                "segments": path_res["segments"],
            },
            "outputs": {
                "capture_raw_csv": str(dev_dir / "viewpoints_capture_raw.csv"),
                "capture_ordered_csv": str(dev_dir / "viewpoints_capture_ordered.csv"),
                "nav_ordered_csv": str(dev_dir / "viewpoints_nav_ordered.csv"),
                "path_csv": str(dev_dir / "path_waypoints.csv"),
                "path_display_csv": str(dev_dir / "path_waypoints_display.csv"),
                "plot_png": str(dev_dir / "device_plan.png"),
                "plot_gif": str(dev_dir / "device_plan_rotate.gif"),
                "generation_diagnostics_png": str(dev_dir / "generation_diagnostics.png"),
                "coverage_map_png": str(dev_dir / "coverage_map.png"),
                "coverage_map_gif": str(dev_dir / "coverage_map_rotate.gif"),
                "reorder_baseline_compare_json": str(dev_dir / "reorder_baseline_compare.json"),
                "reorder_baseline_compare_png": str(dev_dir / "reorder_baseline_compare.png"),
            },
        }
        if baseline_compare is not None:
            report["reorder_baseline_compare"] = baseline_compare
        with (dev_dir / "device_report.json").open("w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)

        if cov["enabled"]:
            total_cov_n += int(cov["sampled_surface_points"])
            total_cov_ok += int(cov["covered_surface_points"])

        summary_rows.append(
            {
                "device": name,
                "type": device_type,
                "standoff_m": standoff,
                "raw_viewpoints": len(raw_vps),
                "ordered_viewpoints": len(ordered_vps),
                "coverage_ratio": float(cov["coverage_ratio"]) if cov["enabled"] else None,
                "sampled_surface_points": int(cov["sampled_surface_points"]),
                "covered_surface_points": int(cov["covered_surface_points"]),
                "viewpoint_generation_time_s": t_gen,
                "avg_generation_time_per_viewpoint_ms": t_per_vp_ms,
                "path_planning_time_s": t_plan,
                "avg_path_planning_time_per_viewpoint_pair_ms": t_plan_pair_ms,
                "device_total_runtime_s": t_dev,
                "path_length_m": float(path_res["total_length_m"]),
                "path_segment_success_ratio": float(path_res.get("segment_success_ratio", 1.0 if len(ordered_vps) <= 1 else 0.0)),
                "report_json": str(dev_dir / "device_report.json"),
            }
        )
        log(
            f"  viewpoints={len(ordered_vps)}, coverage={cov['coverage_ratio']:.3f}, "
            f"gen={t_gen:.2f}s ({t_per_vp_ms:.2f} ms/vp), path_len={path_res['total_length_m']:.2f}m"
        )

    sum_csv = out_dir / "phase2_single_device_summary.csv"
    with sum_csv.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "device",
                "type",
                "standoff_m",
                "raw_viewpoints",
                "ordered_viewpoints",
                "coverage_ratio",
                "sampled_surface_points",
                "covered_surface_points",
                "viewpoint_generation_time_s",
                "avg_generation_time_per_viewpoint_ms",
                "path_planning_time_s",
                "avg_path_planning_time_per_viewpoint_pair_ms",
                "device_total_runtime_s",
                "path_length_m",
                "path_segment_success_ratio",
                "report_json",
            ],
        )
        writer.writeheader()
        for r in summary_rows:
            writer.writerow(r)

    total_cov_ratio = float(total_cov_ok / max(1, total_cov_n))
    total_gen_time = float(sum(float(r.get("viewpoint_generation_time_s", 0.0)) for r in summary_rows))
    total_plan_time = float(sum(float(r.get("path_planning_time_s", 0.0)) for r in summary_rows))
    total_vp = int(sum(int(r.get("ordered_viewpoints", 0)) for r in summary_rows))
    total_pairs = int(sum(max(0, int(r.get("ordered_viewpoints", 0)) - 1) for r in summary_rows))
    summary_json = {
        "phase": "phase2_single_device",
        "device_count": len(summary_rows),
        "global_coverage_ratio_weighted": total_cov_ratio,
        "sampled_surface_points_total": total_cov_n,
        "covered_surface_points_total": total_cov_ok,
        "viewpoint_generation_time_total_s": total_gen_time,
        "viewpoint_generation_time_avg_per_viewpoint_ms": float((total_gen_time / max(1, total_vp)) * 1000.0),
        "path_planning_time_total_s": total_plan_time,
        "path_planning_time_avg_per_pair_ms": float((total_plan_time / max(1, total_pairs)) * 1000.0),
        "summary_csv": str(sum_csv),
    }
    with (out_dir / "phase2_single_device_summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary_json, f, ensure_ascii=False, indent=2)

    log("Phase 2 build complete")
    print(json.dumps(summary_json, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
