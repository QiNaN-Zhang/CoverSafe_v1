#!/usr/bin/env python3
"""Build station-level feasible voxel space for UAV substation inspection.

Implements requirements from try1.md:
1. Read/rotate/preprocess baseline LAS.
2. Compute dynamic safety distances and keep-out volumes from segmented devices.
3. Build and visualize station-level voxel no-fly map.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import laspy
from matplotlib import animation
from matplotlib import colors as mcolors
import matplotlib.pyplot as plt
import numpy as np


Voxel = Tuple[int, int, int]


@dataclass
class DeviceCloud:
    name: str
    path: Path
    device_type: str
    d_eff: float
    voxels_abs: Set[Voxel]


def log_progress(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def format_seconds(sec: float) -> str:
    sec_i = max(0, int(sec))
    h = sec_i // 3600
    m = (sec_i % 3600) // 60
    s = sec_i % 60
    if h > 0:
        return f"{h:02d}:{m:02d}:{s:02d}"
    return f"{m:02d}:{s:02d}"


def should_log_chunk(chunk_idx: int, total_chunks: int, every_n: int) -> bool:
    if chunk_idx == 1 or chunk_idx == total_chunks:
        return True
    if every_n <= 0:
        return False
    return (chunk_idx % every_n) == 0


def load_config(config_path: Path) -> dict:
    with config_path.open("r", encoding="utf-8-sig") as f:
        return json.load(f)


def resolve_input_path(root: Path, configured_path: str, fallback_dirs: Optional[List[str]] = None) -> Path:
    p = Path(configured_path)
    if p.is_absolute():
        if p.exists():
            return p
        raise FileNotFoundError(f"Configured absolute path not found: {p}")

    direct = (root / p).resolve()
    if direct.exists():
        return direct

    fallback_dirs = fallback_dirs or ["pointclouds", "."]
    name_only = p.name
    for d in fallback_dirs:
        candidate = (root / d / name_only).resolve()
        if candidate.exists():
            return candidate

    raise FileNotFoundError(
        f"Input file not found for '{configured_path}'. Tried: "
        f"{direct} and fallback dirs {fallback_dirs}"
    )


def infer_device_type(file_stem: str, type_keywords: Dict[str, List[str]]) -> str:
    stem = file_stem.lower()
    for device_type, keywords in type_keywords.items():
        if any(k.lower() in stem for k in keywords):
            return device_type
    return "default"


def get_type_priority(device_type: str, type_priority: Dict[str, int]) -> int:
    if device_type in type_priority:
        return int(type_priority[device_type])
    return int(type_priority.get("default", 0))


def sanitize_case_name(name: str) -> str:
    out = "".join(ch if (ch.isalnum() or ch in "-_") else "_" for ch in name.strip())
    return out or "case"


def discover_device_files(
    root: Path,
    baseline_name: str,
    origin_name: str,
    search_dirs: Optional[List[str]] = None,
    recursive: bool = False,
) -> List[Path]:
    search_dirs = search_dirs or ["."]
    files: List[Path] = []
    for d in search_dirs:
        base = (root / d).resolve()
        if not base.exists():
            continue
        if recursive:
            files.extend(sorted(base.rglob("*.las")))
        else:
            files.extend(sorted(base.glob("*.las")))
    excludes = {baseline_name.lower(), origin_name.lower()}
    uniq = {}
    for p in files:
        if p.name.lower() in excludes:
            continue
        uniq[str(p)] = p
    return sorted(uniq.values())


def random_sample_points_from_las(
    las_path: Path,
    max_points: int,
    chunk_size: int,
    seed: int,
    log_every_chunks: int = 20,
    progress_name: str = "rotation_sample",
) -> np.ndarray:
    rng = np.random.default_rng(seed)
    chunks: List[np.ndarray] = []
    collected = 0

    with laspy.open(las_path) as reader:
        total_points = int(reader.header.point_count)
        total_chunks = max(1, math.ceil(total_points / chunk_size))
        per_chunk = max(1, math.ceil(max_points / total_chunks))
        t0 = time.time()
        log_progress(
            f"{progress_name}: start ({total_points} pts, ~{total_chunks} chunks, target_sample={max_points})"
        )
        for chunk_idx, pts in enumerate(reader.chunk_iterator(chunk_size), start=1):
            xyz = np.column_stack((pts.x, pts.y, pts.z)).astype(np.float64, copy=False)
            n = xyz.shape[0]
            if n == 0:
                continue

            take = min(per_chunk, n)
            idx = rng.choice(n, size=take, replace=False)
            chunks.append(xyz[idx])
            collected += take

            if should_log_chunk(chunk_idx, total_chunks, log_every_chunks):
                elapsed = time.time() - t0
                eta = elapsed * (total_chunks / max(1, chunk_idx) - 1.0)
                log_progress(
                    f"{progress_name}: chunk {chunk_idx}/{total_chunks}, sampled={collected}, "
                    f"elapsed={format_seconds(elapsed)}, eta={format_seconds(eta)}"
                )

    if not chunks:
        return np.empty((0, 3), dtype=np.float64)

    sample = np.vstack(chunks)
    if sample.shape[0] > max_points:
        pick = rng.choice(sample.shape[0], size=max_points, replace=False)
        sample = sample[pick]
    log_progress(f"{progress_name}: done, final_sample={sample.shape[0]}")
    return sample


def sample_visual_cloud_from_las(
    las_path: Path,
    angle_rad: float,
    max_points: int,
    chunk_size: int,
    seed: int,
    bounds: Optional[np.ndarray] = None,
    z_floor: Optional[float] = None,
    log_every_chunks: int = 20,
    progress_name: str = "visual_cloud",
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    rng = np.random.default_rng(seed)
    chunks_xyz: List[np.ndarray] = []
    chunks_rgb: List[np.ndarray] = []
    collected = 0
    has_rgb = False

    with laspy.open(las_path) as reader:
        total_points = int(reader.header.point_count)
        total_chunks = max(1, math.ceil(total_points / chunk_size))
        per_chunk = max(1, math.ceil(max_points / total_chunks))
        t0 = time.time()
        log_progress(
            f"{progress_name}: start ({total_points} pts, ~{total_chunks} chunks, target_sample={max_points})"
        )
        for chunk_idx, pts in enumerate(reader.chunk_iterator(chunk_size), start=1):
            xyz = np.column_stack((pts.x, pts.y, pts.z)).astype(np.float64, copy=False)
            if xyz.shape[0] == 0:
                continue
            xyz = rotate_xy(xyz, angle_rad)
            keep = np.ones((xyz.shape[0],), dtype=bool)
            if z_floor is not None:
                keep &= xyz[:, 2] >= z_floor
            if bounds is not None:
                keep &= points_in_bounds(xyz, bounds)
            xyz = xyz[keep]
            if xyz.shape[0] == 0:
                continue

            rgb_norm: Optional[np.ndarray] = None
            if hasattr(pts, "red") and hasattr(pts, "green") and hasattr(pts, "blue"):
                red = np.asarray(pts.red)[keep]
                green = np.asarray(pts.green)[keep]
                blue = np.asarray(pts.blue)[keep]
                maxc = float(
                    max(
                        float(red.max()) if red.size else 0.0,
                        float(green.max()) if green.size else 0.0,
                        float(blue.max()) if blue.size else 0.0,
                        1.0,
                    )
                )
                denom = 65535.0 if maxc > 255.0 else 255.0
                rgb_norm = np.column_stack((red, green, blue)).astype(np.float64) / denom
                has_rgb = True

            take = min(per_chunk, xyz.shape[0])
            idx = rng.choice(xyz.shape[0], size=take, replace=False)
            chunks_xyz.append(xyz[idx])
            if rgb_norm is not None:
                chunks_rgb.append(rgb_norm[idx])
            collected += take

            if should_log_chunk(chunk_idx, total_chunks, log_every_chunks):
                elapsed = time.time() - t0
                eta = elapsed * (total_chunks / max(1, chunk_idx) - 1.0)
                log_progress(
                    f"{progress_name}: chunk {chunk_idx}/{total_chunks}, sampled={collected}, "
                    f"elapsed={format_seconds(elapsed)}, eta={format_seconds(eta)}"
                )

    if not chunks_xyz:
        return np.empty((0, 3), dtype=np.float64), None

    xyz_all = np.vstack(chunks_xyz)
    rgb_all = np.vstack(chunks_rgb) if has_rgb and chunks_rgb else None
    if xyz_all.shape[0] > max_points:
        pick = rng.choice(xyz_all.shape[0], size=max_points, replace=False)
        xyz_all = xyz_all[pick]
        if rgb_all is not None:
            rgb_all = rgb_all[pick]
    log_progress(f"{progress_name}: done, final_sample={xyz_all.shape[0]}, has_rgb={rgb_all is not None}")
    return xyz_all, rgb_all


def estimate_z_rotation_angle(points: np.ndarray) -> float:
    if points.shape[0] < 3:
        return 0.0
    xy = points[:, :2]
    xy_centered = xy - np.mean(xy, axis=0, keepdims=True)
    cov = np.cov(xy_centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    principal = eigvecs[:, np.argmax(eigvals)]
    theta = math.atan2(principal[1], principal[0])
    return -theta


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


def quantize_abs_voxels(points: np.ndarray, voxel_size: float) -> np.ndarray:
    return np.floor(points / voxel_size).astype(np.int32)


def points_in_bounds(points: np.ndarray, bounds: np.ndarray) -> np.ndarray:
    mins = bounds[0]
    maxs = bounds[1]
    return np.all((points >= mins) & (points <= maxs), axis=1)


def build_voxel_set_from_las(
    las_path: Path,
    angle_rad: float,
    voxel_size: float,
    chunk_size: int,
    bounds: Optional[np.ndarray] = None,
    z_floor: Optional[float] = None,
    log_every_chunks: int = 20,
    progress_name: str = "voxelize",
) -> Set[Voxel]:
    voxels: Set[Voxel] = set()
    with laspy.open(las_path) as reader:
        total_points = int(reader.header.point_count)
        total_chunks = max(1, math.ceil(total_points / chunk_size))
        t0 = time.time()
        accepted_points = 0
        log_progress(f"{progress_name}: start ({las_path.name}, ~{total_chunks} chunks)")
        for chunk_idx, pts in enumerate(reader.chunk_iterator(chunk_size), start=1):
            xyz = np.column_stack((pts.x, pts.y, pts.z)).astype(np.float64, copy=False)
            if xyz.shape[0] == 0:
                continue
            xyz = rotate_xy(xyz, angle_rad)
            if z_floor is not None:
                xyz = xyz[xyz[:, 2] >= z_floor]
                if xyz.shape[0] == 0:
                    continue
            if bounds is not None:
                mask = points_in_bounds(xyz, bounds)
                xyz = xyz[mask]
                if xyz.shape[0] == 0:
                    continue

            accepted_points += int(xyz.shape[0])
            idx = quantize_abs_voxels(xyz, voxel_size)
            unique_idx = np.unique(idx, axis=0)
            voxels.update(map(tuple, unique_idx.tolist()))
            if should_log_chunk(chunk_idx, total_chunks, log_every_chunks):
                elapsed = time.time() - t0
                eta = elapsed * (total_chunks / max(1, chunk_idx) - 1.0)
                log_progress(
                    f"{progress_name}: chunk {chunk_idx}/{total_chunks}, voxels={len(voxels)}, "
                    f"accepted_pts={accepted_points}, elapsed={format_seconds(elapsed)}, eta={format_seconds(eta)}"
                )

    log_progress(f"{progress_name}: done, unique_voxels={len(voxels)}")
    return voxels


def compute_baseline_stats(
    baseline_path: Path,
    angle_rad: float,
    chunk_size: int,
    ground_quantile: float,
    z_sample_max: int,
    seed: int,
    log_every_chunks: int = 20,
    progress_name: str = "baseline_stats",
) -> Tuple[np.ndarray, float]:
    mins = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
    maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
    rng = np.random.default_rng(seed)
    z_sample = np.empty((0,), dtype=np.float64)

    with laspy.open(baseline_path) as reader:
        total_points = int(reader.header.point_count)
        total_chunks = max(1, math.ceil(total_points / chunk_size))
        t0 = time.time()
        log_progress(f"{progress_name}: start ({baseline_path.name}, ~{total_chunks} chunks)")
        for chunk_idx, pts in enumerate(reader.chunk_iterator(chunk_size), start=1):
            xyz = np.column_stack((pts.x, pts.y, pts.z)).astype(np.float64, copy=False)
            if xyz.shape[0] == 0:
                continue
            xyz = rotate_xy(xyz, angle_rad)

            mins = np.minimum(mins, np.min(xyz, axis=0))
            maxs = np.maximum(maxs, np.max(xyz, axis=0))

            z = xyz[:, 2]
            if z_sample.shape[0] < z_sample_max:
                need = min(z_sample_max - z_sample.shape[0], z.shape[0])
                idx = rng.choice(z.shape[0], size=need, replace=False)
                z_sample = np.concatenate((z_sample, z[idx]))
            else:
                idx = rng.choice(z.shape[0], size=min(256, z.shape[0]), replace=False)
                replace_pos = rng.choice(z_sample.shape[0], size=idx.shape[0], replace=False)
                z_sample[replace_pos] = z[idx]
            if should_log_chunk(chunk_idx, total_chunks, log_every_chunks):
                elapsed = time.time() - t0
                eta = elapsed * (total_chunks / max(1, chunk_idx) - 1.0)
                log_progress(
                    f"{progress_name}: chunk {chunk_idx}/{total_chunks}, z_sample={z_sample.shape[0]}, "
                    f"elapsed={format_seconds(elapsed)}, eta={format_seconds(eta)}"
                )

    z_ground = float(np.quantile(z_sample, ground_quantile)) if z_sample.size else float(mins[2])
    bounds = np.vstack((mins, maxs))
    log_progress(f"{progress_name}: done, z_ground={z_ground:.3f}")
    return bounds, z_ground


def build_sphere_offsets(radius_m: float, voxel_size: float) -> np.ndarray:
    r = int(math.ceil(radius_m / voxel_size))
    if r <= 0:
        return np.array([[0, 0, 0]], dtype=np.int16)
    rr = (radius_m / voxel_size) ** 2
    rng = np.arange(-r, r + 1, dtype=np.int16)
    xx, yy, zz = np.meshgrid(rng, rng, rng, indexing="ij")
    mask = (xx.astype(np.float32) ** 2 + yy.astype(np.float32) ** 2 + zz.astype(np.float32) ** 2) <= rr
    return np.column_stack((xx[mask], yy[mask], zz[mask])).astype(np.int16)


def abs_to_local_voxels(
    voxels_abs: Set[Voxel],
    base_idx: np.ndarray,
    shape: np.ndarray,
) -> Set[Voxel]:
    out: Set[Voxel] = set()
    sx, sy, sz = map(int, shape.tolist())
    bx, by, bz = map(int, base_idx.tolist())
    for ax, ay, az in voxels_abs:
        x = ax - bx
        y = ay - by
        z = az - bz
        if 0 <= x < sx and 0 <= y < sy and 0 <= z < sz:
            out.add((x, y, z))
    return out


def abs_to_local_voxel_array(
    voxels_abs: Set[Voxel],
    base_idx: np.ndarray,
    shape: np.ndarray,
) -> np.ndarray:
    sx, sy, sz = map(int, shape.tolist())
    bx, by, bz = map(int, base_idx.tolist())
    out: List[Voxel] = []
    for ax, ay, az in voxels_abs:
        x = ax - bx
        y = ay - by
        z = az - bz
        if 0 <= x < sx and 0 <= y < sy and 0 <= z < sz:
            out.append((x, y, z))
    if not out:
        return np.empty((0, 3), dtype=np.int32)
    return np.asarray(out, dtype=np.int32)


def inflate_sparse_voxels(
    source_voxels: Set[Voxel],
    offsets: np.ndarray,
    shape: np.ndarray,
    blocked: Set[Voxel],
) -> None:
    sx, sy, sz = map(int, shape.tolist())
    for x, y, z in source_voxels:
        for dx, dy, dz in offsets:
            nx = x + int(dx)
            ny = y + int(dy)
            nz = z + int(dz)
            if 0 <= nx < sx and 0 <= ny < sy and 0 <= nz < sz:
                blocked.add((nx, ny, nz))


def inflate_sparse_voxels_vectorized(
    source_idx: np.ndarray,
    offsets: np.ndarray,
    shape: np.ndarray,
    blocked_mask: np.ndarray,
) -> None:
    if source_idx.shape[0] == 0:
        return
    sx, sy, sz = map(int, shape.tolist())
    xs = source_idx[:, 0]
    ys = source_idx[:, 1]
    zs = source_idx[:, 2]
    for dx, dy, dz in offsets:
        nx = xs + int(dx)
        ny = ys + int(dy)
        nz = zs + int(dz)
        valid = (nx >= 0) & (nx < sx) & (ny >= 0) & (ny < sy) & (nz >= 0) & (nz < sz)
        if np.any(valid):
            blocked_mask[nx[valid], ny[valid], nz[valid]] = True


def write_pcd_xyz_binary(path: Path, points: np.ndarray) -> None:
    n = points.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        points.astype(np.float32).tofile(f)


def visualize_blocked_voxels(
    blocked_idx: np.ndarray,
    origin: np.ndarray,
    voxel_size: float,
    out_dir: Path,
    max_plot_points: int,
    seed: int,
) -> None:
    if blocked_idx.shape[0] == 0:
        return
    rng = np.random.default_rng(seed)
    if blocked_idx.shape[0] > max_plot_points:
        pick = rng.choice(blocked_idx.shape[0], size=max_plot_points, replace=False)
        idx = blocked_idx[pick]
    else:
        idx = blocked_idx

    pts = origin[None, :] + (idx.astype(np.float64) + 0.5) * voxel_size

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    sc = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=pts[:, 2], s=0.8, cmap="viridis", alpha=0.8)
    fig.colorbar(sc, ax=ax, shrink=0.6, label="z (m)")
    ax.set_title("No-fly voxels (3D sample)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    fig.tight_layout()
    fig.savefig(out_dir / "no_fly_zones_3d.png", dpi=220)
    plt.close(fig)

    fig2, ax2 = plt.subplots(figsize=(10, 8))
    ax2.scatter(pts[:, 0], pts[:, 1], c=pts[:, 2], s=1.0, cmap="plasma", alpha=0.7)
    ax2.set_title("No-fly voxels top view (colored by z)")
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_aspect("equal", adjustable="box")
    fig2.tight_layout()
    fig2.savefig(out_dir / "no_fly_zones_topview.png", dpi=220)
    plt.close(fig2)


def sample_blocked_voxel_points(
    blocked_idx: np.ndarray,
    origin: np.ndarray,
    voxel_size: float,
    max_points: int,
    seed: int,
) -> np.ndarray:
    if blocked_idx.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)
    rng = np.random.default_rng(seed)
    if blocked_idx.shape[0] > max_points:
        pick = rng.choice(blocked_idx.shape[0], size=max_points, replace=False)
        idx = blocked_idx[pick]
    else:
        idx = blocked_idx
    return origin[None, :] + (idx.astype(np.float64) + 0.5) * voxel_size


def sample_free_voxel_points(
    blocked_mask: np.ndarray,
    origin: np.ndarray,
    voxel_size: float,
    max_points: int,
    seed: int,
    z_range: Optional[Tuple[float, float]] = None,
) -> np.ndarray:
    if max_points <= 0:
        return np.empty((0, 3), dtype=np.float64)
    sx, sy, sz = blocked_mask.shape
    z_idx_pool: Optional[np.ndarray] = None
    if z_range is not None:
        z0, z1 = z_range
        z_centers = origin[2] + (np.arange(sz, dtype=np.float64) + 0.5) * voxel_size
        z_idx_pool = np.where((z_centers >= z0) & (z_centers <= z1))[0]
        if z_idx_pool.size == 0:
            z_idx_pool = np.array([int(np.argmin(np.abs(z_centers - 0.5 * (z0 + z1))))], dtype=np.int32)

    if z_idx_pool is None:
        free_total = int(blocked_mask.size - np.count_nonzero(blocked_mask))
    else:
        local = blocked_mask[:, :, z_idx_pool]
        free_total = int(local.size - np.count_nonzero(local))
    if free_total <= 0:
        return np.empty((0, 3), dtype=np.float64)
    target = min(max_points, free_total)

    rng = np.random.default_rng(seed)
    picked: Set[Voxel] = set()
    attempts = 0
    max_attempts = max(200000, target * 80)
    while len(picked) < target and attempts < max_attempts:
        remaining = target - len(picked)
        batch = min(max(remaining * 6, 8000), 250000)
        xs = rng.integers(0, sx, size=batch, endpoint=False)
        ys = rng.integers(0, sy, size=batch, endpoint=False)
        if z_idx_pool is None:
            zs = rng.integers(0, sz, size=batch, endpoint=False)
        else:
            pick = rng.integers(0, z_idx_pool.size, size=batch, endpoint=False)
            zs = z_idx_pool[pick]
        for x, y, z in zip(xs.tolist(), ys.tolist(), zs.tolist()):
            v = (x, y, z)
            if blocked_mask[x, y, z] or v in picked:
                continue
            picked.add(v)
            if len(picked) >= target:
                break
        attempts += batch

    if not picked:
        return np.empty((0, 3), dtype=np.float64)
    idx = np.array(list(picked), dtype=np.int32)
    return origin[None, :] + (idx.astype(np.float64) + 0.5) * voxel_size


def visualize_structural_slices(
    blocked_mask: np.ndarray,
    origin: np.ndarray,
    voxel_size: float,
    z_ground: float,
    mission_altitude_agl: float,
    out_dir: Path,
) -> None:
    sx, sy, sz = blocked_mask.shape
    z_centers = origin[2] + (np.arange(sz, dtype=np.float64) + 0.5) * voxel_size
    mission_z = z_ground + mission_altitude_agl
    k_mission = int(np.argmin(np.abs(z_centers - mission_z)))

    x_extent = (origin[0], origin[0] + sx * voxel_size)
    y_extent = (origin[1], origin[1] + sy * voxel_size)
    z_extent = (z_centers[0], z_centers[-1])

    blocked_top_ratio = blocked_mask.mean(axis=2).T
    blocked_xy_mission = blocked_mask[:, :, k_mission].T.astype(np.float32)
    mid_x = sx // 2
    mid_y = sy // 2
    blocked_xz = blocked_mask[:, mid_y, :].T.astype(np.float32)
    blocked_yz = blocked_mask[mid_x, :, :].T.astype(np.float32)

    fig = plt.figure(figsize=(16, 10))
    ax1 = fig.add_subplot(2, 2, 1)
    im1 = ax1.imshow(
        blocked_top_ratio,
        origin="lower",
        extent=(x_extent[0], x_extent[1], y_extent[0], y_extent[1]),
        cmap="magma",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
    )
    fig.colorbar(im1, ax=ax1, shrink=0.78, label="Blocked ratio over z")
    ax1.set_title("Top view blocked ratio (all heights)")
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_aspect("equal", adjustable="box")

    ax2 = fig.add_subplot(2, 2, 2)
    im2 = ax2.imshow(
        blocked_xy_mission,
        origin="lower",
        extent=(x_extent[0], x_extent[1], y_extent[0], y_extent[1]),
        cmap="coolwarm",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
    )
    fig.colorbar(im2, ax=ax2, shrink=0.78, label="Blocked at mission layer")
    ax2.set_title(f"Mission slice XY @ z={z_centers[k_mission]:.2f}m")
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_aspect("equal", adjustable="box")

    ax3 = fig.add_subplot(2, 2, 3)
    im3 = ax3.imshow(
        blocked_xz,
        origin="lower",
        extent=(x_extent[0], x_extent[1], z_extent[0], z_extent[1]),
        cmap="Greys",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
    )
    fig.colorbar(im3, ax=ax3, shrink=0.78, label="Blocked")
    ax3.set_title(f"XZ section @ y={origin[1] + (mid_y + 0.5) * voxel_size:.2f}m")
    ax3.set_xlabel("x")
    ax3.set_ylabel("z")

    ax4 = fig.add_subplot(2, 2, 4)
    im4 = ax4.imshow(
        blocked_yz,
        origin="lower",
        extent=(y_extent[0], y_extent[1], z_extent[0], z_extent[1]),
        cmap="Greys",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
    )
    fig.colorbar(im4, ax=ax4, shrink=0.78, label="Blocked")
    ax4.set_title(f"YZ section @ x={origin[0] + (mid_x + 0.5) * voxel_size:.2f}m")
    ax4.set_xlabel("y")
    ax4.set_ylabel("z")

    fig.tight_layout()
    fig.savefig(out_dir / "blocked_space_slices.png", dpi=220)
    plt.close(fig)


def visualize_overlay_and_gif(
    cloud_xyz: np.ndarray,
    cloud_rgb: Optional[np.ndarray],
    blocked_xyz: np.ndarray,
    out_dir: Path,
    gif_frames: int,
    gif_fps: int,
    elev: float,
    gif_name: str,
) -> Tuple[Path, Path]:
    png_path = out_dir / "overlay_cloud_nofly.png"
    gif_path = out_dir / gif_name

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    if cloud_xyz.shape[0] > 0:
        if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
            ax.scatter(
                cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2],
                c=cloud_rgb, s=0.45, alpha=0.09, depthshade=False
            )
        else:
            ax.scatter(
                cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2],
                c="lightgray", s=0.4, alpha=0.08, depthshade=False
            )
    if blocked_xyz.shape[0] > 0:
        ax.scatter(
            blocked_xyz[:, 0], blocked_xyz[:, 1], blocked_xyz[:, 2],
            c="red", s=0.9, alpha=0.36, depthshade=False
        )
    ax.set_title("Station cloud + no-fly space overlay")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.view_init(elev=elev, azim=25.0)
    fig.tight_layout()
    fig.savefig(png_path, dpi=220)
    plt.close(fig)

    if gif_frames <= 1 or blocked_xyz.shape[0] == 0:
        return png_path, gif_path

    fig2 = plt.figure(figsize=(10, 8))
    ax2 = fig2.add_subplot(projection="3d")
    if cloud_xyz.shape[0] > 0:
        if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
            ax2.scatter(
                cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2],
                c=cloud_rgb, s=0.45, alpha=0.08, depthshade=False
            )
        else:
            ax2.scatter(
                cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2],
                c="lightgray", s=0.4, alpha=0.08, depthshade=False
            )
    ax2.scatter(
        blocked_xyz[:, 0], blocked_xyz[:, 1], blocked_xyz[:, 2],
        c="red", s=0.85, alpha=0.35, depthshade=False
    )
    ax2.set_title("Rotating view: cloud + no-fly space")
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_zlabel("z")

    try:
        writer = animation.PillowWriter(fps=max(1, gif_fps))
        with writer.saving(fig2, str(gif_path), dpi=140):
            for i in range(gif_frames):
                az = 360.0 * (i / gif_frames)
                ax2.view_init(elev=elev, azim=az)
                writer.grab_frame()
    except Exception as exc:
        log_progress(f"GIF export skipped: {exc}")
    plt.close(fig2)
    return png_path, gif_path


def visualize_compare_spaces_and_gif(
    cloud_xyz: np.ndarray,
    cloud_rgb: Optional[np.ndarray],
    blocked_xyz: np.ndarray,
    free_xyz: np.ndarray,
    out_dir: Path,
    gif_frames: int,
    gif_fps: int,
    elev: float,
    gif_name: str,
) -> Tuple[Path, Path]:
    png_path = out_dir / "overlay_compare_nofly_free.png"
    gif_path = out_dir / gif_name

    fig = plt.figure(figsize=(16, 8))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    for ax in (ax1, ax2):
        if cloud_xyz.shape[0] > 0:
            if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
                ax.scatter(
                    cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2],
                    c=cloud_rgb, s=0.42, alpha=0.08, depthshade=False
                )
            else:
                ax.scatter(
                    cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2],
                    c="lightgray", s=0.4, alpha=0.08, depthshade=False
                )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

    if blocked_xyz.shape[0] > 0:
        ax1.scatter(
            blocked_xyz[:, 0], blocked_xyz[:, 1], blocked_xyz[:, 2],
            c="red", s=0.85, alpha=0.36, depthshade=False
        )
    ax1.set_title("No-fly space (blocked)")
    ax1.view_init(elev=elev, azim=25.0)

    if free_xyz.shape[0] > 0:
        ax2.scatter(
            free_xyz[:, 0], free_xyz[:, 1], free_xyz[:, 2],
            c="deepskyblue", s=0.95, alpha=0.28, depthshade=False
        )
    ax2.set_title("Flyable space sample (free, near-ground layer)")
    ax2.view_init(elev=elev, azim=25.0)

    fig.tight_layout()
    fig.savefig(png_path, dpi=220)
    plt.close(fig)

    if gif_frames <= 1:
        return png_path, gif_path

    fig2 = plt.figure(figsize=(16, 8))
    bx1 = fig2.add_subplot(1, 2, 1, projection="3d")
    bx2 = fig2.add_subplot(1, 2, 2, projection="3d")
    for bx in (bx1, bx2):
        if cloud_xyz.shape[0] > 0:
            if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
                bx.scatter(
                    cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2],
                    c=cloud_rgb, s=0.42, alpha=0.08, depthshade=False
                )
            else:
                bx.scatter(
                    cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2],
                    c="lightgray", s=0.4, alpha=0.08, depthshade=False
                )
        bx.set_xlabel("x")
        bx.set_ylabel("y")
        bx.set_zlabel("z")
    if blocked_xyz.shape[0] > 0:
        bx1.scatter(
            blocked_xyz[:, 0], blocked_xyz[:, 1], blocked_xyz[:, 2],
            c="red", s=0.85, alpha=0.35, depthshade=False
        )
    if free_xyz.shape[0] > 0:
        bx2.scatter(
            free_xyz[:, 0], free_xyz[:, 1], free_xyz[:, 2],
            c="deepskyblue", s=0.9, alpha=0.27, depthshade=False
        )
    bx1.set_title("No-fly space")
    bx2.set_title("Flyable space sample (near-ground)")

    try:
        writer = animation.PillowWriter(fps=max(1, gif_fps))
        with writer.saving(fig2, str(gif_path), dpi=130):
            for i in range(gif_frames):
                az = 360.0 * (i / gif_frames)
                bx1.view_init(elev=elev, azim=az)
                bx2.view_init(elev=elev, azim=az)
                writer.grab_frame()
    except Exception as exc:
        log_progress(f"Compare GIF export skipped: {exc}")
    plt.close(fig2)
    return png_path, gif_path


def draw_bbox_lines(ax: Any, mins: np.ndarray, maxs: np.ndarray, color: str, lw: float = 1.0, alpha: float = 0.9) -> None:
    x0, y0, z0 = mins.tolist()
    x1, y1, z1 = maxs.tolist()
    corners = np.array(
        [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ],
        dtype=np.float64,
    )
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    for i0, i1 in edges:
        ax.plot(
            [corners[i0, 0], corners[i1, 0]],
            [corners[i0, 1], corners[i1, 1]],
            [corners[i0, 2], corners[i1, 2]],
            color=color,
            linewidth=lw,
            alpha=alpha,
        )


def visualize_device_types_and_gif(
    devices: List[DeviceCloud],
    voxel_size: float,
    out_dir: Path,
    seed: int,
    max_points_per_type: int,
    max_bboxes: int,
    type_colors: Dict[str, str],
    legend_display_names: Dict[str, str],
    gif_frames: int,
    gif_fps: int,
    elev: float,
    gif_name: str,
) -> Tuple[Path, Path]:
    png_path = out_dir / "device_types_overview.png"
    gif_path = out_dir / gif_name
    rng = np.random.default_rng(seed)

    type_points_abs: Dict[str, np.ndarray] = {}
    per_dev_bbox: List[Tuple[str, np.ndarray, np.ndarray]] = []
    for d in devices:
        if not d.voxels_abs:
            continue
        arr = np.array(list(d.voxels_abs), dtype=np.int32)
        if arr.shape[0] > max_points_per_type:
            pick = rng.choice(arr.shape[0], size=max_points_per_type, replace=False)
            arr_plot = arr[pick]
        else:
            arr_plot = arr
        if d.device_type in type_points_abs:
            type_points_abs[d.device_type] = np.vstack((type_points_abs[d.device_type], arr_plot))
        else:
            type_points_abs[d.device_type] = arr_plot
        per_dev_bbox.append((d.device_type, arr.min(axis=0).astype(np.float64), arr.max(axis=0).astype(np.float64)))

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(projection="3d")
    for device_type, arr in sorted(type_points_abs.items()):
        if arr.shape[0] > max_points_per_type:
            pick = rng.choice(arr.shape[0], size=max_points_per_type, replace=False)
            arr = arr[pick]
        pts = (arr.astype(np.float64) + 0.5) * voxel_size
        c = type_colors.get(device_type, "#808080")
        label = legend_display_names.get(device_type, device_type)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=0.8, alpha=0.62, depthshade=False, c=c, label=label)

    if per_dev_bbox:
        bbox_pick = per_dev_bbox if len(per_dev_bbox) <= max_bboxes else [per_dev_bbox[i] for i in rng.choice(len(per_dev_bbox), size=max_bboxes, replace=False)]
        for dtype, mins_abs, maxs_abs in bbox_pick:
            c = type_colors.get(dtype, "#606060")
            mins = (mins_abs + 0.5) * voxel_size
            maxs = (maxs_abs + 0.5) * voxel_size
            draw_bbox_lines(ax, mins, maxs, c, lw=1.0, alpha=0.9)

    ax.set_title("Segmented device categories (colored + bbox)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.view_init(elev=elev, azim=25.0)
    ax.legend(loc="upper left", markerscale=6)
    fig.tight_layout()
    fig.savefig(png_path, dpi=220)
    plt.close(fig)

    if gif_frames <= 1:
        return png_path, gif_path

    fig2 = plt.figure(figsize=(12, 9))
    bx = fig2.add_subplot(projection="3d")
    for device_type, arr in sorted(type_points_abs.items()):
        if arr.shape[0] > max_points_per_type:
            pick = rng.choice(arr.shape[0], size=max_points_per_type, replace=False)
            arr = arr[pick]
        pts = (arr.astype(np.float64) + 0.5) * voxel_size
        c = type_colors.get(device_type, "#808080")
        label = legend_display_names.get(device_type, device_type)
        bx.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=0.75, alpha=0.62, depthshade=False, c=c, label=label)
    if per_dev_bbox:
        bbox_pick = per_dev_bbox if len(per_dev_bbox) <= max_bboxes else [per_dev_bbox[i] for i in rng.choice(len(per_dev_bbox), size=max_bboxes, replace=False)]
        for dtype, mins_abs, maxs_abs in bbox_pick:
            c = type_colors.get(dtype, "#606060")
            mins = (mins_abs + 0.5) * voxel_size
            maxs = (maxs_abs + 0.5) * voxel_size
            draw_bbox_lines(bx, mins, maxs, c, lw=1.0, alpha=0.86)
    bx.set_title("Rotating segmented categories")
    bx.set_xlabel("x")
    bx.set_ylabel("y")
    bx.set_zlabel("z")
    bx.legend(loc="upper left", markerscale=6)

    try:
        writer = animation.PillowWriter(fps=max(1, gif_fps))
        with writer.saving(fig2, str(gif_path), dpi=140):
            for i in range(gif_frames):
                az = 360.0 * (i / gif_frames)
                bx.view_init(elev=elev, azim=az)
                writer.grab_frame()
    except Exception as exc:
        log_progress(f"Device-type GIF export skipped: {exc}")
    plt.close(fig2)
    return png_path, gif_path


def visualize_free_space_bands_and_gif(
    cloud_xyz: np.ndarray,
    cloud_rgb: Optional[np.ndarray],
    free_low_xyz: np.ndarray,
    free_mid_xyz: np.ndarray,
    free_full_xyz: np.ndarray,
    out_dir: Path,
    gif_frames: int,
    gif_fps: int,
    elev: float,
    gif_name: str,
) -> Tuple[Path, Path]:
    png_path = out_dir / "overlay_free_bands.png"
    gif_path = out_dir / gif_name

    fig = plt.figure(figsize=(18, 6))
    axs = [
        fig.add_subplot(1, 3, 1, projection="3d"),
        fig.add_subplot(1, 3, 2, projection="3d"),
        fig.add_subplot(1, 3, 3, projection="3d"),
    ]
    titles = ["Flyable: Low layer (~2.5m AGL)", "Flyable: Mid-low layer (~10m AGL)", "Flyable: Full volume"]
    free_sets = [free_low_xyz, free_mid_xyz, free_full_xyz]

    for ax, title, free_xyz in zip(axs, titles, free_sets):
        if cloud_xyz.shape[0] > 0:
            if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
                ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c=cloud_rgb, s=0.34, alpha=0.06, depthshade=False)
            else:
                ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c="lightgray", s=0.32, alpha=0.06, depthshade=False)
        if free_xyz.shape[0] > 0:
            ax.scatter(free_xyz[:, 0], free_xyz[:, 1], free_xyz[:, 2], c="deepskyblue", s=0.86, alpha=0.27, depthshade=False)
        ax.set_title(title)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.view_init(elev=elev, azim=25.0)

    fig.tight_layout()
    fig.savefig(png_path, dpi=220)
    plt.close(fig)

    if gif_frames <= 1:
        return png_path, gif_path

    fig2 = plt.figure(figsize=(18, 6))
    bxs = [
        fig2.add_subplot(1, 3, 1, projection="3d"),
        fig2.add_subplot(1, 3, 2, projection="3d"),
        fig2.add_subplot(1, 3, 3, projection="3d"),
    ]
    for bx, title, free_xyz in zip(bxs, titles, free_sets):
        if cloud_xyz.shape[0] > 0:
            if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
                bx.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c=cloud_rgb, s=0.32, alpha=0.05, depthshade=False)
            else:
                bx.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c="lightgray", s=0.3, alpha=0.05, depthshade=False)
        if free_xyz.shape[0] > 0:
            bx.scatter(free_xyz[:, 0], free_xyz[:, 1], free_xyz[:, 2], c="deepskyblue", s=0.82, alpha=0.26, depthshade=False)
        bx.set_title(title)
        bx.set_xlabel("x")
        bx.set_ylabel("y")
        bx.set_zlabel("z")

    try:
        writer = animation.PillowWriter(fps=max(1, gif_fps))
        with writer.saving(fig2, str(gif_path), dpi=130):
            for i in range(gif_frames):
                az = 360.0 * (i / gif_frames)
                for bx in bxs:
                    bx.view_init(elev=elev, azim=az)
                writer.grab_frame()
    except Exception as exc:
        log_progress(f"Free-band GIF export skipped: {exc}")
    plt.close(fig2)
    return png_path, gif_path


def visualize_free_space_layer_scan_gif(
    blocked_mask: np.ndarray,
    origin: np.ndarray,
    voxel_size: float,
    z_ground: float,
    out_dir: Path,
    stride: int,
    fps: int,
    gif_name: str,
) -> Tuple[Path, Path]:
    png_path = out_dir / "free_space_layer_scan_preview.png"
    gif_path = out_dir / gif_name
    sx, sy, sz = blocked_mask.shape
    z_centers = origin[2] + (np.arange(sz, dtype=np.float64) + 0.5) * voxel_size
    x_extent = (origin[0], origin[0] + sx * voxel_size)
    y_extent = (origin[1], origin[1] + sy * voxel_size)

    k_preview = int(np.argmin(np.abs(z_centers - (z_ground + 2.5))))
    free_preview = (~blocked_mask[:, :, k_preview]).T.astype(np.float32)
    fig = plt.figure(figsize=(8.5, 7.2))
    ax = fig.add_subplot(111)
    im = ax.imshow(
        free_preview,
        origin="lower",
        extent=(x_extent[0], x_extent[1], y_extent[0], y_extent[1]),
        cmap="Blues",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
    )
    fig.colorbar(im, ax=ax, shrink=0.78, label="Free (1) / Blocked (0)")
    ax.set_title(f"Free-space scan preview @ z={z_centers[k_preview]:.2f}m (AGL {z_centers[k_preview]-z_ground:.2f}m)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    fig.savefig(png_path, dpi=220)
    plt.close(fig)

    stride = max(1, int(stride))
    layer_ids = list(range(0, sz, stride))
    if not layer_ids or layer_ids[-1] != (sz - 1):
        layer_ids.append(sz - 1)

    fig2 = plt.figure(figsize=(8.5, 7.2))
    ax2 = fig2.add_subplot(111)
    first_map = (~blocked_mask[:, :, layer_ids[0]]).T.astype(np.float32)
    img = ax2.imshow(
        first_map,
        origin="lower",
        extent=(x_extent[0], x_extent[1], y_extent[0], y_extent[1]),
        cmap="Blues",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
    )
    cb = fig2.colorbar(img, ax=ax2, shrink=0.78, label="Free (1) / Blocked (0)")
    _ = cb
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_aspect("equal", adjustable="box")

    try:
        writer = animation.PillowWriter(fps=max(1, int(fps)))
        with writer.saving(fig2, str(gif_path), dpi=140):
            for k in layer_ids:
                free_map = (~blocked_mask[:, :, k]).T.astype(np.float32)
                img.set_data(free_map)
                ax2.set_title(f"Free-space scan layer z={z_centers[k]:.2f}m (AGL {z_centers[k]-z_ground:.2f}m)")
                writer.grab_frame()
    except Exception as exc:
        log_progress(f"Free-space scan GIF export skipped: {exc}")
    plt.close(fig2)
    return png_path, gif_path


def visualize_free_space_layer_scan_iso_gif(
    blocked_mask: np.ndarray,
    cloud_xyz: np.ndarray,
    cloud_rgb: Optional[np.ndarray],
    origin: np.ndarray,
    voxel_size: float,
    z_ground: float,
    out_dir: Path,
    stride: int,
    fps: int,
    elev: float,
    azim: float,
    max_free_points_per_layer: int,
    seed: int,
    gif_name: str,
) -> Tuple[Path, Path]:
    png_path = out_dir / "free_space_layer_scan_iso_preview.png"
    gif_path = out_dir / gif_name
    sx, sy, sz = blocked_mask.shape
    rng = np.random.default_rng(seed)
    z_centers = origin[2] + (np.arange(sz, dtype=np.float64) + 0.5) * voxel_size
    xlim = (origin[0], origin[0] + sx * voxel_size)
    ylim = (origin[1], origin[1] + sy * voxel_size)
    zlim = (z_centers[0], z_centers[-1])

    stride = max(1, int(stride))
    layer_ids = list(range(0, sz, stride))
    if not layer_ids or layer_ids[-1] != (sz - 1):
        layer_ids.append(sz - 1)

    def sample_layer_points(k: int) -> np.ndarray:
        free_xy = np.argwhere(~blocked_mask[:, :, k])
        if free_xy.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float64)
        take = min(max_free_points_per_layer, free_xy.shape[0])
        if free_xy.shape[0] > take:
            pick = rng.choice(free_xy.shape[0], size=take, replace=False)
            free_xy = free_xy[pick]
        pts = np.empty((free_xy.shape[0], 3), dtype=np.float64)
        pts[:, 0] = origin[0] + (free_xy[:, 0].astype(np.float64) + 0.5) * voxel_size
        pts[:, 1] = origin[1] + (free_xy[:, 1].astype(np.float64) + 0.5) * voxel_size
        pts[:, 2] = z_centers[k]
        return pts

    k_preview = int(np.argmin(np.abs(z_centers - (z_ground + 2.5))))
    free_preview = sample_layer_points(k_preview)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    if cloud_xyz.shape[0] > 0:
        if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
            ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c=cloud_rgb, s=0.28, alpha=0.05, depthshade=False)
        else:
            ax.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c="lightgray", s=0.26, alpha=0.05, depthshade=False)
    if free_preview.shape[0] > 0:
        ax.scatter(free_preview[:, 0], free_preview[:, 1], free_preview[:, 2], c="deepskyblue", s=1.05, alpha=0.34, depthshade=False)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)
    ax.set_title(f"Free-space ISO scan preview @ z={z_centers[k_preview]:.2f}m (AGL {z_centers[k_preview]-z_ground:.2f}m)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.view_init(elev=elev, azim=azim)
    fig.tight_layout()
    fig.savefig(png_path, dpi=220)
    plt.close(fig)

    fig2 = plt.figure(figsize=(10, 8))
    ax2 = fig2.add_subplot(projection="3d")
    if cloud_xyz.shape[0] > 0:
        if cloud_rgb is not None and cloud_rgb.shape[0] == cloud_xyz.shape[0]:
            ax2.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c=cloud_rgb, s=0.25, alpha=0.045, depthshade=False)
        else:
            ax2.scatter(cloud_xyz[:, 0], cloud_xyz[:, 1], cloud_xyz[:, 2], c="lightgray", s=0.24, alpha=0.045, depthshade=False)
    free_sc = ax2.scatter([], [], [], c="deepskyblue", s=1.0, alpha=0.33, depthshade=False)
    ax2.set_xlim(xlim)
    ax2.set_ylim(ylim)
    ax2.set_zlim(zlim)
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_zlabel("z")
    ax2.view_init(elev=elev, azim=azim)

    try:
        writer = animation.PillowWriter(fps=max(1, int(fps)))
        with writer.saving(fig2, str(gif_path), dpi=140):
            for k in layer_ids:
                pts = sample_layer_points(k)
                if pts.shape[0] == 0:
                    free_sc._offsets3d = ([], [], [])
                else:
                    free_sc._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])
                ax2.set_title(f"Free-space ISO scan layer z={z_centers[k]:.2f}m (AGL {z_centers[k]-z_ground:.2f}m)")
                writer.grab_frame()
    except Exception as exc:
        log_progress(f"Free-space ISO scan GIF export skipped: {exc}")
    plt.close(fig2)
    return png_path, gif_path


def compute_blocked_mask_for_params(
    shape: np.ndarray,
    device_local: List[Tuple[str, np.ndarray]],
    local_unseg: np.ndarray,
    voxel_size: float,
    d0: float,
    sigma_gps: float,
    k_gps: float,
    d_ctrl: float,
    r_uav: float,
    sigma_model: float,
    risk_coeff: Dict[str, float],
    unsegmented_type: str,
    offset_cache: Dict[float, np.ndarray],
) -> np.ndarray:
    blocked_mask = np.zeros(tuple(int(v) for v in shape.tolist()), dtype=np.bool_)
    d_unc = k_gps * sigma_gps + d_ctrl
    d_geom = r_uav + 0.5 * voxel_size + sigma_model

    for dev_type, arr in device_local:
        if arr.shape[0] == 0:
            continue
        r = float(risk_coeff.get(dev_type, risk_coeff.get("default", 1.0)))
        d_eff = (r * d0) + d_unc + d_geom
        if d_eff not in offset_cache:
            offset_cache[d_eff] = build_sphere_offsets(d_eff, voxel_size)
        inflate_sparse_voxels_vectorized(arr, offset_cache[d_eff], shape, blocked_mask)

    if local_unseg.shape[0] > 0:
        unseg_r = float(risk_coeff.get(unsegmented_type, risk_coeff.get("default", 1.0)))
        unseg_d_eff = (unseg_r * d0) + d_unc + d_geom
        if unseg_d_eff not in offset_cache:
            offset_cache[unseg_d_eff] = build_sphere_offsets(unseg_d_eff, voxel_size)
        inflate_sparse_voxels_vectorized(local_unseg, offset_cache[unseg_d_eff], shape, blocked_mask)

    return blocked_mask


def compute_blocked_mask_for_static_d(
    shape: np.ndarray,
    all_local_sources: List[np.ndarray],
    static_d_eff: float,
    voxel_size: float,
    offset_cache: Dict[float, np.ndarray],
) -> np.ndarray:
    blocked_mask = np.zeros(tuple(int(v) for v in shape.tolist()), dtype=np.bool_)
    if static_d_eff not in offset_cache:
        offset_cache[static_d_eff] = build_sphere_offsets(static_d_eff, voxel_size)
    offsets = offset_cache[static_d_eff]
    for arr in all_local_sources:
        if arr.shape[0] == 0:
            continue
        inflate_sparse_voxels_vectorized(arr, offsets, shape, blocked_mask)
    return blocked_mask


def main() -> None:
    default_cfg = Path(__file__).resolve().parent / "configs" / "config.phase1.feasible_space.json"
    parser = argparse.ArgumentParser(description="Build station-level feasible voxel map from LAS clouds.")
    parser.add_argument("--config", type=Path, default=default_cfg)
    args = parser.parse_args()

    cfg = load_config(args.config)
    progress_cfg = cfg.get("progress", {})
    log_every_chunks = int(progress_cfg.get("log_every_chunks", 20))

    root = Path(cfg["paths"]["project_root"]).resolve()
    baseline_path = resolve_input_path(
        root=root,
        configured_path=cfg["paths"]["baseline_las"],
        fallback_dirs=["pointclouds", "."],
    )
    origin_path = resolve_input_path(
        root=root,
        configured_path=cfg["paths"].get("origin_las", "substation_origin.las"),
        fallback_dirs=["pointclouds", "."],
    )

    out_dir = root / cfg["paths"].get("output_dir", "outputs")
    out_dir.mkdir(parents=True, exist_ok=True)
    log_progress(f"Output directory: {out_dir}")
    log_progress(f"Baseline LAS: {baseline_path}")
    log_progress(f"Origin LAS (excluded from processing): {origin_path}")

    chunk_size = int(cfg["io"]["chunk_size"])
    voxel_size = float(cfg["grid"]["voxel_size"])
    pre_voxel_size = float(cfg["preprocess"]["voxel_size"])
    vis_cfg = cfg.get("visualization", {})
    gif_frames = int(vis_cfg.get("gif_frames", 72))
    gif_fps = int(vis_cfg.get("gif_fps", 12))
    gif_elev = float(vis_cfg.get("gif_elev", 22.0))
    overlay_cloud_max_points = int(vis_cfg.get("overlay_cloud_max_points", 120000))
    overlay_nofly_max_points = int(vis_cfg.get("overlay_nofly_max_points", 120000))
    overlay_free_max_points = int(vis_cfg.get("overlay_free_max_points", 100000))
    mission_altitude_agl = float(vis_cfg.get("mission_altitude_agl", 2.5))
    free_slice_half_thickness = float(vis_cfg.get("free_slice_half_thickness", 0.45))
    mid_altitude_agl = float(vis_cfg.get("mid_altitude_agl", 10.0))
    mid_slice_half_thickness = float(vis_cfg.get("mid_slice_half_thickness", 0.75))
    type_plot_max_points = int(vis_cfg.get("device_type_plot_max_points", 35000))
    type_bbox_max = int(vis_cfg.get("device_type_bbox_max", 80))
    type_colors = vis_cfg.get(
        "type_colors",
        {
            "transformer": "#ff3b30",
            "wires": "#ff9500",
            "shelf": "#34c759",
            "tube": "#00c7be",
            "capacitor": "#007aff",
            "building": "#8e8e93",
            "default": "#9b59b6",
        },
    )
    type_colors = {str(k): mcolors.to_hex(str(v), keep_alpha=False) for k, v in type_colors.items()}
    legend_display_names = vis_cfg.get(
        "legend_display_names",
        {
            "transformer": "main transformer",
            "building": "buildings",
            "tube": "GIS tube",
        },
    )
    legend_display_names = {str(k): str(v) for k, v in legend_display_names.items()}
    gif_name = str(vis_cfg.get("gif_name", "cloud_nofly_rotate.gif"))
    compare_gif_name = str(vis_cfg.get("compare_gif_name", "cloud_nofly_free_compare_rotate.gif"))
    types_gif_name = str(vis_cfg.get("types_gif_name", "device_types_rotate.gif"))
    free_bands_gif_name = str(vis_cfg.get("free_bands_gif_name", "cloud_free_bands_rotate.gif"))
    free_scan_stride = int(vis_cfg.get("free_scan_stride", 1))
    free_scan_fps = int(vis_cfg.get("free_scan_fps", 12))
    free_scan_gif_name = str(vis_cfg.get("free_scan_gif_name", "free_space_layer_scan.gif"))
    free_scan_iso_max_points = int(vis_cfg.get("free_scan_iso_max_points", 70000))
    free_scan_iso_elev = float(vis_cfg.get("free_scan_iso_elev", 22.0))
    free_scan_iso_azim = float(vis_cfg.get("free_scan_iso_azim", 25.0))
    free_scan_iso_fps = int(vis_cfg.get("free_scan_iso_fps", 12))
    free_scan_iso_gif_name = str(vis_cfg.get("free_scan_iso_gif_name", "free_space_layer_scan_iso.gif"))

    log_progress("Stage 1/12: sample baseline points for XY rotation estimation")
    sample_pts = random_sample_points_from_las(
        baseline_path,
        max_points=int(cfg["preprocess"]["rotation_sample_max_points"]),
        chunk_size=chunk_size,
        seed=int(cfg["random_seed"]),
        log_every_chunks=log_every_chunks,
        progress_name="rotation_sample",
    )
    angle_rad = estimate_z_rotation_angle(sample_pts)
    log_progress(f"Rotation angle estimated: {np.degrees(angle_rad):.3f} deg")

    log_progress("Stage 2/12: compute rotated bounds + ground height")
    bounds_rot, z_ground = compute_baseline_stats(
        baseline_path=baseline_path,
        angle_rad=angle_rad,
        chunk_size=chunk_size,
        ground_quantile=float(cfg["preprocess"]["ground_quantile"]),
        z_sample_max=int(cfg["preprocess"]["ground_sample_max_points"]),
        seed=int(cfg["random_seed"]) + 11,
        log_every_chunks=log_every_chunks,
        progress_name="baseline_stats",
    )

    h_min = float(cfg["flight_volume"]["h_min"])
    h_top = float(cfg["flight_volume"]["h_top"])
    z_min = z_ground + h_min
    z_max = float(bounds_rot[1, 2] + h_top)

    x_min, y_min = bounds_rot[0, 0], bounds_rot[0, 1]
    x_max, y_max = bounds_rot[1, 0], bounds_rot[1, 1]
    flight_bounds = np.array([[x_min, y_min, z_min], [x_max, y_max, z_max]], dtype=np.float64)
    log_progress(f"Flight volume z-range: [{z_min:.3f}, {z_max:.3f}]")

    log_progress("Stage 3/12: voxelize baseline obstacles in flight volume")
    base_occ_abs = build_voxel_set_from_las(
        las_path=baseline_path,
        angle_rad=angle_rad,
        voxel_size=voxel_size,
        chunk_size=chunk_size,
        bounds=flight_bounds,
        z_floor=z_ground + float(cfg["preprocess"]["obstacle_z_margin"]),
        log_every_chunks=log_every_chunks,
        progress_name="baseline_obstacles",
    )

    log_progress("Stage 4/12: build preprocessed point cloud for visualization")
    preproc_abs = build_voxel_set_from_las(
        las_path=baseline_path,
        angle_rad=angle_rad,
        voxel_size=pre_voxel_size,
        chunk_size=chunk_size,
        bounds=np.array([[x_min, y_min, z_ground + float(cfg["preprocess"]["preprocess_z_margin"])], [x_max, y_max, bounds_rot[1, 2]]], dtype=np.float64),
        z_floor=z_ground + float(cfg["preprocess"]["preprocess_z_margin"]),
        log_every_chunks=log_every_chunks,
        progress_name="baseline_preprocess",
    )

    log_progress("Stage 5/12: discover segmented device LAS files")
    device_search_dirs = cfg.get("paths", {}).get("device_search_dirs", ["."])
    device_recursive = bool(cfg.get("paths", {}).get("device_recursive", False))
    device_files = discover_device_files(
        root=root,
        baseline_name=baseline_path.name,
        origin_name=origin_path.name,
        search_dirs=device_search_dirs,
        recursive=device_recursive,
    )
    log_progress(f"Discovered {len(device_files)} device LAS files")
    type_keywords = cfg["device_type_keywords"]
    risk_coeff = cfg["risk_coefficients"]
    type_priority = {str(k): int(v) for k, v in cfg.get("type_priority", {"wires": 100, "default": 0}).items()}

    d0 = float(cfg["safety"]["d0"])
    sigma_gps = float(cfg["safety"]["sigma_gps"])
    k_gps = float(cfg["safety"]["k_gps"])
    d_ctrl = float(cfg["safety"]["d_ctrl"])
    r_uav = float(cfg["safety"]["r_uav"])
    sigma_model = float(cfg["safety"]["sigma_model"])

    d_unc = k_gps * sigma_gps + d_ctrl
    d_geom = r_uav + 0.5 * voxel_size + sigma_model

    devices: List[DeviceCloud] = []

    for idx_dev, f in enumerate(device_files, start=1):
        log_progress(f"Device {idx_dev}/{len(device_files)}: {f.name}")
        dev_type = infer_device_type(f.stem, type_keywords)
        r = float(risk_coeff.get(dev_type, risk_coeff.get("default", 1.0)))
        d_elec = r * d0
        d_eff = d_elec + d_unc + d_geom

        vox_abs = build_voxel_set_from_las(
            las_path=f,
            angle_rad=angle_rad,
            voxel_size=voxel_size,
            chunk_size=chunk_size,
            bounds=flight_bounds,
            z_floor=z_ground + float(cfg["preprocess"]["obstacle_z_margin"]),
            log_every_chunks=log_every_chunks,
            progress_name=f"device_{f.stem}",
        )
        devices.append(DeviceCloud(name=f.stem, path=f, device_type=dev_type, d_eff=d_eff, voxels_abs=vox_abs))

    owner: Dict[Voxel, Tuple[int, float, int]] = {}
    for dev_idx, dev in enumerate(devices):
        pr = get_type_priority(dev.device_type, type_priority)
        for v in dev.voxels_abs:
            prev = owner.get(v)
            if prev is None or pr > prev[0] or (pr == prev[0] and dev.d_eff > prev[1]):
                owner[v] = (pr, dev.d_eff, dev_idx)

    owned_sets: List[Set[Voxel]] = [set() for _ in devices]
    for v, (_, _, dev_idx) in owner.items():
        owned_sets[dev_idx].add(v)
    overlap_resolved = 0
    for i, dev in enumerate(devices):
        raw_n = len(dev.voxels_abs)
        owned_n = len(owned_sets[i])
        overlap_resolved += max(0, raw_n - owned_n)
        dev.voxels_abs = owned_sets[i]

    all_device_abs: Set[Voxel] = set(owner.keys())
    unsegmented_abs = base_occ_abs.difference(all_device_abs)
    wire_min_z = z_ground + float(cfg["safety"]["wire_min_height_from_ground"])
    wire_min_k = int(math.floor(wire_min_z / voxel_size))
    unsegmented_abs = {v for v in unsegmented_abs if v[2] >= wire_min_k}

    origin = np.array([x_min, y_min, z_min], dtype=np.float64)
    max_corner = np.array([x_max, y_max, z_max], dtype=np.float64)
    shape = np.ceil((max_corner - origin) / voxel_size).astype(np.int32) + 1
    base_idx = np.floor(origin / voxel_size).astype(np.int32)

    device_local_arrays: List[Tuple[str, str, np.ndarray]] = []
    for dev in devices:
        local_arr = abs_to_local_voxel_array(dev.voxels_abs, base_idx, shape)
        device_local_arrays.append((dev.name, dev.device_type, local_arr))
    local_unseg = abs_to_local_voxel_array(unsegmented_abs, base_idx, shape)

    blocked_mask = np.zeros(tuple(int(v) for v in shape.tolist()), dtype=np.bool_)
    offset_cache: Dict[float, np.ndarray] = {}

    log_progress("Stage 6/12: inflate device keep-out volumes")
    for idx_dev, dev in enumerate(devices, start=1):
        t_inflate = time.time()
        local_vox = device_local_arrays[idx_dev - 1][2]
        if local_vox.shape[0] == 0:
            log_progress(f"Inflate {idx_dev}/{len(devices)} {dev.name}: skip (empty)")
            continue
        if dev.d_eff not in offset_cache:
            offset_cache[dev.d_eff] = build_sphere_offsets(dev.d_eff, voxel_size)
        inflate_sparse_voxels_vectorized(local_vox, offset_cache[dev.d_eff], shape, blocked_mask)
        log_progress(
            f"Inflate {idx_dev}/{len(devices)} {dev.name}: d_eff={dev.d_eff:.3f}, "
            f"offsets={offset_cache[dev.d_eff].shape[0]}, blocked_now={int(blocked_mask.sum())}, "
            f"elapsed={format_seconds(time.time() - t_inflate)}"
        )

    if unsegmented_abs:
        log_progress("Stage 7/12: inflate unsegmented high-altitude obstacles (wire candidates)")
        unseg_type = cfg["safety"].get("unsegmented_type", "gantry")
        unseg_r = float(risk_coeff.get(unseg_type, risk_coeff.get("default", 1.0)))
        unseg_d_eff = (unseg_r * d0) + d_unc + d_geom
        if local_unseg.shape[0] > 0:
            t_unseg = time.time()
            if unseg_d_eff not in offset_cache:
                offset_cache[unseg_d_eff] = build_sphere_offsets(unseg_d_eff, voxel_size)
            inflate_sparse_voxels_vectorized(local_unseg, offset_cache[unseg_d_eff], shape, blocked_mask)
            log_progress(
                f"Inflate unsegmented: d_eff={unseg_d_eff:.3f}, offsets={offset_cache[unseg_d_eff].shape[0]}, "
                f"blocked_now={int(blocked_mask.sum())}, elapsed={format_seconds(time.time() - t_unseg)}"
            )
    else:
        unseg_d_eff = None
        log_progress("Stage 7/12: no unsegmented high-altitude obstacles found")

    blocked_arr = np.argwhere(blocked_mask).astype(np.int32)

    log_progress("Stage 8/12: write voxel map and base visualization assets")
    np.savez_compressed(
        out_dir / "station_grid_map.npz",
        origin=origin,
        voxel_size=np.array([voxel_size], dtype=np.float64),
        shape=shape,
        blocked=blocked_arr,
    )

    if preproc_abs:
        preproc_idx = np.array(sorted(preproc_abs), dtype=np.int32)
        preproc_pts = (preproc_idx.astype(np.float64) + 0.5) * pre_voxel_size
        write_pcd_xyz_binary(out_dir / "substation_preprocessed.pcd", preproc_pts)

    visualize_blocked_voxels(
        blocked_idx=blocked_arr,
        origin=origin,
        voxel_size=voxel_size,
        out_dir=out_dir,
        max_plot_points=int(cfg["visualization"]["max_plot_points"]),
        seed=int(cfg["random_seed"]) + 101,
    )
    log_progress("Stage 8.5/12: export segmented type overview")
    types_png_path, types_gif_path = visualize_device_types_and_gif(
        devices=devices,
        voxel_size=voxel_size,
        out_dir=out_dir,
        seed=int(cfg["random_seed"]) + 151,
        max_points_per_type=type_plot_max_points,
        max_bboxes=type_bbox_max,
        type_colors=type_colors,
        legend_display_names=legend_display_names,
        gif_frames=gif_frames,
        gif_fps=gif_fps,
        elev=gif_elev,
        gif_name=types_gif_name,
    )

    log_progress("Stage 9/12: sample baseline cloud for overlay visualization")
    overlay_cloud_xyz, overlay_cloud_rgb = sample_visual_cloud_from_las(
        las_path=baseline_path,
        angle_rad=angle_rad,
        max_points=overlay_cloud_max_points,
        chunk_size=chunk_size,
        seed=int(cfg["random_seed"]) + 202,
        bounds=flight_bounds,
        z_floor=z_ground + float(cfg["preprocess"]["preprocess_z_margin"]),
        log_every_chunks=log_every_chunks,
        progress_name="overlay_cloud",
    )

    log_progress("Stage 10/12: export no-fly overlay PNG and rotating GIF")
    blocked_xyz_for_overlay = sample_blocked_voxel_points(
        blocked_idx=blocked_arr,
        origin=origin,
        voxel_size=voxel_size,
        max_points=overlay_nofly_max_points,
        seed=int(cfg["random_seed"]) + 303,
    )
    overlay_png_path, overlay_gif_path = visualize_overlay_and_gif(
        cloud_xyz=overlay_cloud_xyz,
        cloud_rgb=overlay_cloud_rgb,
        blocked_xyz=blocked_xyz_for_overlay,
        out_dir=out_dir,
        gif_frames=gif_frames,
        gif_fps=gif_fps,
        elev=gif_elev,
        gif_name=gif_name,
    )

    log_progress("Stage 11/12: export no-fly vs free-space comparison PNG and rotating GIF")
    free_low_xyz_for_overlay = sample_free_voxel_points(
        blocked_mask=blocked_mask,
        origin=origin,
        voxel_size=voxel_size,
        max_points=overlay_free_max_points,
        seed=int(cfg["random_seed"]) + 404,
        z_range=(
            z_ground + mission_altitude_agl - free_slice_half_thickness,
            z_ground + mission_altitude_agl + free_slice_half_thickness,
        ),
    )
    free_mid_xyz_for_overlay = sample_free_voxel_points(
        blocked_mask=blocked_mask,
        origin=origin,
        voxel_size=voxel_size,
        max_points=overlay_free_max_points,
        seed=int(cfg["random_seed"]) + 405,
        z_range=(
            z_ground + mid_altitude_agl - mid_slice_half_thickness,
            z_ground + mid_altitude_agl + mid_slice_half_thickness,
        ),
    )
    free_full_xyz_for_overlay = sample_free_voxel_points(
        blocked_mask=blocked_mask,
        origin=origin,
        voxel_size=voxel_size,
        max_points=overlay_free_max_points,
        seed=int(cfg["random_seed"]) + 406,
        z_range=None,
    )
    compare_png_path, compare_gif_path = visualize_compare_spaces_and_gif(
        cloud_xyz=overlay_cloud_xyz,
        cloud_rgb=overlay_cloud_rgb,
        blocked_xyz=blocked_xyz_for_overlay,
        free_xyz=free_low_xyz_for_overlay,
        out_dir=out_dir,
        gif_frames=gif_frames,
        gif_fps=gif_fps,
        elev=gif_elev,
        gif_name=compare_gif_name,
    )
    free_bands_png_path, free_bands_gif_path = visualize_free_space_bands_and_gif(
        cloud_xyz=overlay_cloud_xyz,
        cloud_rgb=overlay_cloud_rgb,
        free_low_xyz=free_low_xyz_for_overlay,
        free_mid_xyz=free_mid_xyz_for_overlay,
        free_full_xyz=free_full_xyz_for_overlay,
        out_dir=out_dir,
        gif_frames=gif_frames,
        gif_fps=gif_fps,
        elev=gif_elev,
        gif_name=free_bands_gif_name,
    )
    log_progress("Stage 11.2/12: export free-space layer scan GIF")
    free_scan_png_path, free_scan_gif_path = visualize_free_space_layer_scan_gif(
        blocked_mask=blocked_mask,
        origin=origin,
        voxel_size=voxel_size,
        z_ground=z_ground,
        out_dir=out_dir,
        stride=free_scan_stride,
        fps=free_scan_fps,
        gif_name=free_scan_gif_name,
    )
    log_progress("Stage 11.3/12: export free-space ISO layer scan GIF")
    free_scan_iso_png_path, free_scan_iso_gif_path = visualize_free_space_layer_scan_iso_gif(
        blocked_mask=blocked_mask,
        cloud_xyz=overlay_cloud_xyz,
        cloud_rgb=overlay_cloud_rgb,
        origin=origin,
        voxel_size=voxel_size,
        z_ground=z_ground,
        out_dir=out_dir,
        stride=free_scan_stride,
        fps=free_scan_iso_fps,
        elev=free_scan_iso_elev,
        azim=free_scan_iso_azim,
        max_free_points_per_layer=free_scan_iso_max_points,
        seed=int(cfg["random_seed"]) + 507,
        gif_name=free_scan_iso_gif_name,
    )

    total_voxels = int(shape[0] * shape[1] * shape[2])
    blocked_voxels = int(blocked_arr.shape[0])
    free_voxels = max(total_voxels - blocked_voxels, 0)
    sweep_summary: Optional[Dict[str, Any]] = None

    sweep_cfg = cfg.get("sweep_analysis", {})
    if bool(sweep_cfg.get("enabled", False)):
        log_progress("Stage 11.5/12: run parameter sweep and export comparative maps")
        sweep_dir = out_dir / str(sweep_cfg.get("output_subdir", "sweep_maps"))
        sweep_dir.mkdir(parents=True, exist_ok=True)

        export_npz = bool(sweep_cfg.get("export_npz", True))
        low_alt_agl = float(sweep_cfg.get("metric_low_altitude_agl", mission_altitude_agl))
        mid_alt_agl = float(sweep_cfg.get("metric_mid_altitude_agl", mid_altitude_agl))
        low_abs = z_ground + low_alt_agl
        mid_abs = z_ground + mid_alt_agl
        z_centers = origin[2] + (np.arange(int(shape[2]), dtype=np.float64) + 0.5) * voxel_size
        k_low = int(np.argmin(np.abs(z_centers - low_abs)))
        k_mid = int(np.argmin(np.abs(z_centers - mid_abs)))

        dynamic_cases_cfg = sweep_cfg.get("dynamic_cases", [])
        dynamic_cases: List[Dict[str, float]] = []
        if isinstance(dynamic_cases_cfg, list) and dynamic_cases_cfg:
            for i, c in enumerate(dynamic_cases_cfg):
                if not isinstance(c, dict):
                    continue
                name = str(c.get("name", f"dyn_{i+1}"))
                dynamic_cases.append(
                    {
                        "name": name,
                        "d0": float(c.get("d0", d0)),
                        "k_gps": float(c.get("k_gps", k_gps)),
                        "d_ctrl": float(c.get("d_ctrl", d_ctrl)),
                    }
                )
        else:
            d0_values = sweep_cfg.get("d0_values", [d0])
            for i, val in enumerate(d0_values):
                dv = float(val)
                dynamic_cases.append({"name": f"d0_{dv:.2f}", "d0": dv, "k_gps": k_gps, "d_ctrl": d_ctrl})

        static_d_eff_values = [float(v) for v in sweep_cfg.get("static_d_eff_values", [])]
        sweep_rows: List[Dict[str, Any]] = []
        sweep_offset_cache: Dict[float, np.ndarray] = {}

        device_local_for_sweep = [(dev_type, arr) for _, dev_type, arr in device_local_arrays]
        all_local_sources = [arr for _, _, arr in device_local_arrays]
        if local_unseg.shape[0] > 0:
            all_local_sources.append(local_unseg)

        for case in dynamic_cases:
            mask = compute_blocked_mask_for_params(
                shape=shape,
                device_local=device_local_for_sweep,
                local_unseg=local_unseg,
                voxel_size=voxel_size,
                d0=float(case["d0"]),
                sigma_gps=sigma_gps,
                k_gps=float(case["k_gps"]),
                d_ctrl=float(case["d_ctrl"]),
                r_uav=r_uav,
                sigma_model=sigma_model,
                risk_coeff=risk_coeff,
                unsegmented_type=cfg["safety"].get("unsegmented_type", "wires"),
                offset_cache=sweep_offset_cache,
            )
            blocked_n = int(mask.sum())
            free_n = int(mask.size - blocked_n)
            by_z = mask.sum(axis=(0, 1))
            free_ratio_low = float((shape[0] * shape[1] - by_z[k_low]) / (shape[0] * shape[1]))
            free_ratio_mid = float((shape[0] * shape[1] - by_z[k_mid]) / (shape[0] * shape[1]))
            case_name = sanitize_case_name(str(case["name"]))
            npz_path = sweep_dir / f"station_grid_map_dynamic_{case_name}.npz"
            if export_npz:
                np.savez_compressed(
                    npz_path,
                    origin=origin,
                    voxel_size=np.array([voxel_size], dtype=np.float64),
                    shape=shape,
                    blocked=np.argwhere(mask).astype(np.int32),
                )
            sweep_rows.append(
                {
                    "mode": "dynamic",
                    "name": str(case["name"]),
                    "d0": float(case["d0"]),
                    "k_gps": float(case["k_gps"]),
                    "d_ctrl": float(case["d_ctrl"]),
                    "blocked_voxels": blocked_n,
                    "free_voxels": free_n,
                    "free_pct": 100.0 * free_n / max(1, mask.size),
                    "free_ratio_low": free_ratio_low,
                    "free_ratio_mid": free_ratio_mid,
                    "npz_path": str(npz_path) if export_npz else None,
                }
            )

        for static_d_eff in static_d_eff_values:
            mask = compute_blocked_mask_for_static_d(
                shape=shape,
                all_local_sources=all_local_sources,
                static_d_eff=static_d_eff,
                voxel_size=voxel_size,
                offset_cache=sweep_offset_cache,
            )
            blocked_n = int(mask.sum())
            free_n = int(mask.size - blocked_n)
            by_z = mask.sum(axis=(0, 1))
            free_ratio_low = float((shape[0] * shape[1] - by_z[k_low]) / (shape[0] * shape[1]))
            free_ratio_mid = float((shape[0] * shape[1] - by_z[k_mid]) / (shape[0] * shape[1]))
            case_name = sanitize_case_name(f"static_{static_d_eff:.2f}")
            npz_path = sweep_dir / f"station_grid_map_static_{case_name}.npz"
            if export_npz:
                np.savez_compressed(
                    npz_path,
                    origin=origin,
                    voxel_size=np.array([voxel_size], dtype=np.float64),
                    shape=shape,
                    blocked=np.argwhere(mask).astype(np.int32),
                )
            sweep_rows.append(
                {
                    "mode": "static",
                    "name": f"static_d{static_d_eff:.2f}",
                    "d0": None,
                    "k_gps": None,
                    "d_ctrl": None,
                    "static_d_eff": static_d_eff,
                    "blocked_voxels": blocked_n,
                    "free_voxels": free_n,
                    "free_pct": 100.0 * free_n / max(1, mask.size),
                    "free_ratio_low": free_ratio_low,
                    "free_ratio_mid": free_ratio_mid,
                    "npz_path": str(npz_path) if export_npz else None,
                }
            )

        sweep_plot_path = sweep_dir / str(sweep_cfg.get("plot_name", "sweep_free_space_compare.png"))
        sweep_json_path = sweep_dir / str(sweep_cfg.get("summary_name", "sweep_summary.json"))
        sweep_csv_path = sweep_dir / str(sweep_cfg.get("csv_name", "sweep_summary.csv"))
        if sweep_rows:
            labels = [r["name"] for r in sweep_rows]
            x = np.arange(len(labels), dtype=np.float64)
            free_pct_vals = np.array([r["free_pct"] for r in sweep_rows], dtype=np.float64)
            low_vals = np.array([100.0 * r["free_ratio_low"] for r in sweep_rows], dtype=np.float64)
            mid_vals = np.array([100.0 * r["free_ratio_mid"] for r in sweep_rows], dtype=np.float64)
            bar_colors = ["#007aff" if r["mode"] == "dynamic" else "#8e8e93" for r in sweep_rows]

            fig = plt.figure(figsize=(max(10.0, 1.3 * len(labels)), 6))
            ax = fig.add_subplot(111)
            ax.bar(x, free_pct_vals, color=bar_colors, alpha=0.78, width=0.62, label="Overall free %")
            ax.plot(x, low_vals, color="#ff3b30", marker="o", linewidth=1.7, label=f"Free % @ {low_alt_agl:.1f}m AGL")
            ax.plot(x, mid_vals, color="#34c759", marker="s", linewidth=1.7, label=f"Free % @ {mid_alt_agl:.1f}m AGL")
            ax.set_xticks(x)
            ax.set_xticklabels(labels, rotation=30, ha="right")
            ax.set_ylabel("Free-space percentage (%)")
            ax.set_title("Parameter sweep: dynamic vs static safety configuration")
            ax.grid(True, axis="y", alpha=0.25)
            ax.legend(loc="best")
            fig.tight_layout()
            fig.savefig(sweep_plot_path, dpi=220)
            plt.close(fig)

            csv_fields = [
                "mode",
                "name",
                "d0",
                "k_gps",
                "d_ctrl",
                "static_d_eff",
                "blocked_voxels",
                "free_voxels",
                "free_pct",
                "free_ratio_low",
                "free_ratio_mid",
                "npz_path",
            ]
            with sweep_csv_path.open("w", encoding="utf-8-sig", newline="") as f_csv:
                writer = csv.DictWriter(f_csv, fieldnames=csv_fields)
                writer.writeheader()
                for row in sweep_rows:
                    writer.writerow({k: row.get(k, None) for k in csv_fields})
        else:
            csv_fields = [
                "mode",
                "name",
                "d0",
                "k_gps",
                "d_ctrl",
                "static_d_eff",
                "blocked_voxels",
                "free_voxels",
                "free_pct",
                "free_ratio_low",
                "free_ratio_mid",
                "npz_path",
            ]
            with sweep_csv_path.open("w", encoding="utf-8-sig", newline="") as f_csv:
                writer = csv.DictWriter(f_csv, fieldnames=csv_fields)
                writer.writeheader()

        sweep_summary = {
            "enabled": True,
            "low_altitude_agl": low_alt_agl,
            "mid_altitude_agl": mid_alt_agl,
            "rows": sweep_rows,
            "plot_path": str(sweep_plot_path),
            "summary_path": str(sweep_json_path),
            "csv_path": str(sweep_csv_path),
            "output_dir": str(sweep_dir),
        }
        with sweep_json_path.open("w", encoding="utf-8") as f:
            json.dump(sweep_summary, f, ensure_ascii=False, indent=2)

    report = {
        "inputs": {
            "baseline": str(baseline_path),
            "device_files": [str(d.path) for d in devices],
        },
        "rotation": {
            "angle_rad": angle_rad,
            "angle_deg": float(np.degrees(angle_rad)),
        },
        "flight_volume": {
            "bounds_rotated": {
                "x_min": float(x_min),
                "x_max": float(x_max),
                "y_min": float(y_min),
                "y_max": float(y_max),
                "z_min": float(z_min),
                "z_max": float(z_max),
            },
            "z_ground": float(z_ground),
        },
        "safety": {
            "d0": d0,
            "d_unc": d_unc,
            "d_geom": d_geom,
            "unsegmented_d_eff": unseg_d_eff,
            "inspectable_device_types": cfg.get(
                "inspectable_device_types",
                ["transformer", "shelf", "tube", "wires", "capacitor"],
            ),
            "devices": [
                {
                    "name": d.name,
                    "type": d.device_type,
                    "d_eff": d.d_eff,
                    "occupied_voxel_count": len(d.voxels_abs),
                }
                for d in devices
            ],
        },
        "grid": {
            "voxel_size": voxel_size,
            "shape": shape.tolist(),
            "total_voxels": total_voxels,
            "blocked_voxels": blocked_voxels,
            "free_voxels": free_voxels,
        },
        "intermediate": {
            "baseline_occupied_voxels": len(base_occ_abs),
            "unsegmented_candidate_voxels": len(unsegmented_abs),
            "overlap_resolved_voxels": int(overlap_resolved),
            "preprocessed_voxels": len(preproc_abs),
            "overlay_cloud_points": int(overlay_cloud_xyz.shape[0]),
            "overlay_nofly_points": int(blocked_xyz_for_overlay.shape[0]),
            "overlay_free_low_points": int(free_low_xyz_for_overlay.shape[0]),
            "overlay_free_mid_points": int(free_mid_xyz_for_overlay.shape[0]),
            "overlay_free_full_points": int(free_full_xyz_for_overlay.shape[0]),
        },
        "outputs": {
            "grid_npz": str(out_dir / "station_grid_map.npz"),
            "report_json": str(out_dir / "feasible_space_report.json"),
            "preprocessed_pcd": str(out_dir / "substation_preprocessed.pcd"),
            "plot_3d": str(out_dir / "no_fly_zones_3d.png"),
            "plot_top": str(out_dir / "no_fly_zones_topview.png"),
            "types_png": str(types_png_path),
            "types_gif": str(types_gif_path),
            "overlay_png": str(overlay_png_path),
            "overlay_gif": str(overlay_gif_path),
            "compare_png": str(compare_png_path),
            "compare_gif": str(compare_gif_path),
            "free_bands_png": str(free_bands_png_path),
            "free_bands_gif": str(free_bands_gif_path),
            "free_scan_png": str(free_scan_png_path),
            "free_scan_gif": str(free_scan_gif_path),
            "free_scan_iso_png": str(free_scan_iso_png_path),
            "free_scan_iso_gif": str(free_scan_iso_gif_path),
        },
        "sweep": sweep_summary,
    }

    log_progress("Stage 12/12: write report")
    with (out_dir / "feasible_space_report.json").open("w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    log_progress("Build complete")
    print(json.dumps(report["grid"], ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()


