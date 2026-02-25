#!/usr/bin/env python3
"""Phase4 baseline runner (FC-Planner-Lite + PredRecon-Lite)."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


def log(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8-sig") as f:
        return json.load(f)


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def load_phase2_module(root: Path):
    p = root / "phases" / "phase2_single_device" / "build_single_device_inspection.py"
    spec = importlib.util.spec_from_file_location("phase2_module", p)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load phase2 module: {p}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def clone_vp(p2: Any, v: Any) -> Any:
    return p2.Viewpoint(
        vp_id=int(v.vp_id),
        device=str(v.device),
        device_type=str(v.device_type),
        x=float(v.x),
        y=float(v.y),
        z=float(v.z),
        yaw_deg=float(v.yaw_deg),
        pitch_deg=float(v.pitch_deg),
        roll_deg=float(v.roll_deg),
        target_x=float(v.target_x),
        target_y=float(v.target_y),
        target_z=float(v.target_z),
        layer_k=int(v.layer_k),
        component_id=int(v.component_id),
        theta=float(v.theta),
    )


def make_vp(p2: Any, i: int, name: str, typ: str, pos: np.ndarray, target: np.ndarray, layer: int, comp: int) -> Any:
    yaw = math.degrees(math.atan2(float(target[1] - pos[1]), float(target[0] - pos[0])))
    return p2.Viewpoint(
        vp_id=int(i),
        device=name,
        device_type=typ,
        x=float(pos[0]),
        y=float(pos[1]),
        z=float(pos[2]),
        yaw_deg=float(yaw),
        pitch_deg=0.0,
        roll_deg=0.0,
        target_x=float(target[0]),
        target_y=float(target[1]),
        target_z=float(target[2]),
        layer_k=int(layer),
        component_id=int(comp),
        theta=0.0,
    )


def sample_surface_world(p2: Any, dv: Any, max_points: int, seed: int) -> np.ndarray:
    surf = np.argwhere(dv.surface_mask).astype(np.int32)
    if surf.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)
    if surf.shape[0] > max_points:
        rng = np.random.default_rng(seed)
        pick = rng.choice(surf.shape[0], size=max_points, replace=False)
        surf = surf[pick]
    return p2.local_idx_to_world(dv.min_abs_idx, surf, dv.voxel_size)


def dedup_vps(p2: Any, vps: Sequence[Any], min_spacing_m: float) -> List[Any]:
    out: List[Any] = []
    dmin = max(1e-6, float(min_spacing_m))
    for v in vps:
        p = np.array([v.x, v.y, v.z], dtype=np.float64)
        ok = True
        for q in out:
            qq = np.array([q.x, q.y, q.z], dtype=np.float64)
            if float(np.linalg.norm(p - qq)) < dmin:
                ok = False
                break
        if ok:
            out.append(clone_vp(p2, v))
    for i, v in enumerate(out):
        v.vp_id = i
    return out


def generate_fc_lite(p2: Any, dv: Any, name: str, typ: str, standoff: float, cfg: dict, seed: int) -> List[Any]:
    surf = sample_surface_world(p2, dv, int(cfg.get("max_surface_points", 5000)), seed)
    if surf.shape[0] == 0:
        return []
    c = surf.mean(axis=0)
    zmin = float(surf[:, 2].min())
    zmax = float(surf[:, 2].max())
    levels = max(1, int(cfg.get("num_levels", 6)))
    bins = max(4, int(cfg.get("azimuth_bins", 12)))
    z_band = max(float(dv.voxel_size) * 2.0, float(cfg.get("z_band_m", 0.6)))
    max_v = max(1, int(cfg.get("max_viewpoints", 180)))
    vps: List[Any] = []
    k = 0
    for li, z in enumerate(np.linspace(zmin, zmax, levels, endpoint=True).tolist()):
        if len(vps) >= max_v:
            break
        m = np.abs(surf[:, 2] - z) <= z_band
        pts = surf[m]
        if pts.shape[0] < 4:
            continue
        vec = pts[:, :2] - c[:2][None, :]
        ang = np.arctan2(vec[:, 1], vec[:, 0])
        for bi in range(bins):
            if len(vps) >= max_v:
                break
            a0 = -math.pi + 2.0 * math.pi * bi / bins
            a1 = -math.pi + 2.0 * math.pi * (bi + 1) / bins
            ids = np.where((ang >= a0) & (ang < a1))[0]
            if ids.size == 0:
                continue
            rr = np.linalg.norm(vec[ids], axis=1)
            idx = int(ids[int(np.argmax(rr))])
            tgt = pts[idx]
            d = tgt - c
            n = float(np.linalg.norm(d))
            if n < 1e-6:
                continue
            pos = tgt + (d / n) * float(standoff)
            vps.append(make_vp(p2, k, name, typ, pos, tgt, li, bi))
            k += 1
    vps = dedup_vps(p2, vps, float(cfg.get("min_viewpoint_spacing_m", 0.8)))
    if len(vps) > max_v:
        vps = vps[:max_v]
        for i, v in enumerate(vps):
            v.vp_id = i
    return vps


def generate_pred_lite(
    p2: Any,
    dv: Any,
    name: str,
    typ: str,
    standoff: float,
    cam: dict,
    cfg: dict,
    seed: int,
) -> List[Any]:
    surf = sample_surface_world(p2, dv, int(cfg.get("max_surface_points", 2400)), seed)
    if surf.shape[0] == 0:
        return []
    rng = np.random.default_rng(seed + 97)
    c = surf.mean(axis=0)
    max_c = max(8, int(cfg.get("max_candidates", 140)))
    cand_ids = rng.choice(surf.shape[0], size=min(max_c, surf.shape[0]), replace=False)
    cand_pos: List[np.ndarray] = []
    cand_tgt: List[np.ndarray] = []
    for i in cand_ids.tolist():
        tgt = surf[i]
        d = tgt - c
        n = float(np.linalg.norm(d))
        if n < 1e-6:
            continue
        cand_pos.append(tgt + (d / n) * float(standoff))
        cand_tgt.append(tgt)
    if not cand_pos:
        return []
    vis = np.zeros((len(cand_pos), surf.shape[0]), dtype=np.bool_)
    for i in range(len(cand_pos)):
        v = make_vp(p2, 0, name, typ, cand_pos[i], cand_tgt[i], 0, i)
        vis[i] = p2.point_in_frustum_mask(
            surf,
            v,
            hfov_deg=float(cam["hfov_deg"]),
            vfov_deg=float(cam["vfov_deg"]),
            near_m=float(cam.get("near_m", 0.3)),
            far_m=float(cam.get("far_m", 30.0)),
        )
    max_v = max(1, int(cfg.get("max_viewpoints", 120)))
    min_new = max(1, int(cfg.get("min_new_visible_points", 20)))
    uncovered = np.ones((surf.shape[0],), dtype=np.bool_)
    used = np.zeros((len(cand_pos),), dtype=np.bool_)
    selected: List[int] = []
    for _ in range(max_v):
        scores = vis[:, uncovered].sum(axis=1).astype(np.int64)
        scores[used] = -1
        best = int(np.argmax(scores))
        if int(scores[best]) < min_new:
            break
        selected.append(best)
        used[best] = True
        uncovered &= (~vis[best])
        if float(uncovered.mean()) <= float(cfg.get("target_uncovered_ratio", 0.06)):
            break
    if not selected:
        selected = list(range(min(max_v, max(4, len(cand_pos) // 12))))
    vps = [make_vp(p2, i, name, typ, cand_pos[ci], cand_tgt[ci], i, ci) for i, ci in enumerate(selected)]
    vps = dedup_vps(p2, vps, float(cfg.get("min_viewpoint_spacing_m", 0.8)))
    if len(vps) > max_v:
        vps = vps[:max_v]
        for i, v in enumerate(vps):
            v.vp_id = i
    return vps


def nearest_free_world(station: Any, p: np.ndarray, max_radius_steps: int) -> Optional[np.ndarray]:
    idx = station.world_to_idx(p)
    if station.in_bounds_idx(idx) and (not bool(station.blocked[idx[0], idx[1], idx[2]])):
        return p.copy()
    idx = np.clip(idx, 0, station.shape - 1)
    max_r = max(1, int(max_radius_steps))
    best = None
    best_d2 = float("inf")
    for r in range(1, max_r + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    if max(abs(dx), abs(dy), abs(dz)) != r:
                        continue
                    cand = np.array([idx[0] + dx, idx[1] + dy, idx[2] + dz], dtype=np.int32)
                    if not station.in_bounds_idx(cand):
                        continue
                    if bool(station.blocked[cand[0], cand[1], cand[2]]):
                        continue
                    w = station.idx_to_world_center(cand)
                    d2 = float(np.sum((w - p) ** 2))
                    if d2 < best_d2:
                        best_d2 = d2
                        best = w
        if best is not None:
            return best
    return None


def apply_scenario_constraints(p2: Any, station: Any, vps: Sequence[Any], scfg: dict) -> Tuple[List[Any], dict]:
    apply_on = bool(scfg.get("apply_phase1_constraints", False))
    handling = str(scfg.get("constraint_handling", "none")).lower().strip()
    max_r = int(scfg.get("project_max_radius_steps", 24))
    out: List[Any] = []
    raw_total = int(len(vps))
    raw_bad = 0
    projected = 0
    dropped = 0
    proj_dist: List[float] = []
    for v in vps:
        p = np.array([v.x, v.y, v.z], dtype=np.float64)
        ok = station.is_free_world(p)
        if not ok:
            raw_bad += 1
        if (not apply_on) or ok or handling == "none":
            out.append(clone_vp(p2, v))
            continue
        if handling == "drop":
            dropped += 1
            continue
        if handling == "project":
            q = nearest_free_world(station, p, max_r)
            if q is None:
                dropped += 1
                continue
            nv = clone_vp(p2, v)
            nv.x = float(q[0])
            nv.y = float(q[1])
            nv.z = float(q[2])
            nv.yaw_deg = float(math.degrees(math.atan2(float(nv.target_y - nv.y), float(nv.target_x - nv.x))))
            out.append(nv)
            projected += 1
            proj_dist.append(float(np.linalg.norm(q - p)))
            continue
        out.append(clone_vp(p2, v))
    for i, v in enumerate(out):
        v.vp_id = i
    stats = {
        "raw_total_count": raw_total,
        "raw_infeasible_count": raw_bad,
        "raw_infeasible_ratio": float(raw_bad / max(1, raw_total)),
        "projected_count": projected,
        "dropped_count": dropped,
        "avg_projection_distance_m": float(np.mean(proj_dist)) if proj_dist else 0.0,
        "max_projection_distance_m": float(np.max(proj_dist)) if proj_dist else 0.0,
        "apply_phase1_constraints": apply_on,
        "constraint_handling": handling,
    }
    return out, stats


def compute_path_violation(path_pts: np.ndarray, station: Any, step_scale: float = 0.8) -> Tuple[float, float]:
    if path_pts.shape[0] <= 1:
        return 0.0, 0.0
    total = 0.0
    vio = 0.0
    for i in range(path_pts.shape[0] - 1):
        a = path_pts[i]
        b = path_pts[i + 1]
        seg = float(np.linalg.norm(b - a))
        if seg <= 1e-9:
            continue
        total += seg
        n = max(2, int(math.ceil(seg / max(station.voxel * step_scale, 1e-6))))
        bad = 0
        for k in range(n + 1):
            t = float(k / n)
            p = a * (1.0 - t) + b * t
            if not station.is_free_world(p):
                bad += 1
        vio += seg * float(bad / (n + 1))
    if total <= 1e-9:
        return 0.0, 0.0
    return float(vio / total), float(vio)


def write_summary_csv(path: Path, rows: Sequence[dict]) -> None:
    keys = [
        "device",
        "type",
        "raw_viewpoints",
        "ordered_viewpoints",
        "raw_infeasible_ratio",
        "projected_count",
        "dropped_count",
        "coverage_ratio",
        "sampled_surface_points",
        "covered_surface_points",
        "path_length_m",
        "path_segment_success_ratio",
        "path_violation_length_ratio",
        "viewpoint_generation_time_s",
        "viewpoint_reorder_time_s",
        "coverage_eval_time_s",
        "path_planning_time_s",
        "device_total_runtime_s",
        "report_json",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def select_devices(root: Path, p2: Any, phase2_cfg: dict) -> List[Tuple[Path, str]]:
    files = p2.discover_device_las(
        root=root,
        pointcloud_dir=str(phase2_cfg["paths"]["pointcloud_dir"]),
        baseline_name=str(phase2_cfg["paths"]["baseline_name"]),
        origin_name=str(phase2_cfg["paths"]["origin_name"]),
        exclude_keywords=[str(x) for x in phase2_cfg.get("filters", {}).get("exclude_name_keywords", [])],
    )
    keywords = phase2_cfg["device_type_keywords"]
    exclude_types = {str(t) for t in phase2_cfg.get("filters", {}).get("exclude_types", [])}
    include_kw = [str(k).lower() for k in phase2_cfg.get("filters", {}).get("include_name_keywords", [])]
    out: List[Tuple[Path, str]] = []
    for p in files:
        t = p2.infer_device_type(p.stem, keywords)
        if t in exclude_types:
            continue
        if include_kw and (not any(k in p.stem.lower() for k in include_kw)):
            continue
        out.append((p, t))
    return out


def run_case(
    p2: Any,
    root: Path,
    station: Any,
    phase2_cfg: dict,
    eval_cfg: dict,
    angle_rad: float,
    deff_map: Dict[str, float],
    devices: Sequence[Tuple[Path, str]],
    method_name: str,
    method_cfg: dict,
    scenario_name: str,
    scenario_cfg: dict,
    out_dir: Path,
    base_seed: int,
) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    rows: List[dict] = []
    tot_cov_n = 0
    tot_cov_ok = 0
    tot_gen = 0.0
    tot_plan = 0.0
    tot_vp = 0
    tot_pairs = 0
    log(f"[{method_name}/{scenario_name}] output={out_dir}")
    for i, (las_path, typ) in enumerate(devices, start=1):
        t0 = time.perf_counter()
        name = las_path.stem
        seed = int(base_seed + i * 173)
        log(f"[{method_name}/{scenario_name}] {i}/{len(devices)} {name}")
        dv = p2.build_device_voxel(
            device_name=name,
            device_type=typ,
            las_path=las_path,
            voxel_size=float(phase2_cfg["generation"]["voxel_size_m"]),
            chunk_size=int(phase2_cfg["io"]["chunk_size"]),
            angle_rad=angle_rad,
            ground_filter_cfg=phase2_cfg.get("ground_filter", {}),
        )
        if dv is None:
            log(f"[{method_name}/{scenario_name}] skip {name}: empty voxel")
            continue
        standoff = p2.pick_standoff_m(name, typ, phase2_cfg["generation"]["standoff"], deff_map)
        standoff *= float(method_cfg.get("standoff_scale", 1.0))
        standoff = float(max(0.8, standoff))
        t_gen0 = time.perf_counter()
        gen = str(method_cfg.get("generator", method_name)).lower().strip()
        if gen == "fc_planner_lite":
            raw_vps = generate_fc_lite(p2, dv, name, typ, standoff, method_cfg.get("fc_params", {}), seed)
        elif gen == "predrecon_lite":
            raw_vps = generate_pred_lite(p2, dv, name, typ, standoff, eval_cfg["camera"], method_cfg.get("pred_params", {}), seed)
        else:
            raise ValueError(f"Unknown generator: {gen}")
        t_gen = float(time.perf_counter() - t_gen0)
        adapted_vps, feas = apply_scenario_constraints(p2, station, raw_vps, scenario_cfg)
        t_r0 = time.perf_counter()
        reorder = str(method_cfg.get("reorder_strategy", "none")).lower().strip()
        if reorder == "global_tsp":
            ordered_vps = p2.reorder_viewpoints_tsp_global([clone_vp(p2, v) for v in adapted_vps], two_opt_iter=int(phase2_cfg.get("reorder", {}).get("tsp_2opt_iter", 1)))
        elif reorder == "phase2_layered":
            ordered_vps = p2.reorder_viewpoints([clone_vp(p2, v) for v in adapted_vps], reorder_cfg=phase2_cfg.get("reorder", {}))
        else:
            ordered_vps = [clone_vp(p2, v) for v in adapted_vps]
            for k, v in enumerate(ordered_vps):
                v.vp_id = k
        t_reorder = float(time.perf_counter() - t_r0)
        t_c0 = time.perf_counter()
        cov = p2.evaluate_coverage(dv=dv, ordered_vps=ordered_vps, cfg=eval_cfg, seed=seed + 19, return_details=False)
        t_cov = float(time.perf_counter() - t_c0)
        t_p0 = time.perf_counter()
        path_res = p2.build_single_device_path(ordered_vps=ordered_vps, station=station, cfg=eval_cfg)
        t_plan = float(time.perf_counter() - t_p0)
        path_pts = path_res.get("path_points", np.empty((0, 3), dtype=np.float64))
        vio_ratio, vio_len = compute_path_violation(path_pts, station)
        segs = path_res.get("segments", [])
        fallback_cnt = int(sum(1 for s in segs if str(s.get("mode", "")) == "fallback_direct"))
        seg_n = max(1, len(segs))
        n_pairs = max(1, len(ordered_vps) - 1)
        dev_dir = out_dir / name
        dev_dir.mkdir(parents=True, exist_ok=True)
        p2.write_capture_csv(dev_dir / "viewpoints_capture_raw.csv", raw_vps)
        p2.write_capture_csv(dev_dir / "viewpoints_capture_ordered.csv", ordered_vps)
        p2.write_nav_csv(dev_dir / "viewpoints_nav_ordered.csv", ordered_vps)
        p2.write_path_csv(dev_dir / "path_waypoints.csv", path_pts)
        show = p2.simplify_path_for_display(
            path_pts,
            min_step_m=float(phase2_cfg.get("visualization", {}).get("path_display_min_step_m", 0.6)),
            max_points=int(phase2_cfg.get("visualization", {}).get("path_display_max_points", 1800)),
        )
        show = p2.smooth_path_for_display(show, window_size=int(phase2_cfg.get("visualization", {}).get("path_display_smooth_window", 5)))
        p2.write_path_csv(dev_dir / "path_waypoints_display.csv", show)
        t_all = float(time.perf_counter() - t0)
        report = {
            "phase": "phase4_baselines",
            "baseline_method": method_name,
            "scenario": scenario_name,
            "device": name,
            "type": typ,
            "standoff_m": standoff,
            "raw_viewpoints": len(raw_vps),
            "ordered_viewpoints": len(ordered_vps),
            "timing": {
                "viewpoint_generation_time_s": t_gen,
                "viewpoint_reorder_time_s": t_reorder,
                "coverage_eval_time_s": t_cov,
                "path_planning_time_s": t_plan,
                "avg_path_planning_time_per_viewpoint_pair_ms": float((t_plan / n_pairs) * 1000.0),
                "device_total_runtime_s": t_all,
            },
            "coverage": {
                "enabled": bool(cov.get("enabled", False)),
                "sampled_surface_points": int(cov.get("sampled_surface_points", 0)),
                "covered_surface_points": int(cov.get("covered_surface_points", 0)),
                "coverage_ratio": float(cov.get("coverage_ratio", 0.0)),
            },
            "planning": {
                "enabled": bool(eval_cfg["planning"].get("enable_single_device_path_planning", True)),
                "segment_success_ratio": float(path_res.get("segment_success_ratio", 1.0 if len(ordered_vps) <= 1 else 0.0)),
                "total_length_m": float(path_res.get("total_length_m", 0.0)),
                "segments": segs,
            },
            "feasibility": {
                **feas,
                "path_violation_length_ratio": vio_ratio,
                "path_violation_length_m": vio_len,
                "segment_fallback_ratio": float(fallback_cnt / seg_n),
            },
            "outputs": {
                "capture_raw_csv": str(dev_dir / "viewpoints_capture_raw.csv"),
                "capture_ordered_csv": str(dev_dir / "viewpoints_capture_ordered.csv"),
                "nav_ordered_csv": str(dev_dir / "viewpoints_nav_ordered.csv"),
                "path_csv": str(dev_dir / "path_waypoints.csv"),
                "path_display_csv": str(dev_dir / "path_waypoints_display.csv"),
            },
        }
        write_json(dev_dir / "device_report.json", report)
        rows.append(
            {
                "device": name,
                "type": typ,
                "raw_viewpoints": len(raw_vps),
                "ordered_viewpoints": len(ordered_vps),
                "raw_infeasible_ratio": feas["raw_infeasible_ratio"],
                "projected_count": feas["projected_count"],
                "dropped_count": feas["dropped_count"],
                "coverage_ratio": float(cov.get("coverage_ratio", 0.0)),
                "sampled_surface_points": int(cov.get("sampled_surface_points", 0)),
                "covered_surface_points": int(cov.get("covered_surface_points", 0)),
                "path_length_m": float(path_res.get("total_length_m", 0.0)),
                "path_segment_success_ratio": float(path_res.get("segment_success_ratio", 1.0 if len(ordered_vps) <= 1 else 0.0)),
                "path_violation_length_ratio": vio_ratio,
                "viewpoint_generation_time_s": t_gen,
                "viewpoint_reorder_time_s": t_reorder,
                "coverage_eval_time_s": t_cov,
                "path_planning_time_s": t_plan,
                "device_total_runtime_s": t_all,
                "report_json": str(dev_dir / "device_report.json"),
            }
        )
        tot_cov_n += int(cov.get("sampled_surface_points", 0))
        tot_cov_ok += int(cov.get("covered_surface_points", 0))
        tot_gen += t_gen
        tot_plan += t_plan
        tot_vp += len(ordered_vps)
        tot_pairs += max(0, len(ordered_vps) - 1)
        log(f"[{method_name}/{scenario_name}] {name}: vp={len(ordered_vps)}, cov={float(cov.get('coverage_ratio', 0.0)):.3f}, path={float(path_res.get('total_length_m', 0.0)):.2f}m")
    summary_csv = out_dir / "phase2_single_device_summary.csv"
    write_summary_csv(summary_csv, rows)
    summary = {
        "phase": "phase4_baselines",
        "baseline_method": method_name,
        "scenario": scenario_name,
        "device_count": len(rows),
        "global_coverage_ratio_weighted": float(tot_cov_ok / max(1, tot_cov_n)),
        "sampled_surface_points_total": int(tot_cov_n),
        "covered_surface_points_total": int(tot_cov_ok),
        "viewpoint_generation_time_total_s": float(tot_gen),
        "viewpoint_generation_time_avg_per_viewpoint_ms": float((tot_gen / max(1, tot_vp)) * 1000.0),
        "path_planning_time_total_s": float(tot_plan),
        "path_planning_time_avg_per_pair_ms": float((tot_plan / max(1, tot_pairs)) * 1000.0),
        "summary_csv": str(summary_csv),
    }
    write_json(out_dir / "phase2_single_device_summary.json", summary)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Phase4 baseline builder.")
    parser.add_argument("--config", type=Path, default=Path("phases/phase4_baselines/configs/config.phase4.baselines.json"))
    args = parser.parse_args()
    cfg = load_json(args.config)
    root = Path(cfg["paths"]["project_root"]).resolve()
    p2 = load_phase2_module(root)
    phase2_cfg = load_json((root / cfg["paths"]["phase2_reference_config"]).resolve())
    eval_cfg = {"coverage": phase2_cfg["coverage"], "camera": phase2_cfg["camera"], "planning": phase2_cfg["planning"]}
    station_npz = (root / cfg["paths"]["station_grid_npz"]).resolve()
    phase1_report = (root / cfg["paths"]["phase1_report_json"]).resolve()
    station = p2.StationGrid(station_npz)
    angle = p2.load_phase1_rotation_angle(phase1_report)
    deff = p2.load_phase1_deff_map(phase1_report)
    out_root = (root / cfg["paths"]["output_dir"]).resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    devices = select_devices(root, p2, phase2_cfg)
    include_names = {str(x).lower() for x in cfg.get("device_name_whitelist", [])}
    if include_names:
        devices = [(p, t) for (p, t) in devices if p.stem.lower() in include_names]
    max_devices = int(cfg.get("max_devices", 0))
    if max_devices > 0:
        devices = devices[:max_devices]
    log(f"Phase4 output root: {out_root}")
    log(f"Rotation from phase1 report: {math.degrees(angle):.3f} deg")
    log(f"Selected devices: {len(devices)}")
    all_runs: List[dict] = []
    seed = int(cfg.get("random_seed", 42))
    for m_name, m_cfg in cfg.get("methods", {}).items():
        if not bool(m_cfg.get("enable", True)):
            continue
        for s_name, s_cfg in cfg.get("scenarios", {}).items():
            if not bool(s_cfg.get("enable", True)):
                continue
            run_out = out_root / m_name / s_name / "phase2_single_device"
            all_runs.append(
                run_case(
                    p2=p2,
                    root=root,
                    station=station,
                    phase2_cfg=phase2_cfg,
                    eval_cfg=eval_cfg,
                    angle_rad=angle,
                    deff_map=deff,
                    devices=devices,
                    method_name=m_name,
                    method_cfg=m_cfg,
                    scenario_name=s_name,
                    scenario_cfg=s_cfg,
                    out_dir=run_out,
                    base_seed=seed,
                )
            )
    final = {"phase": "phase4_baselines", "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"), "runs": all_runs}
    write_json(out_root / "phase4_baselines_run_summary.json", final)
    print(json.dumps(final, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
