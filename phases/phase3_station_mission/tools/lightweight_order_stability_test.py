#!/usr/bin/env python3
"""Lightweight test for priority-regularized greedy ordering stability.

This script does NOT change the phase3 pipeline. It only runs a fast sweep
using Euclidean connection proxy and reports whether the learned order can
stably match the operator-specified strict order.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Dict, List, Sequence, Tuple

import numpy as np

# Make `phases.*` importable when running this script directly.
ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import phases.phase3_station_mission.build_station_mission as m


def flatten_region_devices(region_sequence: Sequence[dict]) -> List[str]:
    out: List[str] = []
    for reg in region_sequence:
        for d in reg.get("devices", []):
            out.append(str(d))
    return out


def route_endpoint(route: m.CompressedDeviceRoute, reverse: bool, which: str) -> np.ndarray:
    if which == "entry":
        return route.nav_points[-1] if reverse else route.nav_points[0]
    return route.nav_points[0] if reverse else route.nav_points[-1]


def select_priority_regularized_order(
    routes: Dict[str, m.CompressedDeviceRoute],
    start_pt: np.ndarray,
    preference_order: Sequence[str],
    distance_weight: float,
    rank_weight: float,
    violation_weight: float,
    region_switch_penalty: float,
) -> Tuple[List[str], List[bool], float]:
    rank = {str(d): i for i, d in enumerate(preference_order)}
    n = max(1, len(routes))
    unvisited = set(routes.keys())
    cur = start_pt.copy()
    prev_region = None
    order: List[str] = []
    rev_flags: List[bool] = []
    total = 0.0

    while unvisited:
        best = None
        best_score = float("inf")
        rem_sorted = sorted(unvisited)
        for d in rem_sorted:
            rnk = int(rank.get(d, n))
            rank_norm = float(rnk / max(1, n - 1))
            violation = sum(1 for u in unvisited if int(rank.get(u, n)) < rnk)
            base_pen = rank_weight * rank_norm + violation_weight * float(violation)
            region_pen = region_switch_penalty if (prev_region is not None and routes[d].region != prev_region) else 0.0
            for rev in (False, True):
                ent = route_endpoint(routes[d], reverse=rev, which="entry")
                ex = route_endpoint(routes[d], reverse=rev, which="exit")
                travel = float(np.linalg.norm(ent - cur))
                score = distance_weight * travel + base_pen + region_pen
                if score < best_score:
                    best_score = score
                    best = (d, rev, travel, ex)
        if best is None:
            break
        d_pick, rev_pick, travel_pick, ex_pick = best
        order.append(str(d_pick))
        rev_flags.append(bool(rev_pick))
        total += float(travel_pick)
        prev_region = routes[d_pick].region
        cur = ex_pick.copy()
        unvisited.remove(d_pick)
    return order, rev_flags, total


def main() -> None:
    parser = argparse.ArgumentParser(description="Lightweight ordering stability sweep.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("phases/phase3_station_mission/configs/config.phase3.station_mission.json"),
    )
    parser.add_argument("--top-k", type=int, default=12)
    args = parser.parse_args()

    cfg = m.load_json(args.config)
    root = Path(cfg["paths"]["project_root"]).resolve()
    phase2_out = (root / cfg["paths"]["phase2_output_dir"]).resolve()
    phase1_grid = (root / cfg["paths"]["phase1_station_grid_npz"]).resolve()
    phase1_report_path = (root / cfg["paths"]["phase1_report_json"]).resolve()
    out_dir = (root / cfg["paths"]["output_dir"] / "experiments").resolve()
    m.ensure_dir(out_dir)

    phase1_report = m.load_json(phase1_report_path)
    station = m.StationGrid(phase1_grid)

    region_sequence = cfg["mission"]["region_sequence"]
    needed = flatten_region_devices(region_sequence)
    device_to_region, _, expected_by_region = m.build_region_maps(region_sequence)
    strict_target = [str(x) for x in cfg["mission"].get("strict_device_order_target", expected_by_region)]

    data = m.load_device_data(phase2_out=phase2_out, needed_devices=needed)
    routes: Dict[str, m.CompressedDeviceRoute] = {}
    for d, dd in data.items():
        routes[d] = m.compress_device_route(dd, region=device_to_region.get(d, "unknown"), cfg=cfg)
    routes = m.apply_phase2_style_local_order(routes=routes, cfg=cfg)
    routes = m.apply_local_refinement(routes=routes, station=station, cfg=cfg)

    start_xyz = np.array(cfg["mission"]["start_home_xyz"], dtype=np.float64)
    if str(cfg["mission"].get("start_home_frame", "raw")).lower() == "raw":
        start_xyz = m.rotate_xy(start_xyz[None, :], float(phase1_report.get("rotation", {}).get("angle_rad", 0.0)))[0]
    start_xyz = m.nearest_free_world(station, start_xyz, max_k=int(cfg["mission"].get("home_snap_max_search_vox", 12)))

    sweep_violation = [0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0]
    sweep_rank = [0.0, 2.0, 5.0, 10.0, 20.0]
    sweep_region = [0.0, 2.0, 4.0, 8.0]

    rows: List[dict] = []
    for wv in sweep_violation:
        for wr in sweep_rank:
            for wg in sweep_region:
                order, rev, est = select_priority_regularized_order(
                    routes=routes,
                    start_pt=start_xyz,
                    preference_order=strict_target,
                    distance_weight=1.0,
                    rank_weight=wr,
                    violation_weight=wv,
                    region_switch_penalty=wg,
                )
                rows.append(
                    {
                        "distance_weight": 1.0,
                        "rank_weight": float(wr),
                        "violation_weight": float(wv),
                        "region_switch_penalty_m": float(wg),
                        "strict_match": bool(order == strict_target),
                        "order": order,
                        "reverse_flags": rev,
                        "estimated_proxy_cost_m": float(est),
                    }
                )

    by_wv = {}
    for wv in sweep_violation:
        sub = [r for r in rows if float(r["violation_weight"]) == float(wv)]
        ok = sum(1 for r in sub if bool(r["strict_match"]))
        by_wv[f"{wv:g}"] = {"matches": int(ok), "total": int(len(sub)), "ratio": float(ok / max(1, len(sub)))}

    strict_rows = [r for r in rows if bool(r["strict_match"])]
    strict_rows_sorted = sorted(strict_rows, key=lambda x: float(x["estimated_proxy_cost_m"]))
    top_k = int(max(1, args.top_k))
    top_show = strict_rows_sorted[:top_k]

    csv_path = out_dir / "priority_regularized_order_lightweight_test.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "distance_weight",
                "rank_weight",
                "violation_weight",
                "region_switch_penalty_m",
                "strict_match",
                "estimated_proxy_cost_m",
                "order",
            ]
        )
        for r in rows:
            writer.writerow(
                [
                    f"{float(r['distance_weight']):.3f}",
                    f"{float(r['rank_weight']):.3f}",
                    f"{float(r['violation_weight']):.3f}",
                    f"{float(r['region_switch_penalty_m']):.3f}",
                    str(bool(r["strict_match"])),
                    f"{float(r['estimated_proxy_cost_m']):.6f}",
                    "->".join([str(x) for x in r["order"]]),
                ]
            )

    out_json = {
        "strict_target": strict_target,
        "total_cases": int(len(rows)),
        "strict_match_cases": int(len(strict_rows)),
        "strict_match_ratio": float(len(strict_rows) / max(1, len(rows))),
        "match_ratio_by_violation_weight": by_wv,
        "top_strict_match_cases": top_show,
    }
    json_path = out_dir / "priority_regularized_order_lightweight_test.json"
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(out_json, f, ensure_ascii=False, indent=2)

    print(f"Saved: {csv_path}")
    print(f"Saved: {json_path}")
    print(
        json.dumps(
            {
                "strict_target": strict_target,
                "total_cases": len(rows),
                "strict_match_cases": len(strict_rows),
                "strict_match_ratio": out_json["strict_match_ratio"],
                "best_match_case": top_show[0] if top_show else None,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
