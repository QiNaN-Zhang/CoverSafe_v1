#!/usr/bin/env python3
"""Compare ordering connection-cost modes for phase3.

This is a lightweight reproducibility tool:
- Keeps the same phase3 scenario (default: balanced)
- Only changes `ordering.connection_cost.mode`
- Writes JSON/CSV/PNG to outputs/phase3_station_mission/experiments
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import time
from pathlib import Path
import sys
from typing import Dict, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Make `phases.*` importable when running this script directly.
ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import phases.phase3_station_mission.build_station_mission as m


def _pick_scenario_cfg(cfg: dict, scenario_name: str) -> dict:
    patch = {}
    for s in cfg.get("experiments", {}).get("scenarios", []):
        if str(s.get("name", "")) == scenario_name:
            patch = s
            break
    out = m.deep_update(cfg, patch if isinstance(patch, dict) else {})
    out["mission"]["primary_scenario"] = scenario_name
    return out


def _save_rows_csv(path: Path, rows: List[dict]) -> None:
    if not rows:
        return
    keys = [
        "mode",
        "ordering_connection_cost_mode",
        "ordering_s",
        "planning_s",
        "scenario_total_s",
        "scenario_total_s_wall",
        "coverage",
        "trackability_smoothed",
        "path_length_smoothed_m",
        "segment_success_ratio",
        "strict_device_order_match",
        "region_order_match",
        "delta_ordering_s_vs_astar",
        "delta_scenario_total_s_vs_astar",
        "delta_coverage_vs_astar",
        "delta_trackability_vs_astar",
        "delta_path_length_m_vs_astar",
        "delta_segment_success_vs_astar",
    ]
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})


def _save_plot(path: Path, rows: List[dict]) -> None:
    if not rows:
        return
    labels = [str(r["mode"]) for r in rows]
    x = np.arange(len(rows))
    colors = ["#8d99ae", "#2a9d8f", "#457b9d"][: len(rows)]

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    axs = axs.ravel()

    axs[0].bar(x, [float(r["ordering_s"]) for r in rows], color=colors)
    axs[0].set_title("Ordering Time (s)")
    axs[0].set_xticks(x, labels, rotation=15)
    axs[0].grid(True, axis="y", alpha=0.25)

    axs[1].bar(x, [float(r["scenario_total_s"]) for r in rows], color=colors)
    axs[1].set_title("Scenario Total Time (s)")
    axs[1].set_xticks(x, labels, rotation=15)
    axs[1].grid(True, axis="y", alpha=0.25)

    axs[2].bar(x, [float(r["trackability_smoothed"]) for r in rows], color=colors)
    axs[2].set_title("Trackability Smoothed")
    axs[2].set_xticks(x, labels, rotation=15)
    axs[2].grid(True, axis="y", alpha=0.25)

    axs[3].bar(x, [float(r["path_length_smoothed_m"]) for r in rows], color=colors)
    axs[3].set_title("Smoothed Path Length (m)")
    axs[3].set_xticks(x, labels, rotation=15)
    axs[3].grid(True, axis="y", alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare phase3 ordering connection-cost modes.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("phases/phase3_station_mission/configs/config.phase3.station_mission.json"),
    )
    parser.add_argument("--scenario", type=str, default="balanced")
    parser.add_argument(
        "--modes",
        type=str,
        default="astar_hybrid,line_or_euclidean,euclidean",
        help="Comma-separated connection-cost modes.",
    )
    parser.add_argument(
        "--out-prefix",
        type=str,
        default="ordering_connection_mode_compare",
    )
    args = parser.parse_args()

    cfg = m.load_json(args.config)
    root = Path(cfg["paths"]["project_root"]).resolve()
    out_dir = (root / cfg["paths"]["output_dir"]).resolve()
    exp_dir = out_dir / "experiments"
    m.ensure_dir(exp_dir)

    phase2_out = (root / cfg["paths"]["phase2_output_dir"]).resolve()
    phase1_grid = (root / cfg["paths"]["phase1_station_grid_npz"]).resolve()
    phase1_report_path = (root / cfg["paths"]["phase1_report_json"]).resolve()

    phase1_report = m.load_json(phase1_report_path)
    station = m.StationGrid(phase1_grid)
    region_sequence = cfg["mission"]["region_sequence"]
    needed = [str(d) for reg in region_sequence for d in reg.get("devices", [])]
    device_data = m.load_device_data(phase2_out=phase2_out, needed_devices=needed)

    base_cfg = _pick_scenario_cfg(cfg, args.scenario)
    modes = [str(x).strip() for x in str(args.modes).split(",") if str(x).strip()]
    if not modes:
        raise ValueError("No connection-cost mode provided.")

    rows: List[dict] = []
    for mode in modes:
        c = copy.deepcopy(base_cfg)
        c.setdefault("ordering", {}).setdefault("connection_cost", {})["mode"] = mode
        t0 = time.perf_counter()
        res = m.run_single_scenario(
            cfg=c,
            station=station,
            phase1_report=phase1_report,
            device_data=device_data,
            region_sequence=region_sequence,
            out_dir=out_dir,
            scenario_name=f"mode_compare_{args.scenario}_{mode}",
            write_full_outputs=False,
        )
        wall = float(time.perf_counter() - t0)
        timing = res.get("timing", {})
        rows.append(
            {
                "mode": mode,
                "ordering_connection_cost_mode": mode,
                "ordering_s": float(timing.get("ordering_s", 0.0)),
                "planning_s": float(timing.get("planning_s", 0.0)),
                "scenario_total_s": float(timing.get("scenario_total_s", wall)),
                "scenario_total_s_wall": wall,
                "coverage": float(res.get("estimated_global_coverage_ratio", 0.0)),
                "trackability_smoothed": float(res.get("trackability_smoothed", {}).get("trackability_score_0_100", 0.0)),
                "path_length_smoothed_m": float(res.get("path_length_smoothed_m", 0.0)),
                "segment_success_ratio": float(res.get("segment_success_ratio", 0.0)),
                "strict_device_order_match": bool(res.get("order_match", {}).get("strict_device_order_match", False)),
                "region_order_match": bool(res.get("order_match", {}).get("region_order_match", False)),
            }
        )

    base = rows[0]
    for r in rows:
        r["delta_ordering_s_vs_astar"] = float(r["ordering_s"] - base["ordering_s"])
        r["delta_scenario_total_s_vs_astar"] = float(r["scenario_total_s"] - base["scenario_total_s"])
        r["delta_coverage_vs_astar"] = float(r["coverage"] - base["coverage"])
        r["delta_trackability_vs_astar"] = float(r["trackability_smoothed"] - base["trackability_smoothed"])
        r["delta_path_length_m_vs_astar"] = float(r["path_length_smoothed_m"] - base["path_length_smoothed_m"])
        r["delta_segment_success_vs_astar"] = float(r["segment_success_ratio"] - base["segment_success_ratio"])

    out_json = exp_dir / f"{args.out_prefix}.json"
    out_csv = exp_dir / f"{args.out_prefix}.csv"
    out_png = exp_dir / f"{args.out_prefix}.png"
    out_json.write_text(
        json.dumps(
            {
                "scenario": args.scenario,
                "rows": rows,
                "baseline_mode": base["mode"],
                "notes": [
                    "Use this as an efficiency-side comparison.",
                    "Primary output is still recommended with astar_hybrid for robustness.",
                ],
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    _save_rows_csv(out_csv, rows)
    _save_plot(out_png, rows)

    print(f"Saved: {out_json}")
    print(f"Saved: {out_csv}")
    print(f"Saved: {out_png}")
    print(json.dumps({"scenario": args.scenario, "rows": rows}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

