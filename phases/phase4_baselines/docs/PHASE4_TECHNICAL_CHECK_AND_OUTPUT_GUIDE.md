# PHASE4 Technical Check And Output Guide

Updated: 2026-02-25
Scope: Quick code check + output consistency check + documentation consolidation for current Phase4 implementation.

## 1. Quick Check Summary

### 1.1 Code-level checks
- Syntax check passed:
  - `python -m py_compile phases/phase4_baselines/build_phase4_baselines.py`
  - `python -m py_compile phases/phase4_baselines/run_phase4.py`
- No blocking runtime exception path found in current standard configs.

### 1.2 Output-level checks
- Verified key outputs exist and are readable:
  - `outputs/phase4_baselines/phase4_baselines_run_summary.json`
  - `outputs/phase4_baselines/phase4_phase3_comparison_summary.csv`
- `phase4_baselines_run_summary.json` contains 4 runs (2 methods x 2 scenarios), each with required fields:
  - `baseline_method`, `scenario`, `device_count`, `global_coverage_ratio_weighted`, `summary_csv`

Conclusion:
- Current Phase4 pipeline is runnable and output-complete for FC-Planner-Lite + PredRecon-Lite baselines.

## 2. Algorithm And Data Flow (Current Implementation)

### 2.1 Pipeline
1. Load Phase2 module (`build_single_device_inspection.py`) for shared utilities.
2. Discover devices using Phase2 filtering/type inference rules.
3. Generate baseline viewpoints per device:
   - `fc_planner_lite`: layered radial candidate strategy.
   - `predrecon_lite`: greedy gain-based viewpoint selection.
4. Apply scenario constraints:
   - `constrained_projected`: enforce Phase1 free space + nearest-free projection.
   - `unconstrained_native`: no Phase1 constraint enforcement.
5. Reorder viewpoints (`global_tsp` / `phase2_layered` / `none`).
6. Evaluate coverage using Phase2 evaluator.
7. Build per-device path using Phase2 planner.
8. Export Phase2-compatible artifacts and summaries.

### 2.2 Output compatibility
Phase4 output is intentionally Phase2-style, so Phase3 can directly consume:
- `viewpoints_capture_ordered.csv`
- `viewpoints_nav_ordered.csv`
- `path_waypoints_display.csv`
- `device_report.json`
- `phase2_single_device_summary.csv/json`

## 3. Metric Interpretation Notes (Important For Paper)

### 3.1 Coverage vs feasibility
- `coverage_ratio` alone is insufficient.
- Always pair with feasibility signals:
  - `path_segment_success_ratio`
  - `path_violation_length_ratio`

### 3.2 Why unconstrained planning can look "fast"
- In unconstrained runs, planning time may become extremely small.
- This often co-occurs with very low segment success and high violation ratio.
- Interpretation: faster does not imply executable in constrained substation environments.

### 3.3 Viewpoint generation time fairness
- Compare both:
  - total generation time
  - per-viewpoint generation time
- Total time is strongly affected by viewpoint count.

### 3.4 Trackability interpretation
- Higher trackability score does not indicate better mission quality by itself.
- Must be interpreted jointly with coverage and executability.

## 4. Current Result Snapshot (From Existing Outputs)

Phase4 aggregate (`phase4_baselines_run_summary.json`):
- `fc_planner_lite / constrained_projected`: weighted coverage `0.5420`
- `fc_planner_lite / unconstrained_native`: weighted coverage `0.4586`
- `predrecon_lite / constrained_projected`: weighted coverage `0.1466`
- `predrecon_lite / unconstrained_native`: weighted coverage `0.0698`

Phase4->Phase3 comparison (`phase4_phase3_comparison_summary.csv`):
- Unconstrained runs have significantly degraded segment success in station mission.
- Constrained FC baseline is the strongest among current two baselines under this setup.

## 5. Reproducibility Commands

### 5.1 Phase4 full baseline run
```powershell
python phases/phase4_baselines/run_phase4.py --config phases/phase4_baselines/configs/config.phase4.baselines.json
```

### 5.2 Phase4 smoke run
```powershell
python phases/phase4_baselines/run_phase4.py --config phases/phase4_baselines/configs/config.phase4.smoke_all.json
```

### 5.3 Phase3 integration run (example)
- Set `paths.phase2_output_dir` in Phase3 config to one Phase4 output folder:
  - `outputs/phase4_baselines/<method>/<scenario>/phase2_single_device`

## 6. Visualization Deliverables Already Prepared

For paper-friendly path visualization demos:
- Directory:
  - `outputs/phase4_baselines/comparison_with_ours/path_demo`
- Includes:
  - triptych PNGs (station and main_transformer)
  - per-method standalone PNGs
  - per-method rotating GIFs

## 7. Recommended Reporting Template (Paper)

When writing final comparison tables/figures, keep this order:
1. Coverage: `coverage_ratio` / `estimated_global_coverage_ratio`
2. Executability: `segment_success_ratio`, `path_violation_length_ratio`
3. Cost: `path_length_smoothed_m`, runtime
4. Motion quality: trackability metrics
5. Visual evidence: station + single-device path demos with RGB clouds

This avoids over-claiming on single metrics (e.g., short path or high trackability alone).
