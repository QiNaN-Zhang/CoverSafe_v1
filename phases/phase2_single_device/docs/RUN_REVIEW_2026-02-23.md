# Phase 2 Run Review (2026-02-23, refreshed)

Reviewed artifacts:
- `outputs/phase2_single_device/phase2_single_device_summary.csv`
- `outputs/phase2_single_device/phase2_single_device_summary.json`
- `outputs/phase2_single_device/*/device_report.json`

## Sanity check

- Device count: `8` (buildings/wires excluded) -> expected.
- New timing fields exist in device/global summary.
- New visualization outputs exist:
  - `device_plan_rotate.gif`
  - `coverage_map_rotate.gif`
  - `reorder_baseline_compare.json/png` (main_transformer)

## Current key metrics

- Global weighted coverage: `0.6293`
- Generation time total: `2.139s`
- Path planning total: `58.155s`
- Path planning average per VP pair: `84.16ms`

## Device-level observations

1. Shelf viewpoint counts are controlled
- `shelf1`: `220` viewpoints, coverage `0.950`
- `shelf2`: `220` viewpoints, coverage `0.977`
- `shelf3`: `76` viewpoints, coverage `0.965`（启用底部离群过滤）
- `shelf4`: `36` viewpoints, coverage `0.828`（加强地面切除）

2. Thin-ground removal is active for shelf/tube
- `shelf1/2/3/4`: removed layers > 0
- `tube1/2`: removed layers > 0
- Ground-removal statistics are recorded in each `device_report.json -> preprocess`.
- Additional visual trimming is now applied by device name for `shelf2~4`, `tube1~2` to suppress residual near-ground artifacts in coverage plots.
- Early-stage process trimming (`process_trim_raise_m_*`) is now active, so viewpoint generation and coverage evaluation also use de-grounded device voxels.
- `shelf3` uses an extra bottom-band outlier filter (`bottom_outlier_filter_by_name`) to remove sparse near-ground residuals.

3. Tube coverage remains relatively low
- `tube1`: `0.335`
- `tube2`: `0.297`
- Still consistent with occlusion/accessibility-limited interpretation.

4. Reorder baseline comparison is available on main_transformer
- Proposed layered reorder and global TSP baseline are exported for direct comparison.
- `reorder_baseline_compare.png` is now a side-by-side 3D plan-style figure (same visual language as `device_plan.png`).
- `reorder_baseline_compare.json` now includes reorder time, path length, avg path length per viewpoint pair, and path-planning timing for both methods.

## Conclusion

Phase2 is now in a stable, reproducible state for handoff to Phase3:
- viewpoints quality + order prior are ready,
- single-device path remains optional/diagnostic,
- documentation and outputs are aligned for paper writing.
