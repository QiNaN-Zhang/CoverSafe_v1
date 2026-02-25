# Phase 2 Run Review (2026-02-22)

Reviewed files:
- `outputs/phase2_single_device/phase2_single_device_summary.csv`
- `outputs/phase2_single_device/*/device_report.json`

## Key observations

1. Current run still contains `wires1/wires2`  
- This is inconsistent with Phase 2 inspection scope.
- Cause: previous config did not exclude `wires`.
- Fix applied in config: `exclude_types = ["building", "wires"]`.

2. Very large single-device path length on shelf devices  
- `shelf1`: 225 viewpoints, `6790.68 m`, success `0.906`.
- `shelf2`: 214 viewpoints, `8751.16 m`, success `0.840`.
- This is mainly a visualization/execution path densification issue (A* node-by-node polyline), not a viewpoint generation failure.
- Fix applied: add `path_waypoints_display.csv` (simplified/smoothed for display) and RGB-based `device_plan.png`.

3. Low coverage on tube devices  
- `tube1`: `0.218`
- `tube2`: `0.159`
- Interpretation: likely combined effect of slim geometry + occlusion + station-level feasibility constraints.
- Recommended to keep as measured result and explain with accessibility constraints instead of force-fitting parameters.

4. Apparent outlier on `wires2` coverage  
- Coverage `0.932` with only 4 viewpoints and 752 sampled surface points.
- This is plausible for a small/simple segmented shape, but not useful for Phase 2 since wires are now excluded.

## Current baseline conclusion

- Weighted global coverage in this run: `0.5201` (10 devices, includes wires).
- For the intended Phase 2 scope (8 inspectable devices), rerun with updated config is needed to obtain final summary.

