# Phase 2: Single-Device Inspection Planning

## Purpose
Phase 2 generates inspection viewpoints for each target device, reorders viewpoints, evaluates coverage, and builds optional local paths.

## Dependencies
```powershell
python -m pip install -r phases/phase2_single_device/requirements.phase2.txt
```

## Input
- Device LAS files: `pointclouds/*.las`
- Phase 1 outputs:
  - `outputs/phase1_feasible_space/station_grid_map.npz`
  - `outputs/phase1_feasible_space/feasible_space_report.json`

If any required LAS file is missing, the program exits immediately.

## Run
```powershell
python phases/phase2_single_device/run_phase2.py
```

Use a custom config:
```powershell
python phases/phase2_single_device/run_phase2.py --config phases/phase2_single_device/configs/config.phase2.single_device.json
```

## Output
Default output directory: `outputs/phase2_single_device`

Per-device outputs include:
- `viewpoints_capture_ordered.csv`
- `viewpoints_nav_ordered.csv`
- `path_waypoints.csv`
- `device_report.json`

Global summary:
- `phase2_single_device_summary.csv`
- `phase2_single_device_summary.json`
