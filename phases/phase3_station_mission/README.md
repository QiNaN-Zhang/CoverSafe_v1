# Phase 3: Station Mission Generation

## Purpose
Phase 3 assembles Phase 2 per-device results into a station-level mission path, including ordering, global path planning, and post-processing.

## Dependencies
```powershell
python -m pip install -r phases/phase3_station_mission/requirements.phase3.txt
```

## Input
- Phase 1:
  - `outputs/phase1_feasible_space/station_grid_map.npz`
  - `outputs/phase1_feasible_space/feasible_space_report.json`
- Phase 2:
  - `outputs/phase2_single_device/<device>/viewpoints_capture_ordered.csv`
  - `outputs/phase2_single_device/<device>/viewpoints_nav_ordered.csv`
  - `outputs/phase2_single_device/<device>/device_report.json`

If any required LAS file is missing, the program exits immediately.

## Run
```powershell
python phases/phase3_station_mission/run_phase3.py
```

Use a custom config:
```powershell
python phases/phase3_station_mission/run_phase3.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json
```

## Output
Default output directory: `outputs/phase3_station_mission`

Key artifacts:
- `mission_waypoints_capture.csv`
- `mission_waypoints_nav.csv`
- `path_waypoints.csv`
- `path_waypoints_display.csv`
- `phase3_station_mission_summary.json`
