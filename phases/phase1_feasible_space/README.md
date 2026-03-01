# Phase 1: Feasible Space Generation

## Purpose
Phase 1 builds a station-level 3D voxel map with blocked/free space under safety constraints.

## Dependencies
```powershell
python -m pip install -r phases/phase1_feasible_space/requirements.phase1.txt
```

## Input
- LAS files in `pointclouds/` (segmented device clouds).
- Optional baseline/origin LAS names from config.

If any required LAS file is missing, the program exits immediately.

## Run
```powershell
python phases/phase1_feasible_space/run_phase1.py
```

Use a custom config:
```powershell
python phases/phase1_feasible_space/run_phase1.py --config phases/phase1_feasible_space/configs/config.phase1.feasible_space.json
```

## Output
Default output directory: `outputs/phase1_feasible_space`

Key artifacts:
- `station_grid_map.npz`
- `feasible_space_report.json`
