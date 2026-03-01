# Phase 1: Feasible Space Generation

## Purpose
Phase 1 builds a station-level 3D voxel map with blocked/free space under safety constraints.

## Input
- LAS files in `pointclouds/` (segmented device clouds).
- Optional baseline/origin LAS names from config.

If baseline/origin LAS are missing, the script can synthesize a baseline LAS from available segmented LAS files.

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
