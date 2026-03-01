# UAV Substation Inspection Planning (Phases 1-3)

## Project Overview
This repository contains a multi-phase planning pipeline for UAV-based substation inspection:

- Phase 1: Station-level feasible-space generation.
- Phase 2: Single-device viewpoint generation, ordering, coverage evaluation, and local path planning.
- Phase 3: Station-level mission assembly and global path generation from Phase 2 outputs.
- Phase 5: Sim2Real integration placeholder (`Coming Soon`).

`Phase 4` is intentionally not open-sourced in this repository.

## Quick Start

### 1) Create a Python environment and install dependencies
```powershell
python -m venv .venv
.venv\Scripts\activate
python -m pip install -r phases/phase1_feasible_space/requirements.phase1.txt
python -m pip install -r phases/phase2_single_device/requirements.phase2.txt
```

### 2) Prepare point clouds
This project requires 14 LAS files in `pointclouds/`:

`building1.las`, `building2.las`, `building3.las`, `capacitor.las`, `main_transformer.las`, `shelf1.las`, `shelf2.las`, `shelf3.las`, `shelf4.las`, `substation_baseline.las`, `tube1.las`, `tube2.las`, `wires1.las`, `wires2.las`.

Recommended hosting platform: **Hugging Face Datasets** (free, public, script-friendly).

Automatic download:
```powershell
python scripts/download_pointclouds.py --source hf --repo-id <your-org-or-user>/<your-dataset-repo>
```

Manual download is also supported: place all LAS files directly into `pointclouds/`.

### 3) Run the full pipeline
From repository root:
```powershell
python phases/phase1_feasible_space/run_phase1.py
python phases/phase2_single_device/run_phase2.py
python phases/phase3_station_mission/run_phase3.py
```

Default outputs:
- Phase 1: `outputs/phase1_feasible_space`
- Phase 2: `outputs/phase2_single_device`
- Phase 3: `outputs/phase3_station_mission`

## Run Phases Independently

### Phase 1 only
```powershell
python phases/phase1_feasible_space/run_phase1.py --config phases/phase1_feasible_space/configs/config.phase1.feasible_space.json
```

### Phase 2 only
```powershell
python phases/phase2_single_device/run_phase2.py --config phases/phase2_single_device/configs/config.phase2.single_device.json
```

### Phase 3 only
```powershell
python phases/phase3_station_mission/run_phase3.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json
```

## Repository Structure
```text
.
|-- README.md
|-- scripts/
|   `-- download_pointclouds.py
|-- pointclouds/
|-- phases/
|   |-- phase1_feasible_space/
|   |-- phase2_single_device/
|   |-- phase3_station_mission/
|   `-- phase5_sim2real/
`-- outputs/  (generated locally, not tracked)
```
