# CoverSafe：Safety-Aware Coverage Planning with Efficient Viewpoint Generation for UAV Inspection of Substations



## Project Overview

This repository contains a multi-phase planning pipeline for UAV-based substation inspection:

- Phase 1: Station-level feasible-airspace generation.
- Phase 2: Single-device viewpoint generation, ordering, coverage evaluation, and local path planning.
- Phase 3: Station-level mission assembly and global path generation from Phase 2 outputs.
- Phase 4: Sim2Real integration (`Coming Soon`).

## Quick Start

### 1) Create a Python environment and install dependencies

```powershell
python -m venv .venv
.venv\Scripts\activate
python scripts/install_requirements.py --upgrade-pip
```

Optional phase-specific installation:

```powershell
python -m pip install -r phases/phase1_feasible_space/requirements.phase1.txt
python -m pip install -r phases/phase2_single_device/requirements.phase2.txt
python -m pip install -r phases/phase3_station_mission/requirements.phase3.txt
```

### 2) Download point clouds

Point cloud data is provided via Google Drive:

`https://drive.google.com/drive/folders/1R0N_E9OJJkdzURY1gaMfgWTxnTRZ_vMq?usp=drive_link`

Steps:

1. Open the link and download all `.las` files.
2. Ensure local folder `pointclouds/` exists (create it if needed).
3. Place all downloaded `.las` files into `pointclouds/`.

Required LAS files:

`building1.las`, `building2.las`, `building3.las`, `capacitor.las`, `main_transformer.las`, `shelf1.las`, `shelf2.las`, `shelf3.las`, `shelf4.las`, `substation_baseline.las`, `tube1.las`, `tube2.las`, `wires1.las`, `wires2.las`.

If any required LAS file is missing, Phase 1/2/3 will print:
`Missing required point cloud models. Please check.`
and stop.

We will provide additional data access options (for example, Hugging Face) in the future.

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

Phase 4 (Sim2Real) is currently a placeholder and marked as `Coming Soon`.

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
|-- requirements.txt
|-- scripts/
|   `-- install_requirements.py
|-- pointclouds/
|   |-- .gitkeep
|   `-- <14 required .las files downloaded from Google Drive>
|-- phases/
|   |-- phase1_feasible_space/
|   |-- phase2_single_device/
|   |-- phase3_station_mission/
|   `-- phase4_sim2real/
|-- media/
|   |-- figures/
|   |   |-- Benchmark Comparisons/
|   |   |-- Phase 1/
|   |   |-- Phase 2/
|   |   `-- Phase 3/
|   `-- videos/
|       |-- Full Version.mp4
|       `-- Quick View.mp4
`-- outputs/  (generated locally, not tracked)
```
