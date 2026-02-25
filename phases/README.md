# Project Phases

- `phase1_feasible_space`: 全站可通行空域建模（已完成初版）。
- `phase2_single_device`: 单设备巡检点生成、重排、覆盖率评测、单设备路径规划。
- `phase3_station_mission`: 多设备任务拼接与全站任务级路径规划。
- `phase4_baselines`: 对比算法接入与统一评测。
- `phase5_sim2real`: 仿真到实机一致性验证与参数闭环。

## Recommended Entrypoints

- Phase 1: `python phases/phase1_feasible_space/build_feasible_space.py --config phases/phase1_feasible_space/configs/config.phase1.feasible_space.json`
- Phase 1 wrapper: `python phases/phase1_feasible_space/run_phase1.py --config phases/phase1_feasible_space/configs/config.phase1.feasible_space.json`
- Phase 2: `python phases/phase2_single_device/build_single_device_inspection.py --config phases/phase2_single_device/configs/config.phase2.single_device.json`
