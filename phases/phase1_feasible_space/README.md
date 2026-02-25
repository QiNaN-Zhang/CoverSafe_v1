# Phase 1: 全站级可通行空域

本目录为 Phase 1 的规范化实现与文档入口。

## 入口

推荐从项目根目录执行：

```powershell
python phases/phase1_feasible_space/run_phase1.py --config phases/phase1_feasible_space/configs/config.phase1.feasible_space.json
```

也可直接执行主脚本：

```powershell
python phases/phase1_feasible_space/build_feasible_space.py --config phases/phase1_feasible_space/configs/config.phase1.feasible_space.json
```

## 目录说明

- `build_feasible_space.py`: Phase 1 主脚本。
- `run_phase1.py`: Phase 1 包装入口。
- `configs/config.phase1.feasible_space.json`: 主配置。
- `configs/config.phase1.smoke.json`: 轻量烟测配置。
- `requirements.phase1.txt`: 依赖清单。
- `docs/README_feasible_space.md`: 详细说明。
- `docs/PHASE1_TODO.md`: 遗留问题清单。
- `docs/PHASE1_STATUS.md`: 状态说明。

## 输出

Phase 1 输出目录统一为：
- `outputs/phase1_feasible_space`

关键文件：
- `outputs/phase1_feasible_space/station_grid_map.npz`
- `outputs/phase1_feasible_space/feasible_space_report.json`
- `outputs/phase1_feasible_space/sweep_maps/*`
