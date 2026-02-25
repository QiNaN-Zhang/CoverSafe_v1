# 变电站全站级可通行空域生成（Phase 1）

本模块用于完成项目第一大块内容：从全站点云和设备分割点云出发，生成可用于后续路径规划的体素化可通行空域地图。

核心脚本：`phases/phase1_feasible_space/build_feasible_space.py`  
主配置：`phases/phase1_feasible_space/configs/config.phase1.feasible_space.json`

## 1. 功能概述

脚本主要完成以下流程：

1. 读取 `substation_baseline.las`，估计并执行 XY 旋转对齐。
2. 估计地面高度与飞行体积边界。
3. 体素化障碍与各设备分割点云。
4. 根据设备类型计算动态安全距离：`d_eff = r(type) * d0 + d_unc + d_geom`。
5. 处理设备重叠体素归属（按 `type_priority`，当前 `wires` 最高）。
6. 膨胀得到禁飞体素，导出全站 `npz` 地图。
7. 导出分类可视化、可通行空域可视化、分层扫描可视化和扫参结果。

## 2. 输入数据约定

默认读取目录：`pointclouds/`

- 全站点云：
  - `substation_baseline.las`（实际用于处理）
  - `substation_origin.las`（仅用于排除，不参与处理）
- 设备分割点云：其余 `.las` 文件自动发现并按文件名关键词推断类型。

当前已支持类型：`transformer / shelf / tube / wires / capacitor / building / ...`

## 3. 环境与安装

建议使用 Python 3.10+，并先创建虚拟环境。

```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -r phases/phase1_feasible_space/requirements.phase1.txt
```

## 4. 运行方式

```powershell
python phases/phase1_feasible_space/build_feasible_space.py --config phases/phase1_feasible_space/configs/config.phase1.feasible_space.json
```

## 5. 主要输出（`outputs/phase1_feasible_space/`）

- 地图与报告：
  - `outputs/phase1_feasible_space/station_grid_map.npz`
  - `outputs/phase1_feasible_space/feasible_space_report.json`
  - `outputs/phase1_feasible_space/substation_preprocessed.pcd`
- 禁飞与可通行叠加：
  - `outputs/phase1_feasible_space/overlay_cloud_nofly.png`
  - `outputs/phase1_feasible_space/cloud_nofly_rotate.gif`
  - `outputs/phase1_feasible_space/overlay_compare_nofly_free.png`
  - `outputs/phase1_feasible_space/cloud_nofly_free_compare_rotate.gif`
- 分类结果（按设备类型）：
  - `outputs/phase1_feasible_space/device_types_overview.png`
  - `outputs/phase1_feasible_space/device_types_rotate.gif`
- 可通行空域多视角：
  - `outputs/phase1_feasible_space/overlay_free_bands.png`（近地/中低空/全体积）
  - `outputs/phase1_feasible_space/cloud_free_bands_rotate.gif`
  - `outputs/phase1_feasible_space/free_space_layer_scan.gif`（俯视逐层扫描）
  - `outputs/phase1_feasible_space/free_space_layer_scan_iso.gif`（正等轴测逐层扫描）
- 扫参与敏感性分析：
  - `outputs/phase1_feasible_space/sweep_maps/sweep_summary.json`
  - `outputs/phase1_feasible_space/sweep_maps/sweep_summary.csv`
  - `outputs/phase1_feasible_space/sweep_maps/sweep_free_space_compare.png`
  - `outputs/phase1_feasible_space/sweep_maps/station_grid_map_dynamic_*.npz`
  - `outputs/phase1_feasible_space/sweep_maps/station_grid_map_static_*.npz`

## 6. `npz` 格式说明

`station_grid_map.npz` 包含：

- `origin`: 体素网格原点（世界坐标）
- `voxel_size`: 体素分辨率（m）
- `shape`: 网格尺寸 `(nx, ny, nz)`
- `blocked`: 禁飞体素索引数组 `(N,3)`

后续路径规划通常将 `blocked` 视作硬约束，`~blocked` 为空域可行域。

## 7. 配置要点（`config.phase1.feasible_space.json`）

- `risk_coefficients`: 各类型风险系数 `r(type)`。
- `type_priority`: 重叠归属优先级（当前 `wires` 优先）。
- `inspectable_device_types`: 后续巡检点生成时可直接过滤 `building`。
- `visualization.*`: 各图采样量、GIF 帧率、扫描视角参数。
- `sweep_analysis.*`: 动态/静态对照扫参与 CSV/JSON/NPZ 导出开关。

## 8. 复现实验建议

- 固定 `random_seed` 保证图与统计稳定可复现。
- 论文中建议同时引用：
  - `feasible_space_report.json`（主实验参数与统计）
  - `sweep_summary.csv`（敏感性/消融）
  - 对应 `npz`（后续路径规划输入）
