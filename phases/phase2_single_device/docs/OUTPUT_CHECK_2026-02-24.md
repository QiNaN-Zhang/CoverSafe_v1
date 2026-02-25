# Phase2 输出体检（2026-02-24）

检查对象：`outputs/phase2_single_device`

## 1. 完整性检查
- 设备数量：`8`（符合排除 buildings/wires 的设定）。
- 每个设备目录核心文件齐全：
  - `viewpoints_capture_raw.csv`
  - `viewpoints_capture_ordered.csv`
  - `viewpoints_nav_ordered.csv`
  - `path_waypoints.csv`
  - `path_waypoints_display.csv`
  - `device_plan.png`
  - `device_plan_rotate.gif`
  - `generation_diagnostics.png`
  - `coverage_map.png`
  - `coverage_map_rotate.gif`
  - `device_report.json`

## 2. 一致性检查
- `summary.csv` 与各 `device_report.json` 的关键字段一致（`ordered_viewpoints`、`coverage_ratio`）。
- `summary.json` 与 `summary.csv` 聚合值一致：
  - `global_coverage_ratio_weighted = 0.6292857142857143`
  - `viewpoint_generation_time_total_s = 2.1392473000159953`
  - `path_planning_time_total_s = 58.15454410001985`

## 3. 现象与判断
- `shelf/tube` 的地面切除已在早期体素阶段生效（非仅可视化隐藏）。
- `shelf3` 已启用底部离群过滤，coverage 图中底部异常绿色点显著减少。
- `shelf4` 已加强切除，当前统计：
  - `ground_removed_layers=5`
  - `coverage_ratio=0.8279`
  - `ordered_viewpoints=36`
- `tube1/tube2` coverage 仍偏低，属于遮挡/可达性限制主导的现象，未发现数据结构错误。

## 4. 当前无结构性异常
- 未发现缺失文件、统计不一致、或格式损坏问题。
- 输出可用于后续 Phase3 输入与论文整理。
