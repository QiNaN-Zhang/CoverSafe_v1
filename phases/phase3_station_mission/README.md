# Phase 3: 全站级巡检任务生成

主脚本：`phases/phase3_station_mission/build_station_mission.py`  
包装入口：`phases/phase3_station_mission/run_phase3.py`

## 1. 目标

基于 Phase 2 的单设备巡检结果，在 Phase 1 可飞行栅格约束下，生成一条全站级安全巡检路径，并输出：

- 设备区域访问顺序与是否匹配目标顺序。
- 全局路径（原始 + 平滑）。
- 可跟踪性指标（转角、曲率 proxy、jerk proxy、拐点密度、净空相关指标）。
- 小规模参数实验与统计图。
- 复杂度与耗时统计、全站RGB点云可视化、任务仿真动画。

## 2. 默认巡检区域顺序

配置文件内默认区域顺序为：

1. `shelf2/tube2/shelf3`
2. `capacitor`
3. `main_transformer`
4. `shelf1/tube1/shelf4`

注意：
- 区域顺序固定。
- 设备顺序支持 `ordering.strategy`：
  - `region_constrained`：按区域块约束求最优顺序。
  - `greedy_biased`：基于距离 + 偏置先验的贪心算法（软约束，不是硬强制）。
  - `priority_regularized`：优先级正则化贪心。通过“违约惩罚 + 距离代价”调参，可稳定收敛到指定顺序。

## 3. 运行

```powershell
python phases/phase3_station_mission/build_station_mission.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json
```

或：

```powershell
python phases/phase3_station_mission/run_phase3.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json
```

## 4. 输入

- Phase 1 输出：`outputs/phase1_feasible_space/station_grid_map.npz`
- Phase 1 报告：`outputs/phase1_feasible_space/feasible_space_report.json`
- Phase 2 输出：`outputs/phase2_single_device/<device>/...`

## 5. 输出

目录：`outputs/phase3_station_mission`

- `mission_waypoints_capture.csv`
- `mission_waypoints_nav.csv`
- `path_waypoints.csv`（全局原始路径）
- `path_waypoints_shortcut.csv`（线视通捷径化后路径）
- `path_waypoints_display.csv`（平滑后路径）
- `path_segments.csv`
- `phase3_station_mission_summary.csv`（逐设备统计）
- `phase3_station_mission_summary.json`（全局总结）
- `phase3_complexity_and_timing.json`（复杂度变量与耗时）
- `phase3_coverage_summary.json`（总coverage估计摘要）
- `station_mission_plan.png`
- `station_mission_rotate.gif`
- `station_mission_rgb_path.png`
- `station_mission_rgb_path_rotate.gif`
- `station_mission_local_refine_details.png`
  - 同区域下 `raw/shortcut/smoothed` 三条路径对比
- `station_mission_local_refine_details_smoothing_focus.png`
  - 额外选取更强调 `shortcut` 与 `smoothed` 差异的局部区域
- `station_mission_simulation.gif`
- `station_mission_simulation_fpv.gif`
  - 固定第三人称局部点云仿真，慢速播放并在稀疏巡检点显示浅绿色视场锥
- `experiments/phase3_experiment_summary.csv`
- `experiments/phase3_experiment_summary.json`
- `experiments/phase3_experiment_compare.png`
  - 仅对比 `conservative / balanced / aggressive`
- `experiments/station_mission_rgb_path_compare_phase2_style.png`
  - 左：主方案全站路径；右：`phase2_style_local` 全站路径
- `experiments/station_mission_rgb_path_compare_phase2_style_rotate.gif`
- `experiments/priority_regularized_order_lightweight_test.csv`
- `experiments/priority_regularized_order_lightweight_test.json`
- `experiments/phase1_safety_distance_effect_summary.csv`
- `experiments/phase1_safety_distance_effect_summary.json`
- `experiments/phase1_safety_distance_effect_compare.png`
- `experiments/ordering_connection_mode_compare.csv`
- `experiments/ordering_connection_mode_compare.json`
- `experiments/ordering_connection_mode_compare.png`

## 6. 轻量顺序稳定性测试（论文口径）

为避免“纯人工硬编码顺序”争议，Phase3 提供独立轻量测试脚本验证：  
在 `priority_regularized` 策略下，通过可解释参数（尤其 `violation_weight`）可稳定得到与人工要求一致的设备顺序。

运行：

```powershell
python phases/phase3_station_mission/tools/lightweight_order_stability_test.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json
```

当前站点测试结论（见 `outputs/phase3_station_mission/experiments/priority_regularized_order_lightweight_test.json`）：
- 总计 180 组参数。
- `violation_weight=64` 时，严格顺序匹配率为 `20/20 = 100%`（在该子网格内稳定）。

## 7. 核心方法概述

1. 读取 Phase2 每设备 `viewpoints_capture_ordered.csv` / `viewpoints_nav_ordered.csv`。  
2. 对设备内航点进行覆盖保真压缩（近点合并 + 远离/低贡献点裁剪）。  
3. 在固定区域顺序下，自动搜索区域内最优设备顺序，并通过 DP 选择每设备正向/反向遍历。  
4. 对完整任务航点序列执行全局路径规划（直连优先，失败走 A*，再 fallback）。  
5. 做安全约束下的平滑，并输出可跟踪性指标。  
6. 对不同压缩/平滑参数做小实验，输出对比统计。

## 8. 结果解释说明（论文写作备用，2026-02-24）

1. `phase2_style_local` 的定位与取舍：
   - 该方法作为对照实验保留，但不建议作为主方案重点讨论。
   - 当前结果中，相比 `balanced`，`phase2_style_local` 在全局效率上显著变差：
     - 平滑路径长度：`2414.13m -> 3773.99m`（约 +56%）
     - 段成功率：`0.9018 -> 0.8795`
     - 顺序代价：`443.79 -> 515.70`
   - 覆盖率/可跟踪性仅有小幅变化：
     - coverage：`0.4984 -> 0.5069`
     - trackability(smoothed)：`15.366 -> 15.382`
   - 结论：收益不足以抵消全局代价，建议主文弱化，放附录作为负结果/消融。

2. 全局路径视觉“杂乱”的解释口径：
   - Phase3 的一级目标是“可执行 + 安全 + 覆盖”，不是视觉最短/最直。
   - 站内障碍密集、设备跨区跨层，且存在大量避障连接段，导致路径自然包含绕行与高度切换。
   - 当前主场景虽视觉复杂，但已实现：
     - 点数压缩：`raw 11830 -> smoothed 287`
     - 严格顺序与区域顺序匹配：均为 `True`
   - 因此应将“形态复杂”解释为约束驱动结果，而非算法失效。

3. Local refinement 两张局部图的解释口径：
   - `station_mission_local_refine_details.png`：展示“总体差异较明显”区域；
   - `station_mission_local_refine_details_smoothing_focus.png`：刻意强调 `shortcut->smoothed` 差异。
   - 若两图视觉差异仍有限，属于预期现象：local refinement 仅做局部、受约束微调，不改变全局拓扑。
   - 该模块的主要价值是局部可跟踪性与安全裕度的稳健微改善，论文中应以指标表为主、图示为辅。

## 9. 补充实验说明（phase1 动/静态安全距离）

1. `conservative / balanced / aggressive` 不是 phase1 安全距离扫参：
   - 它们主要是 phase3 内部的“压缩/捷径化/平滑”参数强度对比，不改变 phase1 栅格来源。

2. phase1 动/静态安全距离对 phase3 的影响实验：
   - 入口配置：`phase1_safety_sweep_experiment`
   - 数据来源：`outputs/phase1_feasible_space/sweep_maps/sweep_summary.csv` 及对应 `station_grid_map_*.npz`
   - 目的：在相同 phase3 场景（默认 `balanced`）下，仅替换 phase1 栅格，观察 coverage / 可跟踪性 / 路径长度等指标变化。
   - 输出：
     - `experiments/phase1_safety_distance_effect_summary.csv`
     - `experiments/phase1_safety_distance_effect_summary.json`
     - `experiments/phase1_safety_distance_effect_compare.png`

3. 与主实验关系：
    - 该实验是“附加小实验”，不替代主实验，不覆盖主输出。
4. 当前数据结论（`phase1_safety_distance_effect_summary.json`）：
   - 全样本均值（dynamic - static）：
     - coverage：`+0.0083`
     - trackability(smoothed)：`+1.209`
     - path_length(smoothed)：`+699.57m`
     - low_clearance_ratio：`-0.0228`（更低更好）
   - 说明：dynamic 在覆盖率/可跟踪性上整体更好，但路径长度更长，且不同安全距离下存在明显 trade-off。
   - 建议：论文里将其定义为“敏感性补充实验”，重点报告趋势与权衡，不与主实验直接混排排名。

## 10. 效率优化开关（不改变默认行为）

- `ordering.connection_cost.mode`（默认 `astar_hybrid`）：
  - `astar_hybrid`：原有行为（线连通优先，不通则 A*，再 fallback）
  - `line_or_euclidean`：快速近似（可用于顺序搜索加速测试）
  - `euclidean`：纯欧氏近似（仅建议用于快速验证）
- 同时增加了连接代价缓存的双向复用（A->B 与 B->A 共享缓存），用于减少重复计算开销。
- 当前快测结果（`experiments/ordering_connection_mode_compare.json`，balanced）：
  - `astar_hybrid`：`ordering_s=44.57s`，`scenario_total_s=95.29s`
  - `line_or_euclidean`：`ordering_s=0.0039s`，`scenario_total_s=53.82s`
  - `euclidean`：`ordering_s=0.00014s`，`scenario_total_s=53.64s`
  - 覆盖率一致；可跟踪性略升（`+0.183`）；路径长度小幅增加（`+5.81m`）；段成功率微降（`-0.0018`）
- 稳妥策略建议：
  - 主实验/主结论保持 `astar_hybrid`（鲁棒、可解释）。
  - 将 `line_or_euclidean` 作为“效率侧对比”或“快速迭代模式”。
  - 如果后续追求兼顾，可采用两阶段：先 `line_or_euclidean` 快速筛选，再用 `astar_hybrid` 复核候选顺序。

## 11. 复现建议（最终）

1. 主流程复现（含主输出）：
```powershell
python phases/phase3_station_mission/build_station_mission.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json
```

2. 顺序稳定性轻量复现（不重跑全流程）：
```powershell
python phases/phase3_station_mission/tools/lightweight_order_stability_test.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json
```

3. 连接代价模式效率对比复现：
```powershell
python phases/phase3_station_mission/tools/compare_ordering_connection_modes.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json --scenario balanced
```

4. phase1 动/静态安全距离补充实验复现：
  - 打开配置 `phase1_safety_sweep_experiment.enable=true`
  - 运行主脚本（第 1 条命令）
  - 可用 `max_cases_per_mode` 做快速子集复现，再跑全量
