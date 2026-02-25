# Phase3 算法与输出说明

入口脚本：`phases/phase3_station_mission/build_station_mission.py`

## 1. 路径生成流程

1. 读取 Phase2 的单设备有序巡检点与报告。  
2. 执行设备内压缩：
- 合并过近点（减少局部抖动）。
- 在保底覆盖约束下，优先删除“远离设备中心且近共线”的点。  
3. 执行全站顺序规划：
- 固定区域序列。
- 区域内部枚举设备顺序。
- 对每种设备顺序，使用 DP 自动选择每设备的正向或反向访问。  
可选：使用 `greedy_biased` 策略，以“距离 + 软偏置先验”做设备顺序判定（非硬强制）。
可选：使用 `priority_regularized` 策略，以“距离 + 优先级违约惩罚”做贪心判定。
4. 在 Phase1 栅格上规划全局路径：
- 直线连通优先。
- 不可直连时用 A*。
- A*失败时记录 fallback 直连段。  
5. 对原始路径先做 line-of-sight 捷径化，再做安全平滑，并计算可跟踪性指标。  
6. 汇总统计并可视化。
7. 输出复杂度变量与耗时分解。
8. 可选附加实验：phase1 动/静态安全距离扫参对 phase3 指标影响。

## 2. 可跟踪性指标

输出包含以下指标（原始路径与平滑路径都统计）：

- 最大/平均/P95 转角（deg）
- 曲率 proxy 与最小转弯半径 proxy
- jerk proxy（RMS 与 energy）
- 拐点数量与每 100m 拐点密度
- 最小/平均净空、低净空段比例
- 综合可跟踪性分数（0~100）

## 3. 顺序一致性分析

脚本会同时输出：

- `region_order_match`：是否符合目标区域顺序。
- `strict_device_order_match`：是否完全符合严格设备顺序。

其中“区域顺序”与“严格设备顺序”均可在配置里调整。

## 4. 输出文件用途

- `mission_waypoints_capture.csv`：全任务巡检点（含姿态，含 home 起终点）。
- `mission_waypoints_nav.csv`：导航点（仅坐标）。
- `path_waypoints.csv`：全局原始可执行路径。
- `path_waypoints_shortcut.csv`：捷径化后的路径。
- `path_waypoints_display.csv`：平滑后路径（用于展示与跟踪分析）。
- `path_segments.csv`：逐段规划方式与长度。
- `phase3_station_mission_summary.csv`：逐设备统计。
- `phase3_station_mission_summary.json`：全局总结与实验结果。
- `phase3_complexity_and_timing.json`：复杂度统计与阶段耗时。
- `phase3_coverage_summary.json`：总coverage估计摘要。
- `station_mission_plan.png/gif`：全站路径可视化。
- `station_mission_rgb_path.png/gif`：RGB全站点云+总路径可视化。
- `station_mission_local_refine_details.png`：局部区域路径细节图。
  - 自动选择差异最明显的小区域，包含 raw / shortcut / smoothed 三条路径对比。
- `station_mission_local_refine_details_smoothing_focus.png`：局部区域路径细节图（平滑优势强调版）。
  - 自动偏向选择 `shortcut` 与 `smoothed` 差异更明显的小区域。
- `station_mission_simulation.gif`：全局任务仿真动画。
- `station_mission_simulation_fpv.gif`：固定第三人称局部点云仿真动画（稀疏巡检点显示浅绿色视场锥）。
- `experiments/*`：小实验对比结果。
  - `phase3_experiment_compare.png` 仅展示 conservative / balanced / aggressive。
  - `station_mission_rgb_path_compare_phase2_style.png/.gif` 展示主方案与 phase2_style_local 全站路径对比。
  - `priority_regularized_order_lightweight_test.csv/.json` 用于验证算法调参是否可稳定匹配目标顺序。
  - `phase1_safety_distance_effect_summary.csv/.json/.png` 用于验证 phase1 动/静态安全距离对 phase3 指标的影响。
  - `ordering_connection_mode_compare.csv/.json/.png` 用于验证 `connection_cost.mode` 的效率与指标变化。

## 5. 论文写作口径建议（顺序约束）

- 实际任务层面：区域访问顺序可来自运维规程/风险分区/作业窗口，是任务先验约束。  
- 算法层面：Phase3 同时提供 `priority_regularized` 可选策略，将“目标顺序”转化为可解释惩罚项而非人工 if/else。  
- 实证层面：本项目轻量测试显示，在当前站点配置下，当 `violation_weight=64` 时可稳定匹配目标顺序（参数子网格匹配率 100%）。  
- 表达方式：建议论文中写成“规则先验 + 可调优化器”的两层框架，避免表述为纯手工强制。

## 6. 论文写作口径建议（实验解释，2026-02-24）

1. 关于 `phase2_style_local`：
- 建议作为对照/消融保留，不作为主方法主叙事。
- 当前结果（`outputs/phase3_station_mission/experiments/phase3_experiment_summary.csv`）显示：
  - 与 `balanced` 比较，`phase2_style_local` 平滑路径明显更长（`2414.13m -> 3773.99m`）、段成功率更低（`0.9018 -> 0.8795`）、顺序代价更高（`443.79 -> 515.70`）。
  - 覆盖率和可跟踪性仅有小幅变化（coverage `0.4984 -> 0.5069`，trackability(smoothed) `15.366 -> 15.382`）。
- 结论：该方法在本场景的全局代价偏高，主文建议弱化，附录给出完整对照。

2. 关于全局路径“看起来杂乱”：
- 这是站级约束优化的正常现象：障碍密度高、设备分布跨层跨区、避障连接段占比高。
- Phase3 优先目标为“可执行性/安全性/覆盖”，视觉简洁性是次级目标。
- 在该目标下，主场景已实现显著点数压缩（`raw 11830 -> smoothed 287`）并保持顺序约束完全匹配。

3. 关于 Local refinement 局部图“差异不够夸张”：
- Local refinement 的设计本身是受约束的局部微调，不改变全局路径拓扑。
- 因此 `shortcut` 与 `smoothed` 常见为“可控小改动”，不应期待所有区域都出现大形态差异。
- 已同时输出两类图：
  - `station_mission_local_refine_details.png`（总体差异区域）
  - `station_mission_local_refine_details_smoothing_focus.png`（平滑优势强调区域）
- 论文中建议“图示辅助 + 指标为主”的证据组织方式。

## 7. 论文写作口径建议（phase1 安全距离补充实验）

- 先澄清：`conservative/balanced/aggressive` 并非 phase1 安全距离扫参，而是 phase3 内部压缩与平滑强度对比。  
- 若论文需要回答“phase1 动/静态安全距离是否影响 phase3”，可直接引用 `phase1_safety_distance_effect_*` 结果。  
- 建议表述为：  
  - “在保持 phase3 算法与参数一致的前提下，仅替换 phase1 可行栅格，对任务级指标进行敏感性评估。”  
  - “该实验属于补充分析，不影响主实验定义与主结论。”  
- 结果解读建议：  
  - 如果 dynamic 在 coverage/trackability 上更优且长度不劣，可作为正向证据。  
  - 若存在 trade-off（某些指标升、某些降），应如实报告并强调工程可调性与任务优先级选择。

## 8. phase1 动/静态安全距离实验当前结论（2026-02-25）

- 结果来源：`outputs/phase3_station_mission/experiments/phase1_safety_distance_effect_summary.json`
- 全样本均值（dynamic - static）：
  - coverage：`+0.0083`
  - trackability(smoothed)：`+1.209`
  - path_length(smoothed)：`+699.57m`
  - low_clearance_ratio：`-0.0228`（更低更好）
- 解释：
  - dynamic 方案在覆盖率与可跟踪性上有正向收益；
  - 但路径长度有明显上升，且不同安全距离参数下段成功率波动较大；
  - 结论应写成“敏感性分析与工程权衡”，而不是“单向绝对更优”。

## 9. connection_cost.mode 效率对比与稳妥策略

- 结果来源：`outputs/phase3_station_mission/experiments/ordering_connection_mode_compare.json`（balanced）
- 实测：
  - `astar_hybrid`：`ordering_s=44.57s`，`scenario_total_s=95.29s`
  - `line_or_euclidean`：`ordering_s=0.0039s`，`scenario_total_s=53.82s`
  - `euclidean`：`ordering_s=0.00014s`，`scenario_total_s=53.64s`
  - 覆盖率一致；可跟踪性略升；路径长度小幅上升；段成功率微降
- 建议口径：
  - 主实验默认保留 `astar_hybrid`（更稳妥、对复杂障碍更鲁棒）。
  - `line_or_euclidean` 作为“效率对比”和“调参加速模式”。
  - 最稳妥流程：先 `line_or_euclidean` 快速筛选，再 `astar_hybrid` 复核最终候选。

## 10. 最小复现命令

```powershell
# 主流程
python phases/phase3_station_mission/build_station_mission.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json

# 顺序稳定性轻量测试
python phases/phase3_station_mission/tools/lightweight_order_stability_test.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json

# 连接代价模式对比
python phases/phase3_station_mission/tools/compare_ordering_connection_modes.py --config phases/phase3_station_mission/configs/config.phase3.station_mission.json --scenario balanced
```
