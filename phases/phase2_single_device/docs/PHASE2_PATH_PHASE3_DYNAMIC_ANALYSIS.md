# Phase 2 分析：单设备路径必要性、Phase3衔接与动态/静态安全距离影响

## A. 单设备路径规划是否必要

结论：保留，但定义为 **optional 诊断模块**。

- Phase 2 主目标：高质量视点 + 可解释遍历顺序。
- 单设备路径在 Phase 2 仅用于：
  - 连通性预检查
  - 演示可执行性
  - 形成 Phase3 的先验
- 最终全站最优路径必须在 Phase 3 统一求解（跨设备拼接后）。

## B. Phase3 如何利用 Phase2 输出（高效做法）

建议输入：
- `viewpoints_capture_ordered.csv`（保留拍摄约束）
- `viewpoints_nav_ordered.csv`（导航节点）
- `coverage`/`quality`标签（可扩展）

建议流程：
1. 设备内保覆盖压缩（减少节点）。
2. 设备间构建桥接图（距离+可达性+风险代价）。
3. 先设备级访问序，再节点级细化（GTSP/TSP混合思路）。
4. 在 Phase 1 栅格上做统一全局规划与平滑。

## C. 动态安全距离影响：应做到哪一阶段

建议：
- Phase 2 做小验证（机制层）。
- 主实验放在 Phase 3（任务层）。

原因：
- Phase 2 指标（coverage、视点数）只反映局部可观测性。
- 论文主结论应由 Phase 3 的任务级指标给出（总路径、总时长、可达率、安全裕量等）。

## D. Main Transformer 小规模实验（已扩展）

实验对象：`main_transformer`

实验变量：
- 动态安全距离（d0 sweep）
- 静态安全距离（3组 static npz）

实验设置：
- 仅选择 `main_transformer`
- 使用不同 Phase1 `station_grid_map*.npz`
- 保留单设备路径规划，以统计 `path_length`

结果目录：
- `outputs/phase2_single_device/experiments/main_transformer_phase1_sweep/`

结果文件：
- `main_transformer_phase1_sweep_summary.csv`
- `main_transformer_phase1_sweep_plot.png`

### D1. 动态 sweep 结果（摘要）

- `dynamic_d0_1_8`: viewpoints `93`, coverage `0.450`, path `912.57m`
- `dynamic_d0_2_0`: viewpoints `75`, coverage `0.403`, path `843.34m`
- `dynamic_d0_2_3`: viewpoints `56`, coverage `0.364`, path `760.87m`
- `dynamic_d0_2_6`: viewpoints `42`, coverage `0.300`, path `531.82m`

趋势：d0 增大 -> 视点数与 coverage 下降，路径长度总体下降。

### D2. 静态 sweep 结果（新增）

- `static_3_30`: viewpoints `130`, coverage `0.475`, path `1020.10m`
- `static_3_80`: viewpoints `100`, coverage `0.435`, path `899.16m`
- `static_4_30`: viewpoints `72`, coverage `0.408`, path `876.72m`

趋势：静态安全距离增大时，视点数与 coverage 同样下降，路径长度总体缩短。

### D3. 结论（小实验层面）

- 动态与静态都体现了“安全距离↑ -> coverage/viewpoints↓”的可达性规律。
- 动态策略在同等安全尺度附近可提供更细粒度调节空间。
- 但最终“哪种更优”仍取决于 Phase 3 全局任务指标，不宜只凭 Phase 2 coverage 判定。

## E. 现阶段建议

1. Phase 2 固定当前参数，作为稳定输入层。
2. Phase 3 将动态/静态作为关键对照实验。
3. 论文结构：
- Phase 2：机制有效性（局部）
- Phase 3：任务收益（全局）

## F. Phase3 建议直接复用的 Phase2 输出

- 每设备顺序点：`viewpoints_nav_ordered.csv`
- 拍摄姿态点：`viewpoints_capture_ordered.csv`
- 局部质量指标：`coverage_ratio`、`path_segment_success_ratio`
- 局部复杂度指标：`ordered_viewpoints`、`path_length_m`

建议在 Phase3 中把上述指标作为设备级先验权重，而不是直接拼接 Phase2 局部路径。

## G. 关于 main_transformer 重排 baseline 对比的论文写法建议

当前结果表明这是一个典型 trade-off：
- 我们方法在 coverage 与重排速度上有优势（覆盖略高，重排耗时更低）。
- baseline（全局 TSP-NN + 2-opt）在单设备局部路径长度与路径规划耗时上更优。

建议论文中“如实保留”该小实验，而不是删除不提，理由：
1. 结果透明，可提高方法可信度。
2. 对比清楚揭示了不同目标函数：
- baseline更偏向单设备局部最短路径；
- 我们方法更偏向分层可解释、可跟踪顺序，并为Phase3跨设备拼接提供结构先验。
3. 与项目整体目标一致：最终性能结论应由 Phase3 全局任务指标给出，Phase2 小实验主要用于机制分析。

推荐呈现方式：
- 主文给出简要结论 + trade-off 解释；
- 详细数值（路径长度、pair长度统计、重排耗时、路径规划耗时）放在附录/补充材料。
