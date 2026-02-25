# Phase4 Baseline 选型与可复现性评估

## 1. 评估范围

评估对象：

1. `FC-Planner`（`baseline_codes/FC-Planner-master`）
2. `PredRecon`（`baseline_codes/PredRecon-master`）
3. `StructuralInspectionPlanner`（`baseline_codes/StructuralInspectionPlanner-master`）

目标约束：

- 运行环境优先 `Windows + Python`。
- 不做 ROS/Ubuntu 全链路部署。
- 复现实验重点是输出 `viewpoints + path`，并接入现有 Phase2/Phase3 评测。

## 2. 快速代码体检结论

### 2.1 FC-Planner

- README 与 CMake 明确依赖 ROS/catkin（Noetic/Melodic）、PCL、OMPL。
- 核心模块位于 C++:
  - `src/viewpoint_manager/*`
  - `src/hierarchical_coverage_planner/*`
  - `src/path_searching/*`
- 代码中可识别的“可抽取核心思想”：
  - skeleton/normal 引导视点生成；
  - 迭代视点更新与裁剪；
  - TSP/LKH 路径组织。

结论：**不适合 Windows 下直接原仓运行；适合做 Python 版核心思路轻量复现（高优先级）。**

### 2.2 PredRecon

- README 与 Planner 子模块明确依赖 ROS + AirSim/Unreal + CUDA + LibTorch。
- `Planner/Code/src/predrecon/*` 主要为 C++ ROS 节点（active_perception、plan_manage、exploration_manager）。
- `SPM` 为深度学习模块，训练/推理链路重，且依赖编译扩展。
- 可抽取核心思想：
  - frontier/coverage 驱动的主动视点选择；
  - 全局与局部层级规划；
  - 可见性驱动的增量覆盖策略。

结论：**完整复现成本高；建议做 PredRecon-style planning 的 Python 轻量实现（保留“active reconstruction”范式）。**

### 2.3 StructuralInspectionPlanner (SIP)

- README 与 CMake 显示依赖 ROS Indigo/catkin，核心为 C++，包含 RRT*/TSP/mesh 流程。
- 输入建模偏 mesh，与当前点云+栅格评测链路存在额外转换成本。

结论：**可做经典对照，但成本最高，建议作为选做 baseline。**

## 3. 最终 baseline 选择建议

主实验建议：

1. `FC-Planner-style (Lite, Python)` - 必做  
2. `PredRecon-style (Lite, Python)` - 必做

备选增强：

3. `SIP-style (Lite, Python)` - 选做（仅在时间允许时加入）

这样可以同时满足：

- 近年代表方法（FC/PredRecon）；
- 审稿人常见关注的 learning-based active reconstruction 方向；
- classic established（SIP）可作为补充论证。

## 4. 复现策略（Phase4 约束下）

统一策略：**不追求原仓完整复刻，追求“核心规划思想 + 同口径评测”的可比实验。**

- 输入：沿用 Phase1/Phase2 现有输入（点云、可行栅格、相机参数）。
- 输出：强制对齐 Phase2 输出字段（`viewpoints_capture_ordered.csv`、`viewpoints_nav_ordered.csv`、`device_report.json`）。
- 评测：全部复用现有 Phase2/Phase3 指标，避免口径不一致。
- 调参原则：仅做基础可运行调参，不做长时间精细调优。

## 5. 风险与应对

主要风险：

1. baseline 原生依赖链过重，无法在本地快速跑通。
2. SIP 的 mesh 流程会拖慢整体节奏。
3. 若 baseline 输出不可行点过多，可能无法直接形成全站任务路径。

应对：

1. 采用 `Strict` 与 `Projected` 两套协议（见执行计划文档）。
2. 优先确保 FC/PredRecon 两个 baseline 完整出结果，SIP 设为可裁剪项。
3. 先保证能生成可评测结果，再考虑少量调参优化。
