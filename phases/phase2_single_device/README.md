# Phase 2: 单设备巡检点生成与评测

主脚本：`phases/phase2_single_device/build_single_device_inspection.py`

本阶段目标：在 Phase 1 全站可通行空域约束下，为单设备生成可执行巡检点（viewpoints）、给出较好遍历顺序，并计算覆盖率与可选的单设备局部路径。

## 1. 当前默认对象范围

- 自动发现 `pointclouds/*.las`。
- 默认排除 `building` 和 `wires`，即只对 8 个待巡检设备执行 Phase 2。
- `wires` 在项目中主要用于 Phase 1 全站空域建模，不进入 Phase 2 巡检点生成。

对应配置：`phases/phase2_single_device/configs/config.phase2.single_device.json`

## 2. 运行

先安装依赖（可选二选一）：

```powershell
pip install -r phases/phase2_single_device/requirements.phase2.txt
```

或安装项目统一依赖（包含 Phase 1/2）：

```powershell
pip install -r requirements.txt
```

然后运行：

```powershell
python phases/phase2_single_device/build_single_device_inspection.py --config phases/phase2_single_device/configs/config.phase2.single_device.json
```

## 3. 输入

- 设备点云：`pointclouds/*.las`
- 全站栅格：`outputs/phase1_feasible_space/station_grid_map.npz`（Phase 1 输出）
- 可选参数报告：`outputs/phase1_feasible_space/feasible_space_report.json`（读取旋转角、设备 `d_eff`）

## 4. 输出

目录：`outputs/phase2_single_device`

每设备子目录包含：
- `viewpoints_capture_raw.csv`：原始视点（含姿态）
- `viewpoints_capture_ordered.csv`：重排后视点（含姿态）
- `viewpoints_nav_ordered.csv`：仅导航坐标
- `path_waypoints.csv`：原始局部路径（直线 + A*）
- `path_waypoints_display.csv`：仅用于展示的抽稀/平滑路径
- `device_plan.png`：RGB 点云基底 + 视点 + 展示路径
- `device_plan_rotate.gif`：`device_plan` 的旋转动态图
- `generation_diagnostics.png`：视点生成诊断图
- `coverage_map.png`：单设备覆盖区域图（黑色基底 + 绿色 covered 点）
- `coverage_map_rotate.gif`：单设备覆盖区域旋转图
- `device_report.json`：设备统计
- `reorder_baseline_compare.json`：重排基线对比统计（可选）
- `reorder_baseline_compare.png`：重排基线对比图（可选）

全局汇总：
- `phase2_single_device_summary.csv`
- `phase2_single_device_summary.json`

## 5. 两类数据格式约定

1. 定点拍摄格式（capture）
- 核心字段：`x,y,z,yaw_deg,pitch_deg,roll_deg,target_x,target_y,target_z`
- 用途：用于相机姿态控制与拍摄指向。

2. 导航格式（nav）
- 核心字段：`x,y,z,source_viewpoint_id`
- 用途：用于后续路径规划，不耦合相机姿态。

巡检点重排顺序查看位置：
- `viewpoints_capture_ordered.csv`（`viewpoint_id`即访问顺序）
- `viewpoints_nav_ordered.csv`（`waypoint_id`即访问顺序）
- `device_plan.png`（蓝线按重排顺序连接）

上述拆分是合理的：拍摄与机动控制在工程上应解耦，便于后续 Phase 3 做跨设备全局优化。

## 6. 方法简述（对应代码）

1. 形状自适应分层  
按 `vfov` 与 standoff 计算层间距，在 z 方向做自适应采样，避免固定层厚引起的漏检/冗余。

2. 非凸设备处理  
每层做 2D 占据投影，连通域分解后沿边界点外法向生成候选视点，适应 z 轴 non-convex 结构。

3. 密度自适应  
单层视点数由边界周长和 `hfov` 覆盖宽度估计，叠加重叠率和去重半径控制密度。

4. 全站可行性约束  
视点生成阶段即用 Phase 1 栅格过滤不可行点，并沿外法向做小范围搜索，避免“单设备可行、全站不可行”。

5. 重排与可跟踪性  
先按高度带（`band_height_m`）分组；层内做近邻顺序并可选 2-opt；层间从上一层末点接入下一层最近点，得到“低到高、邻近优先”的参考遍历顺序。

6. 覆盖率评测  
先用相机锥体半空间判定，再用体素 DDA 射线检查遮挡，替代低效的四棱锥体积和判定。

7. 时间统计  
输出每设备巡检点生成总时间、每点平均生成时间、路径规划总耗时、每对相邻点平均路径规划耗时（见 `device_report.json` 和全局 `summary.csv/json`）。

8. 薄层地面去除（shelf/tube）  
对 `shelf/tube` 设备先做底层剥离（可按类型/按设备名配置），并在 **Phase2 早期体素构建阶段** 支持世界坐标下的底部抬升裁剪（`process_trim_raise_m_*`）。  
这意味着后续的视点生成、coverage 评测、重排、路径规划都基于去地面后的设备体素。  
可视化仍保留 `visual_trim_*` 作为展示层面的辅助，但核心地面切除不再依赖可视化阶段。
对少量设备还可启用 `bottom_outlier_filter_by_name`，在底部高度带内剔除低邻域离群体素（当前用于 `shelf3`）。

## 7. 为什么视点看起来不均匀

不均匀是算法设计结果，不是随机错误，主要来自：
- 设备截面非均匀：不同层周长不同，导致每层目标点数不同。
- 非凸/多连通结构：每个连通分量独立采样，局部会更密。
- 全站可行性过滤：位于禁飞栅格内的候选点会被剔除或外推，导致局部稀疏。
- 去重与层窗机制：会合并过近点、并在层间做窗口投影，进一步改变局部分布。

`generation_diagnostics.png` 用于展示上述机制（层占据 vs 视点数、standoff 分布、重排轨迹预览）。

## 8. 当前实现优势（可用于论文写作）

- 贴近变电站约束：视点生成直接受 Phase1 全站可行空域约束。
- 形状自适应：分层 + 连通域 + 边界法向，能处理 non-convex 与多分量结构。
- 工程可控：密度正则、设备级上限、按类型参数可快速调试。
- 输出完整：生成、重排、覆盖、局部路径、耗时统计全部可追踪。
- 对比友好：内置 `main_transformer` 的重排基线对比，便于论文做消融/对照。

## 9. 可选基线对比（单设备）

默认配置会在 `main_transformer` 上输出一个轻量对比：
- 基线顺序：全局 `TSP-NN + 2-opt`
- 路径算法：与主方法相同（直连 + A* fallback）

输出文件：
- `reorder_baseline_compare.json`
- `reorder_baseline_compare.png`

其中 `reorder_baseline_compare.png` 为左右 3D 视角对比（风格与 `device_plan.png` 一致）：左图为我们方法，右图为 baseline。  
`reorder_baseline_compare.json` 统计两种方法的：
- `path_length_m`
- `avg_path_length_per_viewpoint_pair_m`
- `avg_pair_path_length_m` / `min_pair_path_length_m` / `max_pair_path_length_m`
- `reorder_time_s`
- `path_planning_time_s`
- `avg_path_planning_time_per_viewpoint_pair_ms`
- `segment_success_ratio`

## 10. 关于低覆盖率的解释口径

对部分设备（尤其细长、遮挡重、邻近高风险区域）低 coverage 可能是物理可达性问题，而非算法故障。  
建议口径：
- 覆盖率不是单纯越高越好，应与安全约束、可达性、任务风险共同评价。
- 对“无人机几何上难覆盖”的区域，应考虑人工补检/地面机器人协同。
- 不建议直接删除难覆盖区域后再报告 coverage，除非论文中明确给出“可达性掩码”定义和规则。

## 11. 单设备路径规划在 Phase 2 的定位

- 建议定位为“展示与局部可执行性验证”，不是全局最优结果。
- 设备间真实最优路径应在 Phase 3 统一拼接与全站规划时求解。
- 因此本阶段保留路径功能，但把重点放在“高质量视点 + 合理遍历顺序”。

## 12. 本地复现（推荐）

在项目根目录 `F:\FINAL` 执行：

```powershell
pip install -r phases/phase2_single_device/requirements.phase2.txt
python phases/phase2_single_device/build_single_device_inspection.py --config phases/phase2_single_device/configs/config.phase2.single_device.json
```

运行后检查：
- `outputs/phase2_single_device/phase2_single_device_summary.csv`
- `outputs/phase2_single_device/<device>/device_report.json`

快速语法检查：

```powershell
python -m py_compile phases/phase2_single_device/build_single_device_inspection.py
```

## 13. 本次运行结果复核

- 复核文档：`phases/phase2_single_device/docs/RUN_REVIEW_2026-02-23.md`
- 输出体检：`phases/phase2_single_device/docs/OUTPUT_CHECK_2026-02-24.md`
- 遗留清单：`phases/phase2_single_device/PHASE2_TODO.md`
- 算法与格式详解：`phases/phase2_single_device/docs/ALGORITHM_AND_FORMAT_EXPLAINER.md`
- Phase3衔接与动态安全距离分析：`phases/phase2_single_device/docs/PHASE2_PATH_PHASE3_DYNAMIC_ANALYSIS.md`
