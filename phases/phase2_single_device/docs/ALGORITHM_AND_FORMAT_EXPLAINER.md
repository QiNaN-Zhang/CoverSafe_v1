# Phase2 算法与数据格式详解

入口脚本：`phases/phase2_single_device/build_single_device_inspection.py`

本文档用于回答四个问题：
1. 当前 Phase2 到底做了什么。  
2. 每一步输入/输出是什么。  
3. 为什么会出现“非均匀视点”和“局部低 coverage”。  
4. 结果文件如何被 Phase3 直接复用。

## 1. Phase2 目标与边界

### 1.1 目标
- 主目标：生成单设备高质量巡检点（viewpoints）。  
- 次目标：生成可跟踪、可解释的单设备访问顺序。  
- 可选目标：输出单设备局部路径用于可执行性诊断与展示。

### 1.2 非目标
- Phase2 不追求全站全局最短路径。  
- 设备间拼接与全局最优由 Phase3 负责。

## 2. 完整流程（按执行顺序）

### 2.1 数据读取与设备筛选
- 自动发现 `pointclouds/*.las`。  
- 排除 `baseline/origin`。  
- 默认排除 `building` 与 `wires`（Phase2 只处理 8 个待巡检设备）。

### 2.2 体素化与早期地面切除（关键）
- 所有设备先做体素化（与 Phase1 旋转角对齐）。  
- 在体素阶段执行地面切除：
  - 底层剥离：`force_remove_bottom_layers_*`  
  - 底部抬升裁剪：`process_trim_raise_m_*`  
  - 底部离群过滤：`bottom_outlier_filter_by_name`（当前用于 `shelf3`）
- 这一步完成后，后续视点生成/coverage/重排/路径规划都基于“去地面”模型。

### 2.3 视点生成
- 按设备高度分层，提取每层占据边界。  
- 对边界做弧长均匀采样。  
- 按外法向生成候选视点。  
- 使用 Phase1 可通行栅格做可行性过滤：
  - 若候选点不可行，沿法向内外搜索最近可行点。  
- 去重与密度正则（避免过密/过稀）。  
- 设备级点数上限约束（如 shelf1/shelf2）。

### 2.4 视点重排（单设备顺序）
- 主方法：高度带分组 + 带内 NN + 有限 2-opt。  
- 输出“低到高、局部连续”的顺序先验。  
- 可选 baseline：全局 `TSP-NN + 2-opt`（用于对照）。

### 2.5 coverage 评测
- 从表面体素采样点。  
- 相机 frustum 判定（HFOV/VFOV）。  
- 体素射线遮挡检查（DDA）。  
- 统计：
  - `sampled_surface_points`
  - `covered_surface_points`
  - `coverage_ratio`

### 2.6 单设备路径规划（可选）
- 相邻视点先尝试直连。  
- 直连失败再 A*（Phase1 栅格中规划）。  
- 仍失败则 fallback_direct（保底记录失败段）。  
- 输出路径长度、成功率与耗时指标。

### 2.7 可视化
- `device_plan.png/gif`：点云 + 视点 + 路径。  
- `coverage_map.png/gif`：黑色基底 + 绿色 covered 点。  
- `generation_diagnostics.png`：生成与排序诊断图。  
- `reorder_baseline_compare.png`：主方法 vs baseline 左右 3D 对比图（main_transformer）。

## 3. 为什么视点密度不均匀

视点非均匀是“受控结果”，不是随机故障：
- 不同高度层几何复杂度不同（周长不同）。  
- non-convex 与多连通分量导致局部增密。  
- 全站可行性约束会剔除部分候选点。  
- 去重和密度正则会抑制局部过密并有限补洞。

对变电站场景，这种非均匀通常是合理的：复杂结构应分配更多观测资源。

## 4. main_transformer baseline 小实验解读

当前对比通常会出现 trade-off：
- 主方法：coverage 与重排可解释性更好。  
- baseline：局部路径更短、局部规划耗时更低。

建议论文处理方式：
- 如实保留该实验。  
- 明确它反映的是“局部最短路径”与“结构化顺序先验”的目标差异。  
- 最终优劣应以 Phase3 全局任务指标定论。

## 5. 输出文件格式（字段级）

每设备目录：`outputs/phase2_single_device/<device>/`

### 5.1 视点文件
- `viewpoints_capture_raw.csv`  
- `viewpoints_capture_ordered.csv`  
核心字段：  
`x,y,z,yaw_deg,pitch_deg,roll_deg,target_x,target_y,target_z,layer_k,component_id`

- `viewpoints_nav_ordered.csv`  
核心字段：  
`x,y,z,source_viewpoint_id`

### 5.2 路径文件
- `path_waypoints.csv`（原始规划路径）  
- `path_waypoints_display.csv`（展示抽稀/平滑路径）

### 5.3 报告文件
- `device_report.json`  
关键字段：
- `preprocess.ground_removed_layers/voxels`
- `ordered_viewpoints`
- `coverage.coverage_ratio`
- `planning.total_length_m`
- `timing.viewpoint_generation_time_s`
- `timing.viewpoint_reorder_time_s`
- `timing.path_planning_time_s`
- `timing.avg_path_planning_time_per_viewpoint_pair_ms`

### 5.4 对比文件（可选）
- `reorder_baseline_compare.json`
- `reorder_baseline_compare.png`

## 6. 全局汇总文件
- `outputs/phase2_single_device/phase2_single_device_summary.csv`
- `outputs/phase2_single_device/phase2_single_device_summary.json`

其中 `summary.json` 关键聚合指标：
- `global_coverage_ratio_weighted`
- `viewpoint_generation_time_total_s`
- `path_planning_time_total_s`
- 各平均耗时指标

## 7. 复杂度（单设备，近似）

记：
- 原始点云点数 `N`
- 体素数 `V`
- 视点数 `M`
- coverage 采样点 `S`

主要复杂度：
- 体素化：`O(N)`  
- 分层边界处理：`O(V)` 到 `O(V log V)`  
- 视点生成与可行搜索：`O(M * R)`（`R` 为法向搜索步数，常数级）  
- 重排：`O(M log M)` + 局部 2-opt  
- coverage：`O(M * S)` + 射线步进  
- 局部路径：与 A* 展开节点规模相关

## 8. 复现与核查

### 8.1 运行命令
```powershell
pip install -r phases/phase2_single_device/requirements.phase2.txt
python -m py_compile phases/phase2_single_device/build_single_device_inspection.py
python phases/phase2_single_device/build_single_device_inspection.py --config phases/phase2_single_device/configs/config.phase2.single_device.json
```

### 8.2 快速核查
```powershell
Get-Content outputs/phase2_single_device/phase2_single_device_summary.json
Get-Content phases/phase2_single_device/docs/OUTPUT_CHECK_2026-02-24.md
```
