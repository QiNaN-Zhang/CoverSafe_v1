diff --git a/f:\FINAL\phases/phase2_single_device/docs/ALGORITHM_AND_FORMAT_EXPLAINER.md b/f:\FINAL\phases/phase2_single_device/docs/ALGORITHM_AND_FORMAT_EXPLAINER.md
deleted file mode 100644
--- a/f:\FINAL\phases/phase2_single_device/docs/ALGORITHM_AND_FORMAT_EXPLAINER.md
+++ /dev/null
@@ -1,123 +0,0 @@
-﻿# Phase 2 单设备算法与输出格式说明
-
-## 1. 当前算法总览
-
-Phase 2（当前实现）对每个设备执行：
-1. 点云体素化（与 Phase 1 相同旋转坐标系）。
-2. 分层截面建模（z 方向）。
-3. 边界弧长均匀采样，按外法向在安全边界外生成候选视点。
-4. 用全站可行栅格过滤/外推视点，尽量贴近目标安全边界。
-5. 密度正则与设备级预算控制（可对 `shelf1/shelf2` 单独限额）。
-6. 生成“层间螺旋上升倾向”的遍历顺序。
-7. Coverage 评测（锥体判定 + 遮挡）。
-8. 可选单设备路径规划（诊断/展示用途）。
-
-入口：`phases/phase2_single_device/build_single_device_inspection.py`
-
-## 2. 视点密度为什么会不均匀
-
-结论：这是“受控非均匀”，不是纯漏洞。
-
-来源包括：
-- 几何因素：不同高度层截面面积/周长不同。
-- 拓扑因素：non-convex + 多连通分量会局部增密。
-- 约束因素：受 Phase 1 可行域约束，部分位置会被剔除或外推。
-- 正则因素：会压制过密点并有限补大间隙，不追求绝对均匀。
-
-在变电站任务中，这种非均匀是合理的：复杂/风险高区域应分配更多观测资源。
-
-## 3. 单设备巡检点生成算法（当前）
-
-### 3.1 分层与边界采样
-
-- 层间距由 `vfov` 与 standoff 决定，并受上下限约束。
-- 每层提取边界后，用弧长均匀采样，避免旧方案的索引抽样聚集。
-
-### 3.2 贴近安全边界
-
-- 默认目标位置在 `d_eff + margin` 附近。
-- 若不可行，则沿外法向做“内外双向搜索”，在可行区内选与目标 standoff 最接近的点。
-
-### 3.3 密度正则 + 设备级控制
-
-- 组内（layer/component）先做最小间距抑制。
-- 对大间隙只做有限补点（`max_add_ratio` 控制）。
-- 可按设备名设置上限（如 `shelf1/shelf2`）以抑制点数爆炸。
-
-## 4. 巡检点重排序算法（当前）
-
-目标：输出“更可跟踪”的参考顺序（Phase3前置先验），而非全局最优。
-
-核心策略：
-- 先按高度层排序（低到高），形成“螺旋上升”总体趋势。
-- 层内按全局极角环绕排序，并在相邻层交替方向（减少回折）。
-- 层与层连接时优先选择离前一层末点最近的起点。
-- 对中等规模层内序列可做一次 2-opt（TSP风格局部改良）。
-
-## 5. Coverage 评测算法
-
-- 表面采样点：从设备表面体素采样。
-- 可见性判定：相机锥体半空间测试。
-- 遮挡判定：体素 DDA 射线检查。
-- 输出：`covered/sampled` 比例。
-
-该定义清晰、可复现，且效率优于体积和法。
-
-## 6. 计算复杂度（按单设备）
-
-设：
-- 点云原始点数 `N`
-- 设备体素数 `V`
-- 生成视点数 `M`
-- coverage采样点数 `S`
-
-主要复杂度：
-1. 体素化：`O(N)`（哈希去重近似线性）。
-2. 分层边界采样：`O(V)` 到 `O(V log V)`（含排序/均匀采样）。
-3. 视点生成与可行搜索：`O(M * R)`，`R` 为法向搜索步数（常数级）。
-4. 重排序：
-- 层内极角排序 `O(M log M)`
-- 局部 2-opt（限定一次）最坏 `O(M_l^2)`，总和约 `O(sum M_l^2)`
-5. Coverage：`O(M * S)`（锥体筛选）+ 遮挡射线步进（步数常数近似）
-6. 单设备路径规划（可选）：与 A* 展开数相关，最坏 `O(E log E)`（E为展开节点）。
-
-## 7. generation_diagnostics.png 解读
-
-四个子图：
-1. XY视点散点（按层着色）：看空间分布。
-2. 每层占据体素数 vs 每层视点数：看密度分配是否合理。
-3. standoff分布直方图：看“贴边程度”和外推频次。
-4. 重排后XY轨迹预览：看顺序是否清晰、是否过度折返。
-
-## 8. Phase2 输出格式
-
-每设备目录：`outputs/phase2_single_device/<device>/`
-
-- `viewpoints_capture_ordered.csv`（拍摄控制）

- - `x,y,z,yaw_deg,pitch_deg,roll_deg,target_x,target_y,target_z,...`
    -- `viewpoints_nav_ordered.csv`（导航）

- - `x,y,z,source_viewpoint_id`
    -- `path_waypoints.csv`（原始局部路径）
    -- `path_waypoints_display.csv`（展示路径）
    -- `device_plan.png`
    -- `generation_diagnostics.png`
    -- `coverage_map.png`
    -- `coverage_map_rotate.gif`
    -- `device_report.json`（含耗时统计）

- 

- 

- 

- 

- 

- 

- 

- 

- 

- 

- 

- 

- 

- 

- 

- 

- 
