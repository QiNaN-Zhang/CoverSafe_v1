下面是gpt基于 **now.zip 的工程实现**（Phase1/2/3 的输入输出与评测口径）+ 提供的 **FC-Planner** 论文（含其常用对比方法谱系）整理出的 **2~3 个最合适 baseline**。优先满足：**(1) 2023~2026 新算法/顶会期刊 + 易复现/开源；(2) 能和你当前评测体系对齐；(3) 在变电站特殊约束下“天然吃亏”的点明确**。

* * *

先对齐：我理解你现在的方法核心（用于选 baseline）
-----------------------------

从 now.zip 的 Phase1/2/3 结构来看，你的方法优势主要集中在：

* **Phase1：全站可飞行空域建模**（栅格/禁飞/安全距离/导线等），后续所有视点与路径都要满足可行性约束（这是“变电站特殊性”的核心落点）。

* **Phase2：单设备视点生成**：形状自适应分层、非凸结构连通域处理、边界外法向生成候选视点、密度与去重可控；并且**生成阶段就做全站可行性过滤/外推修复**；覆盖率评测用视锥 + 体素射线遮挡检查（工程可追踪）。

* **Phase3：多设备任务拼接**：区域顺序约束 + 设备顺序搜索/正反遍历 DP + 全局连通代价（直连优先、A* fallback）+ 平滑与“可跟踪性”指标。

因此，你最需要的 baseline 类型其实是两类：

1. **UAV 3D Coverage Path Planning (CPP)**：输入点云/场景，输出覆盖视点与轨迹（与你最像）。

2. **Learning-based / active reconstruction / NBV**：审稿人点名要补的（它们往往不擅长你的“强约束+细长障碍+禁飞区”场景）。

* * *

我建议你最终选这 3 个 baseline（做 2 个也行）
------------------------------

### Baseline 1（CPP强相关、且近年顶会SOTA）：**FC-Planner (ICRA 2024)** ✅首选

* **是什么**：面向 UAV 的复杂 3D 场景覆盖规划（点云输入），通过骨架引导的空间分解 + 视点生成 + 分层/并行规划来生成覆盖轨迹。它在论文中明确对标“视点生成 + 路径规划”的经典框架范式。([GitHub](https://github.com/HKUST-Aerial-Robotics/FC-Planner?utm_source=chatgpt.com "HKUST-Aerial-Robotics/FC-Planner"))

* **为什么适合你**：
  
  * 和你一样是“点云→视点→覆盖路径”的 pipeline（非常好对齐）。
  
  * 顶会新算法 + 官方代码（复现成本相对可控）。([GitHub](https://github.com/HKUST-Aerial-Robotics/FC-Planner?utm_source=chatgpt.com "HKUST-Aerial-Robotics/FC-Planner"))
  
  * 你甚至已经拿到论文 PDF：

* **为什么它在变电站约束下可能更吃亏（有利于你）**：
  
  * 目标是“覆盖整个复杂场景”，而你是“设备级目标 + 全站禁飞约束优先”，在变电站这种**导线/禁飞区密集**的环境里，它更容易生成大量“理论可覆盖但工程不可执行/被你 效率下滑。
  
  * 其骨架/内部空间判定在“细长导线 + 密集遮挡”的点云上更可能不稳定（这点很常见且很符合变电站特殊性叙事）。

* **你这边怎么接入最省事（建议口径）**：
  
  * **单设备对比**：把每个设备点云（你 Phase2 的输入）作为 FC-Planner 的输入，得到其视点/路径；再用你现有的 coverage evaluator 和 trackability 指标统一评测。
  
  * **全站对比**：把“全站点云（不含 building/wires 或含 wires 两版）”分别跑，做敏感性讨论：含 wires 往往更难（更能突出你的 Phase1/约束建模价值）。

* * *

### Baseline 2（审稿人点名的 learning-based active reconstruction）：**PredRecon (ICRA 2023)** ✅首选

* **是什么**：ICRA 2023 的“Prediction-boosted”自主航拍重建规划框架（属于你审稿人想看的那类：**learning-based / active reconstruction**），并且**官方开源**。([GitHub](https://github.com/HKUST-Aerial-Robotics/PredRecon?utm_source=chatgpt.com "HKUST-Aerial-Robotics/PredRecon"))

* **为什么适合你**：
  
  * 直接回应审稿意见里那句 _“recent learning-based active reconstruction methods”_。
  
  * 同样是 aerial reconstruction / view planning 方向，语义与论文叙事高度一致。([GitHub](https://github.com/HKUST-Aerial-Robotics/PredRecon?utm_source=chatgpt.com "HKUST-Aerial-Robotics/PredRecon"))

* **为什么它在变电站约束下可能更吃亏（有利于你）**：
  
  * 这类方法通常更关注“信息增益/重建质量/探索效率”，但对你这种“强禁飞约束 + 可跟踪性指标 + 设备优先级顺序”的任务目标不匹配：
    
    * 一旦你用 Phase1 空域把它的候选视点过滤/投影，常见结果是**覆盖不全或路径显著变长**（这正好凸显你的优势：约束内生、可执行优先）。
  
  * 变电站设备存在大量**重复结构 + 细长遮挡**，学习/预测模块更容易出现泛化偏差（你可以把它写成“domain shift + safety constraints”）。

* **接入建议（尽量让对比“公平但不替它擦屁股”）**：
  
  * 你可以设两种接入方式（论文里写清楚即可）：
    
    1. **原生模式**：按其仓库默认流程跑；
    
    2. **约束投影模式**：把其输出视点落到你的 feasible grid 上（最近可行点/沿法向搜索），再评测。
  
  * 这样你既展示它“在一般场景很强”，也展示“在变电站约束下为何不如你的任务定制方法”。

* * *

### Baseline 3（“Established NBV/CPP”用来回应审稿人、且很可能在你场景下表现差）：**SIP / Structural Inspection Planner (ICRA 2015)** ✅建议作为“经典对照”

> 如果你只想做 2 个 baseline：就做 **FC-Planner + PredRecon**；  
> 如果你愿意做第 3 个来增强说服力：我建议加 SIP（它非常“好解释”，而且在你场景下确实容易吃亏）。

* **是什么**：经典结构巡检路径规划（迭代视点重采样 + 连接代价优化），**ETH 的开源实现**。([GitHub](https://github.com/ethz-asl/StructuralInspectionPlanner?utm_source=chatgpt.com "ethz-asl/StructuralInspectionPlanner"))

* **为什么仍值得加**：审稿人明确说希望看到“NBV/CPP established algorithms”，SIP 在 aerial inspection 领域就是最典型的“established baseline”之一。([GitHub](https://github.com/ethz-asl/StructuralInspectionPlanner?utm_source=chatgpt.com "ethz-asl/StructuralInspectionPlanner"))

* **为什么它很可能在你的变电站特殊约束下不佳（有利于你）**：
  
  * 它强依赖结构表示（常见是 mesh/几何假设），而你这边是点云+体素+禁飞栅格；把变电站点云转 mesh 的误差/孔洞/导线细结构会显著拖累它。
  
  * 它并非为“密集禁飞区 + 强可跟踪性”设计：你一旦用 Phase1 约束去筛它的视点/路径，通常会出现**大量视点失效→覆盖率下降**或**大幅绕行→路径变长**。

* **工程成本提醒**：它会比前两个更“折腾”（因为输入格式/表示不同），但它作为“经典算法在变电站环境下的失败案例”，往往能写出非常有说服力的分析段落。

* * *

你最终怎么选（我给你一个明确结论）
-----------------

* **最推荐的 2 个（满足你“近3年+顶会+开源+强相关”）**：
  
  1. **FC-Planner (ICRA 2024)** ([GitHub](https://github.com/HKUST-Aerial-Robotics/FC-Planner?utm_source=chatgpt.com "HKUST-Aerial-Robotics/FC-Planner")) +
  
  2. **PredRecon (ICRA 2023)** ([GitHub](https://github.com/HKUST-Aerial-Robotics/PredRecon?utm_source=chatgpt.com "HKUST-Aerial-Robotics/PredRecon"))

* **如果你要做满 3 个，为了“审稿人一句话就闭嘴”+“更容易跑出你想要的结论”**：  
  
  3) **SIP / StructuralInspectionPlanner (ICRA 2015, classic)** ([GitHub](https://github.com/ethz-asl/StructuralInspectionPlanner?utm_source=chatgpt.com "ethz-asl/StructuralInspectionPlanner"))

* * *

如果你愿意，我下一步可以直接按你仓库的 **Phase4_baselines** 预期接口，帮你把这 2~3 个 baseline 的“接入形态”定义成统一的输入输出（viewpoints/path）规范，以及你应该报告哪些指标（coverage / path length / segment success / trackability / runtime）才能让对比最有说服力。











下面是gpt的进一步分析和建议：

下面我把“**Phase4_baselines** 怎么落地”一次性补全：让你复现 2~3 个 baseline 后，**不改你现有 Phase2/Phase3 的评测器**，只做“适配/转换”，就能直接跑出**单设备 + 全站任务**两套对比结果（coverage / 可执行性 / 可跟踪性 / 安全净空 / 耗时）。

* * *

0. 目标：Baseline 也输出成“你的 Phase2 格式”，Phase3 直接复用

---------------------------------------------

你的 Phase2/Phase3 已经把**评测体系**做得很完善了（coverage + 轨迹可执行性 + trackability 等）。最稳妥、最省事、也最难被审稿人挑刺的对比方式是：

> **Baseline 只负责生成 viewpoints/trajectory；评测一律用你现在的 evaluator。**  
> 这样你对比的是“算法输出质量”，不是“评测口径差异”。

Phase3 配置里已经支持替换 `phase2_output_dir`，所以 baseline 只要把输出目录做成 Phase2 的结构即可。

* * *

1. Phase4_baselines：统一输入输出接口（MD 定义）

-----------------------------------

### 1.1 输入（统一给所有 baseline）

* **设备点云**：`pointclouds/<device>.las`（你 Phase2 已有自动发现与类型推断逻辑）

* **全站可行栅格**：`outputs/phase1_feasible_space/station_grid_map.npz`

* **Phase1 报告**：`outputs/phase1_feasible_space/feasible_space_report.json`（用里面的旋转角、`d_eff` 等）

* **相机模型**：沿用你 Phase2 config 的 `camera`（HFOV/VFOV/near/far/pitch=0）

* **预算约束（强烈建议加）**：
  
  * 每设备最大视点数 `K_device_max`
  
  * 或总视点数 `K_total`
  
  * 或总里程/时间预算（更贴近真实任务）

### 1.2 输出（必须对齐你 Phase2）

建议目录结构：
    outputs/baselines/<method_name>/phase2_single_device/<device>/
        viewpoints_capture_raw.csv
        viewpoints_capture_ordered.csv
        viewpoints_nav_ordered.csv
        path_waypoints.csv                  (可选，但建议有)
        path_waypoints_display.csv          (可选)
        device_report.json                  (必须)
    outputs/baselines/<method_name>/phase2_single_device/phase2_single_device_summary.json (可选)

#### A) `viewpoints_capture_ordered.csv` 必要字段（你现成格式）

表头必须一致（你 Phase2 写死了这些字段，Phase3也会读）：
    viewpoint_id,device,device_type,role,x,y,z,yaw_deg,pitch_deg,roll_deg,target_x,target_y,target_z,layer_k,component_id

**Baseline 适配规则（建议统一这样做，最省事且合理）：**

* `pitch_deg=0, roll_deg=0`

* `target_*`：用“设备点云质心（或设备体素质心）”

* `yaw_deg`：`atan2(target_y - y, target_x - x)`（你 Phase2 就是这么算的）

* `layer_k/component_id`：baseline 没有的话可以填 `0/0`（不影响 Phase3）

#### B) `viewpoints_nav_ordered.csv`

    x,y,z,source_viewpoint_id

#### C) `device_report.json`（关键：用于汇总与论文写表）

最少写这些字段就够 Phase3/论文使用（字段名你可以完全沿用 Phase2 的报告）：
    {
      "ordered_viewpoints": 123,
      "coverage": {"enabled": true, "coverage_ratio": 0.52},
      "planning": {"total_length_m": 87.3, "segment_success_ratio": 0.94},
      "timing": {"viewpoint_generation_time_s": 1.23, "viewpoint_reorder_time_s": 0.12, "path_planning_time_s": 3.45},
      "feasibility": {
        "raw_infeasible_ratio": 0.31,
        "avg_projection_distance_m": 0.42
      }
    }

> `feasibility.*` 是我建议你新增的：**专门用来“打 baseline 的痛点”**（变电站禁飞/安全距离约束下，baseline 天然更容易生成不可行视点/不可行连线）。

* * *

2. 三套对比协议：既“公平”，又能突出变电站特殊性

--------------------------

为了既稳又“好看”，我建议你同时输出 **两套主结果 + 一套补充分析**（补充放附录也行）：

### 协议 1：Strict（不修复）——突出“强约束下可执行性”

* baseline 输出的视点/连线 **不做投影修复**

* 直接统计：
  
  * `raw_infeasible_ratio`（视点落在禁飞区/碰撞体素）
  
  * `direct_connection_fail_ratio`（直连失败比例）
  
  * `astar_fail_ratio`（A* 也失败的比例）

* **意义**：这就是变电站特殊性：不是“能覆盖就行”，而是“必须安全可飞”。你的方法 Phase1/Phase2 是“约束内生”，baseline 多数是“约束外生”。

### 协议 2：Projected（最小修复）——用于给出可比的“最终任务轨迹指标”

因为不修复的话很多 baseline 根本无法形成完整任务，你需要一套“可比结果”：

* 对不可行视点做**最小修复**：沿法向/近邻搜索最近可行栅格点（你 Phase2 自己也在做类似事）

* 然后再进入 Phase3，得到全站：
  
  * `path_length(smoothed)`
  
  * `segment_success_ratio`
  
  * `trackability_score`
  
  * `low_clearance_ratio`

* **意义**：即使给 baseline 一次“工程落地修复”，它也通常会出现：
  
  * 视点整体被推远/被迫绕行 → 路径更长
  
  * 拐点/jerk 增大 → trackability 变差
  
  * 低净空段比例上升或成功率下降

### 协议 3（可选补充）：Budget 曲线（防止审稿人说“你视点更多所以覆盖高”）

选 1 个代表设备（比如 main_transformer），做一条曲线：

* 固定 `K = 10, 20, 30, ...`

* 对每种 K 截断/采样 baseline 输出

* 画 `coverage(K)`、`path_length(K)` 或 `coverage vs length`

> 这张图通常很有杀伤力：你的方法往往在“小 K”就上得很快，而 baseline 要么覆盖上不去，要么上去了但长度爆炸。

* * *

3. 具体 baseline 怎么接入（你选的 2~3 个）

------------------------------

### Baseline A：FC-Planner（ICRA 2024，已开源，偏 CPP/coverage）

FC-Planner 的 repo 明确给出：运行后会生成离散轨迹文件（例如 `TrajInfoMBS.txt`），可直接拿来做“轨迹/视点序列”。([GitHub](https://github.com/HKUST-Aerial-Robotics/FC-Planner "GitHub - HKUST-Aerial-Robotics/FC-Planner: [ICRA'24 Best UAV Paper Award Finalist] An Efficient Global Planner for Aerial Coverage"))

**推荐接入方式（最低成本、对你最有利）：**

1. **按设备跑**（每个设备点云当作一个 scene 输入）
   
   * 好处：直接对齐你的 Phase2“单设备覆盖率”指标

2. 解析其输出轨迹为 `(x,y,z)` 序列 → 写成你的 `viewpoints_nav_ordered.csv`

3. 用“设备质心”为 target 生成 yaw → 写 `viewpoints_capture_ordered.csv`

4. 直接用你的 Phase2 coverage evaluator 计算 coverage（而不是用它论文里的指标），保证公平

5. 把这份 Phase2 输出目录交给 Phase3，跑全站任务

**它在你场景下容易吃亏的点（你可在论文里解释）：**

* 它主要目标是“大场景快速覆盖”，但你是“禁飞/净空/可跟踪性”优先；在导线/设备密集区，它生成的局部策略更容易被 Phase1 约束打断，导致绕行与失败段上升。

* * *

### Baseline B：PredRecon（ICRA 2023，开源，偏 active reconstruction）

PredRecon repo 显示它是 ICRA 2023，并且开源（GPL），但它的完整系统依赖 ROS + AirSim/Unreal。([GitHub](https://github.com/HKUST-Aerial-Robotics/PredRecon "GitHub - HKUST-Aerial-Robotics/PredRecon: [ICRA'23] A Prediction-boosted Planner for Fast and High-quality Autonomous Aerial Reconstruction"))

这里我给你两条路线，你按“复现成本”选其一即可：

**路线 B1（更“论文对标”，但工程更重）：完整跑 PredRecon 系统**

* 优点：最能回应审稿人“learning-based active reconstruction”

* 缺点：你需要把变电站点云/场景接入它的仿真/重建链路（工作量大）

**路线 B2（我更推荐，性价比高）：只复现/实现它的“planner 逻辑”，对接你的点云与栅格**

* 你把它当作“**active reconstruction 风格 baseline**”：核心是“基于当前模型的未覆盖区域选择下一批视点 + 分层/层级规划”

* 输入直接用你的：
  
  * 设备表面体素/采样点（已有）
  
  * Phase1 栅格（已有）

* 输出走你的 Phase2 格式

* 论文表述上写清楚：
  
  * “We re-implemented the planning component following PredRecon, and evaluate under our substation feasibility constraints and camera model.”

* 这样既回应审稿意见，又不会陷入“搭 Unreal 场景”泥潭。

> 如果你担心审稿人质疑“不是原作者代码”，那就把它定位为：**PredRecon-style planning baseline（按论文复现）**，并在附录给出实现细节与参数。

PredRecon 在你场景下也很容易吃亏：它追求“信息增益/重建效率”，但对“禁飞区 + 净空 + 可跟踪性”不是内生优化目标，经过你 Phase1 约束投影后通常会出现明显性能折损。

* * *

### Baseline C（经典 established，专门用于“审稿人闭嘴”）：Structural Inspection Planner / SIP（开源）

SIP 的官方 repo说明它基于三角网格（mesh）并开源。([GitHub](https://github.com/ethz-asl/StructuralInspectionPlanner?utm_source=chatgpt.com "ethz-asl/StructuralInspectionPlanner"))

**接入要点：**

* 你需要把设备点云转为 mesh（CloudCompare / Poisson / BPA 均可）

* 它输出 viewpoint tour → 同样写成你的 Phase2 csv

* 这类方法在“导线细结构 + 点云孔洞 + 强禁飞约束”下通常会明显吃亏：  
  mesh 误差/孔洞会让视点优化目标漂移，最终可执行性与效率都差（而你是点云/体素原生处理，更稳）

* * *

4. 你应该产出哪些表/图（直接对应论文 Results）

-----------------------------

### 4.1 单设备表（主文一张表就够）

对每个设备（8 个）统计：

* `coverage_ratio`

* `#viewpoints`

* `local_path_length`（Phase2 可选局部路径）

* `planning_success_ratio`（直连/ A* 成功率）

* `runtime`（generation + planning）

### 4.2 全站任务表（最关键，建议主文 1 张表）

对 Phase3 输出的 **smoothed** 路径统计：

* `path_length_smoothed`

* `segment_success_ratio`

* `trackability_score` +（max turn / jerk proxy / turning density）

* `low_clearance_ratio` +（min/avg clearance）

* `order_match`（区域顺序、严格设备顺序）

### 4.3 “变电站特殊性杀伤图”（建议至少 1 张）

* `raw_infeasible_ratio`（Strict，不修复）柱状图：一眼说明 baseline 不适配强约束环境

* 或 “Budget 曲线”一张：coverage vs #views / vs path length

* * *

5. 最小落地步骤（你现在就能按这个开干）

---------------------

1. 新建 `phases/phase4_baselines/`

2. 写一个统一函数：`export_as_phase2_format(method_name, device_name, viewpoints_xyz_ordered)`
   
   * 自动补 yaw/target
   
   * 自动做“Strict 统计 + Projected 修复（可开关）”

3. 生成输出到：`outputs/baselines/<method>/phase2_single_device/...`

4. 跑 Phase3：把 `config.phase3.station_mission.json` 的
   
   * `paths.phase2_output_dir` 改成对应 baseline 目录即可

5. 把 Phase2 summary + Phase3 summary 的关键字段汇总成一张总表（你已有很多汇总 json/csv 了）

* * *

如果你愿意，我下一条可以直接给你一个**Phase4_baselines 的目录结构 + 适配器伪代码（Python）**，以及“Strict/Projected 修复”的最小实现逻辑（完全对齐你 Phase2 的可行搜索方式），这样你基本只需要写 2~3 个 parser（把 FC-Planner / PredRecon-style / SIP 的输出读进来）就能跑全套对比。
