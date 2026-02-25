# Phase4 对比实验设计与执行计划

## 1. 对用户构想的评估结论

你的构想总体合理，核心逻辑正确：

- 必做全站级对比（有/无 Phase1 约束）是主证据链。
- 单物体实验可作为补充，验证复杂局部几何下的差异。

需要精简的点：

- 为控制规模，单物体不建议“全设备都做”，建议只做 1 个代表设备（如 `main_transformer`）。
- 不建议扩展过多协议，最终论文主文保留 1~2 张表 + 1~2 张图即可。

## 2. 推荐实验矩阵（控规模版）

### E1. 全站-约束内（必做）

定义：

- baseline 生成 viewpoints/path 后，严格施加 Phase1 可行域约束。
- 用现有评测口径计算全站 coverage、路径长度、可跟踪性等。

目的：

- 对比“在变电站安全约束下谁更可执行、谁更稳健”。

### E2. 全站-约束外/原生（必做）

定义：

- baseline 按其原生约束（或弱约束）生成结果，不提前用 Phase1 可行域限制。
- 再统计其路径侵入 Phase1 不可行区域的程度（次数/长度占比）。

目的：

- 证明 baseline 即使覆盖更高，也可能不满足安全与工程可执行要求。

### E3. 单物体代表设备（选做）

定义：

- 仅选择 `main_transformer`（或另一个几何复杂设备）做单设备对比。

目的：

- 用一张小表或一条曲线补充说明“局部复杂几何下我们方法优势”。

## 3. 指标设计（与现有体系对齐）

主指标（用于主文表格）：

1. `estimated_global_coverage_ratio`（或设备级 `coverage_ratio`）
2. `path_length_smoothed_m`
3. `segment_success_ratio`
4. `trackability_score_0_100`（smoothed）
5. `runtime`（核心规划耗时）

约束/安全指标（用于证明 baseline 不适配）：

1. `raw_infeasible_ratio`（视点不可行比例）
2. `violation_length_ratio`（路径落入不可行区长度占比）
3. `low_clearance_ratio`

## 4. 协议定义（保证公平且能落地）

### P1 Strict（不修复）

- baseline 原始输出直接评估可行性与违规程度。
- 用于展示“原生策略在强约束场景下的不适配”。

### P2 Projected（最小修复）

- 将不可行视点投影到最近可行栅格点后，再走全流程评测。
- 用于给出可对比的最终任务指标（路径、可跟踪性、覆盖率）。

说明：P1 与 P2 都保留，主文报告 P2 的可比结果，P1 作为关键补充证据。

## 5. 论文产出建议（严格控量）

主文建议：

1. 表1（全站主结果）：我们方法 vs FC-Lite vs PredRecon-Lite（可选 + SIP-Lite）
2. 表2（安全/可执行性）：Strict 协议下的 infeasible/violation 指标
3. 图1（全站路径可视化）：我们方法 vs 最强 baseline
4. 图2（可选）：代表设备的 coverage-length 或 budget 曲线

## 6. Phase4 执行计划（当前仅规划，不写代码）

Step 1: 冻结 baseline 与协议  
- 固定 `FC-Lite + PredRecon-Lite` 为必做，`SIP-Lite` 为选做。  
- 固定 `Strict + Projected` 双协议。  

Step 2: 冻结统一 I/O 规范  
- baseline 输出强制对齐 Phase2 文件格式与字段。  
- 输出目录统一到 `outputs/phase4_baselines/<method>/...`。  

Step 3: 先打通单设备最小闭环  
- 在 `main_transformer` 上跑通每个 baseline 的 viewpoints/path 输出。  
- 完成与 Phase2 evaluator 对齐。  

Step 4: 扩展到全站并接入 Phase3  
- 使用 Phase3 全站拼接与轨迹评测，得到主对比指标。  

Step 5: 汇总与制表  
- 产出主文 1~2 张表、1~2 张图。  
- 形成最终对比分析结论。  

## 7. 当前阶段完成标准

本文件与 `PHASE4_BASELINE_SELECTION_AND_FEASIBILITY.md` 一起作为 Phase4 初步方案冻结文档。  
下一阶段再进入具体代码实现与运行，不在本阶段执行。
