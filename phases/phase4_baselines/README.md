# Phase 4: Baseline 对比实验

本目录用于 Phase4（baseline 对比）实现与文档管理。

## 1. 阶段目标

在不修改 Phase1~Phase3 代码与输出的前提下，选择 2~3 个 baseline，做轻量复现并统一到现有评测体系，形成可用于论文的对比结论：

- 主结论：在变电站安全约束与可跟踪性要求下，我们方法更适配。
- 辅结论：baseline 在无约束/弱约束下可能覆盖更高，但不满足工程安全约束。

## 2. 当前 baseline 选型结论

- 必做 1: `FC-Planner`（ICRA 2024）
- 必做 2: `PredRecon`（ICRA 2023）
- 选做 3: `StructuralInspectionPlanner / SIP`（ICRA 2015, classic established）

说明：3 个候选都包含 ROS/Ubuntu 依赖，不适合在 Windows 下直接完整运行；Phase4 采用“提取核心规划思想 + Python 轻量复现 + 统一评测输出”的策略。

## 3. 文档入口

- 选型与可复现性分析：
  - `docs/PHASE4_BASELINE_SELECTION_AND_FEASIBILITY.md`
- 对比实验设计与执行计划：
  - `docs/PHASE4_EXPERIMENT_DESIGN_AND_PLAN.md`
- 首版实现状态：
  - `docs/PHASE4_IMPLEMENTATION_STATUS.md`
- 外部调研建议：
  - `advice.md`

## 4. 目录约定

- 代码与文档：`phases/phase4_baselines`
- 输出：`outputs/phase4_baselines`
- baseline 原始仓库：`baseline_codes/*`
- 论文资料：`baseline_papers/*`

## 5. 当前阶段状态

已完成首版可运行实现：

- `FC-Planner-Lite` 与 `PredRecon-Lite` 两个 baseline。
- `constrained_projected` 与 `unconstrained_native` 两个场景协议。
- 输出对齐 Phase2 格式，可直接作为 Phase3 输入目录。

当前定位是“轻量复现 + 快速对比”，后续可按实验结果再做参数微调。

## 6. 运行入口

主脚本：

- `phases/phase4_baselines/build_phase4_baselines.py`

包装入口：

- `phases/phase4_baselines/run_phase4.py`

默认配置：

- `phases/phase4_baselines/configs/config.phase4.baselines.json`

示例命令（项目根目录执行）：

```powershell
python phases/phase4_baselines/run_phase4.py --config phases/phase4_baselines/configs/config.phase4.baselines.json
```

快速 smoke（单设备）：

```powershell
python phases/phase4_baselines/run_phase4.py --config phases/phase4_baselines/configs/config.phase4.smoke_all.json
```

可用配置项：

- `device_name_whitelist`: 仅运行指定设备名列表（如 `["main_transformer"]`）
- `max_devices`: 仅运行前 N 个设备（`0` 表示不限制）

输出结构（按方法和场景分层）：

- `outputs/phase4_baselines/<method>/<scenario>/phase2_single_device/<device>/*`
- `outputs/phase4_baselines/phase4_baselines_run_summary.json`


## Phase4 Technical Check (2026-02-25)

- See: `docs/PHASE4_TECHNICAL_CHECK_AND_OUTPUT_GUIDE.md`
