# Phase4 实现状态（首版）

## 已实现内容

1. 两个 baseline 轻量实现：
   - `fc_planner_lite`
   - `predrecon_lite`
2. 两种场景协议：
   - `constrained_projected`
   - `unconstrained_native`
3. 统一输出为 Phase2 兼容格式，可直接供 Phase3 读取：
   - `viewpoints_capture_raw.csv`
   - `viewpoints_capture_ordered.csv`
   - `viewpoints_nav_ordered.csv`
   - `path_waypoints.csv`
   - `path_waypoints_display.csv`
   - `device_report.json`
   - `phase2_single_device_summary.csv/json`
4. 运行入口与配置：
   - `build_phase4_baselines.py`
   - `run_phase4.py`
   - `configs/config.phase4.baselines.json`

## 已完成自检

1. 脚本语法检查通过：
   - `python -m py_compile phases/phase4_baselines/build_phase4_baselines.py`
   - `python -m py_compile phases/phase4_baselines/run_phase4.py`
2. smoke 运行通过（`main_transformer` 单设备）：
   - `configs/config.phase4.smoke_all.json`
   - 输出位于 `outputs/phase4_baselines/*`

## 下一步建议

1. 先按全设备运行 `config.phase4.baselines.json` 生成完整 baseline 输出。
2. 将对应目录作为 `phase3` 的 `phase2_output_dir` 输入，跑全站任务。
3. 汇总 `device_report.json` 与 `phase3_station_mission_summary.json` 形成论文表格。


## Phase4 Technical Check (2026-02-25)

- See: `docs/PHASE4_TECHNICAL_CHECK_AND_OUTPUT_GUIDE.md`
