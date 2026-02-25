# Phase2 TODO / Handoff Checklist

## 0. 当前状态（已完成）
- [x] 设备筛选：默认处理 8 个待巡检设备（排除 buildings / wires）。  
- [x] 早期地面切除已前移到体素阶段（非仅可视化隐藏）。  
- [x] `shelf3` 底部离群过滤已启用。  
- [x] 单设备视点生成、重排、coverage、局部路径流程可稳定运行。  
- [x] main_transformer baseline 对比输出可用。  
- [x] 关键输出与汇总统计一致性检查通过。

## 1. P0（冻结前必须完成）
1. 锁定 Phase2 配置文件：  
`phases/phase2_single_device/configs/config.phase2.single_device.json`

2. 每次重跑后执行最小检查：  
- `outputs/phase2_single_device/phase2_single_device_summary.csv/json`  
- 每设备 `device_report.json` 是否存在且字段齐全  
- 每设备 `coverage_map.png`、`device_plan.png` 是否生成  

3. 保存一份“冻结版本说明”：  
- 当前参数  
- 当前指标  
- 当前已知限制（tube 低 coverage）

## 2. P1（论文材料）
1. 结果表建议至少包含：  
- `ordered_viewpoints`  
- `coverage_ratio`  
- `path_length_m`  
- `viewpoint_generation_time_s`  
- `path_planning_time_s`

2. baseline 小实验建议写法：  
- 保留并如实报告（不要删除）  
- 强调 trade-off（局部最短 vs 结构化顺序先验）  
- 详细数值放附录，主文写结论

3. 低 coverage 设备解释口径：  
- 归因于可达性/遮挡约束，不盲目追求 100%  
- 可作为多平台协同（人工/UGV）扩展点

## 3. P2（与 Phase3 交接）
1. 直接作为 Phase3 输入的文件：  
- `viewpoints_nav_ordered.csv`（导航）  
- `viewpoints_capture_ordered.csv`（拍摄姿态）  
- `device_report.json`（质量与复杂度先验）

2. Phase3 优先任务：  
- 跨设备 viewpoint merge/dedup  
- 设备级访问顺序 + 点级细化两层优化  
- 在 Phase1 可通行空域上统一全局路径规划

3. 扫参影响评估建议：  
- Phase2 只做机制验证  
- 主要结论在 Phase3（总路径、总耗时、可达率、安全裕量）

## 4. 一键复现命令（保留）
```powershell
pip install -r phases/phase2_single_device/requirements.phase2.txt
python -m py_compile phases/phase2_single_device/build_single_device_inspection.py
python phases/phase2_single_device/build_single_device_inspection.py --config phases/phase2_single_device/configs/config.phase2.single_device.json
```
