# Baselines vs Ours: Comparison Report

Updated at: 2026-02-25 19:31:13

## Naming Convention

- Unified 5-method naming in core comparison figures:
  - Ours-balanced
  - FC-Planner (CON)
  - FC-Planner (UN)
  - PredRecon (CON)
  - PredRecon (UN)

## Figure-by-Figure Notes

- `phase2_comparison_with_ours.png`: 8-device aggregate Phase2 comparison; planning time uses log scale to reveal tiny unconstrained bars.
- `phase2_main_transformer_comparison_with_ours.png`: single-object (main_transformer) comparison.
- `phase2_shelf4_comparison_with_ours.png`: single-object (shelf4) comparison.
- `phase2_shelf1_comparison_with_ours.png`: single-object (shelf1) comparison (new backup figure).
- `phase3_comparison_with_ours.png`: balanced-scenario station-level metric bars.
- `phase3_composite_coverage_success_with_ours.png`: utility-style composite `coverage x success`.
- `phase3_tradeoff_scatter_with_ours.png`: includes Ours-conservative/Ours-balanced/Ours-aggressive and baseline points.
- `phase3_runtime_comparison_with_ours.png`: lightweight-config runtime-only comparison.

## Single-Object Addendum

- shelf1 key values:
  - Ours-balanced: coverage=0.950, viewpoints=220, planning_s=19.586, success=0.840
  - FC-Planner (CON): coverage=0.583, planning_s=23.057, success=0.821
  - FC-Planner (UN): coverage=0.477, planning_s=0.000726, success=0.000

## Runtime Comparison Note

- Runtime plot uses lightweight Phase3 config (same for ours-lightweight and baselines): no GIF/simulation/sweep/extra experiments.
- This keeps runtime comparison focused on core station-mission generation cost.