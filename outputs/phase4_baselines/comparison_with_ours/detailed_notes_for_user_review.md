# Detailed Notes For Comparison Figures

Updated at: 2026-02-25 19:31:13

## A) Phase3 tradeoff plot updates
- Top-left note changed to: `CON: constrained projected; UN: unconstrained native.`
- Title second line changed to: `*Markersize:segment success`.
- Label text uses ASCII parentheses to avoid garbling:
  - Ours-conservative, Ours-balanced, Ours-aggressive
  - FC-Planner (CON), FC-Planner (UN), PredRecon (CON), PredRecon (UN)
- Ours-aggressive label is placed in the upper-right vicinity of its marker.

## B) New single-object figure
- Added `phase2_shelf1_comparison_with_ours.png` and `phase2_shelf1_comparison_with_ours.csv`.
- Existing main_transformer and shelf4 single-object figures were kept unchanged as requested.

## C) Runtime figure update
- Removed the upper-left note box in `phase3_runtime_comparison_with_ours.png`.
- Other visual elements are unchanged.

## D) Naming normalization in core multi-method figures
- Applied unified names in key comparison figures and CSVs:
  - Ours-balanced
  - FC-Planner (CON)
  - FC-Planner (UN)
  - PredRecon (CON)
  - PredRecon (UN)

## E) Image coverage check
- All current images in this folder now have explicit references and interpretation in the two markdown documents.