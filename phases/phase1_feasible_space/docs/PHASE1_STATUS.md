# Phase 1 Status

## Current status

- Core script is now `phases/phase1_feasible_space/build_feasible_space.py`.
- Phase directory provides config + wrapper entrypoint (`run_phase1.py`).
- Output target is unified to `outputs/phase1_feasible_space/`.

## Notes about latest output

- Stage 6 and Stage 7 are still major runtime hotspots (inflate operations).
- Visual products and sweep artifacts are available and suitable as Phase 1 references.
- `wires` are intentionally high-priority in overlap handling for conservative no-fly modeling.

## Carry-over risks for later phases

- Dynamic safety distance gains should be validated with mission-level metrics, not only free-space ratio.
- Sweep conclusions should be revisited after Phase 3 route optimization is in place.
