#!/usr/bin/env python3
"""Phase 4 wrapper entrypoint."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> None:
    default_cfg = Path(__file__).resolve().parent / "configs" / "config.phase4.baselines.json"
    parser = argparse.ArgumentParser(description="Run Phase 4 baseline builder.")
    parser.add_argument("--config", type=Path, default=default_cfg)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    script = repo_root / "phases" / "phase4_baselines" / "build_phase4_baselines.py"
    cfg = args.config.resolve() if args.config.is_absolute() else (repo_root / args.config).resolve()
    cmd = [sys.executable, str(script), "--config", str(cfg)]
    raise SystemExit(subprocess.call(cmd, cwd=str(repo_root)))


if __name__ == "__main__":
    main()
