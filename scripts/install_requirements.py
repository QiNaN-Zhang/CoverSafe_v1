#!/usr/bin/env python3
"""Install all project runtime dependencies from the root requirements file."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Install project dependencies.")
    parser.add_argument(
        "--requirements",
        type=Path,
        default=Path("requirements.txt"),
        help="Path to requirements file (default: requirements.txt).",
    )
    parser.add_argument(
        "--upgrade-pip",
        action="store_true",
        help="Upgrade pip before installing dependencies.",
    )
    args = parser.parse_args()

    req = args.requirements.resolve()
    if not req.exists():
        print(f"Requirements file not found: {req}", flush=True)
        return 1

    if args.upgrade_pip:
        rc = subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])
        if rc != 0:
            return rc

    cmd = [sys.executable, "-m", "pip", "install", "-r", str(req)]
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
