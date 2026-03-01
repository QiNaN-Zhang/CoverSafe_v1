#!/usr/bin/env python3
"""Download required LAS point clouds into the local pointclouds directory."""

from __future__ import annotations

import argparse
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Iterable, List


POINTCLOUD_FILES: List[str] = [
    "building1.las",
    "building2.las",
    "building3.las",
    "capacitor.las",
    "main_transformer.las",
    "shelf1.las",
    "shelf2.las",
    "shelf3.las",
    "shelf4.las",
    "substation_baseline.las",
    "tube1.las",
    "tube2.las",
    "wires1.las",
    "wires2.las",
]


def build_url(source: str, repo_id: str, revision: str, base_url: str, filename: str) -> str:
    if source == "hf":
        if not repo_id:
            raise ValueError("--repo-id is required when --source hf is used.")
        return f"https://huggingface.co/datasets/{repo_id}/resolve/{revision}/{filename}?download=true"
    if source == "url":
        if not base_url:
            raise ValueError("--base-url is required when --source url is used.")
        return f"{base_url.rstrip('/')}/{filename}"
    raise ValueError(f"Unsupported source: {source}")


def download_one(url: str, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + ".part")
    try:
        urllib.request.urlretrieve(url, tmp)
        tmp.replace(target)
    except Exception:
        if tmp.exists():
            tmp.unlink(missing_ok=True)
        raise


def validate_existing(target_dir: Path, filenames: Iterable[str]) -> List[str]:
    missing = []
    for name in filenames:
        if not (target_dir / name).exists():
            missing.append(name)
    return missing


def main() -> int:
    parser = argparse.ArgumentParser(description="Download project LAS files to pointclouds/.")
    parser.add_argument("--source", choices=["hf", "url"], default="hf")
    parser.add_argument("--repo-id", default="", help="Hugging Face dataset repo id, e.g. your-org/substation-las.")
    parser.add_argument("--revision", default="main", help="Dataset revision for Hugging Face source.")
    parser.add_argument("--base-url", default="", help="HTTP base URL when --source url is used.")
    parser.add_argument("--target-dir", default="pointclouds", help="Local target directory for LAS files.")
    parser.add_argument("--force", action="store_true", help="Re-download files even if they already exist.")
    args = parser.parse_args()

    target_dir = Path(args.target_dir).resolve()
    print(f"Target directory: {target_dir}")

    failures: List[str] = []
    for idx, filename in enumerate(POINTCLOUD_FILES, start=1):
        target = target_dir / filename
        if target.exists() and not args.force:
            print(f"[{idx:02d}/{len(POINTCLOUD_FILES):02d}] Skip existing: {filename}")
            continue

        try:
            url = build_url(args.source, args.repo_id, args.revision, args.base_url, filename)
            print(f"[{idx:02d}/{len(POINTCLOUD_FILES):02d}] Download: {filename}")
            download_one(url, target)
        except (urllib.error.URLError, urllib.error.HTTPError, ValueError, OSError) as exc:
            failures.append(f"{filename}: {exc}")
            print(f"  FAILED: {exc}")

    missing = validate_existing(target_dir, POINTCLOUD_FILES)
    if failures or missing:
        print("\nDownload finished with issues.")
        if failures:
            print("Failed files:")
            for item in failures:
                print(f"  - {item}")
        if missing:
            print("Missing files after download:")
            for name in missing:
                print(f"  - {name}")
        return 1

    print("\nAll required pointcloud files are available.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
