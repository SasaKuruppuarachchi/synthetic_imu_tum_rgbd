"""Detect and convert ROS1 .bag files in the Input directory."""
from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path


def find_ros1_bags(input_dir: Path) -> list[Path]:
    """Return all .bag files found anywhere under input_dir."""
    return sorted(input_dir.rglob("*.bag"))


def handle_ros1_bags(input_dir: Path, script_dir: Path) -> None:
    """Detect ROS1 .bag files, prompt user, convert to MCAP.

    Bags whose corresponding ``{stem}_mcap`` directory already exists in
    *input_dir* are silently skipped (no re-conversion).  Source .bag files
    are only removed when the user explicitly confirms.
    """
    bags = find_ros1_bags(input_dir)
    if not bags:
        return

    # Filter out bags already converted (their _mcap dir already lives in Input/)
    bags_needing_conversion = [b for b in bags if not (input_dir / f"{b.stem}_mcap").exists()]

    if not bags_needing_conversion:
        print(f"\n{len(bags)} ROS1 .bag file(s) detected but all already converted — skipping re-conversion.")
        return

    print(f"\nDetected {len(bags_needing_conversion)} ROS1 .bag file(s) requiring conversion in {input_dir}:")
    for b in bags_needing_conversion:
        print(f"  {b.name}")

    convert_response = input("Convert to MCAP? [Y/n] ").strip().lower()
    if convert_response not in {"", "y", "yes"}:
        print("Skipping ROS1 conversion. .bag files will not be processed.")
        return

    bags = bags_needing_conversion

    convert_script = script_dir / "scripts" / "ros1_bag_to_ros2_mcap_convert.sh"
    if not convert_script.exists():
        print(f"Warning: conversion script not found at {convert_script}. Skipping.", file=sys.stderr)
        return []

    tmp_dir = input_dir / "_ros1_convert_tmp"
    tmp_dir.mkdir(parents=True, exist_ok=True)

    try:
        print(f"Running conversion script for {len(bags)} bag(s)...")
        subprocess.run(["bash", str(convert_script), str(input_dir), str(tmp_dir)], check=True)
    except subprocess.CalledProcessError as exc:
        print(f"Conversion script failed (exit {exc.returncode}). Aborting ROS1 import.", file=sys.stderr)
        shutil.rmtree(tmp_dir, ignore_errors=True)
        return []

    mcap_root = tmp_dir / "mcap"
    if not mcap_root.exists():
        print("Warning: no MCAP output directory after conversion.", file=sys.stderr)
        shutil.rmtree(tmp_dir, ignore_errors=True)
        return []

    converted_dirs: list[Path] = []
    for mcap_dir in sorted(mcap_root.iterdir()):
        if not mcap_dir.is_dir():
            continue
        dest = input_dir / mcap_dir.name
        if dest.exists():
            print(f"  [skip-move] {mcap_dir.name} already exists in Input/")
        else:
            mcap_dir.rename(dest)
            print(f"  [moved] {mcap_dir.name} -> Input/")
        converted_dirs.append(input_dir / mcap_dir.name)

    shutil.rmtree(tmp_dir, ignore_errors=True)

    remove_response = input("Remove source .bag files? [Y/n] ").strip().lower()
    if remove_response in {"", "y", "yes"}:
        for bag in bags:
            try:
                bag.unlink()
                print(f"  [removed] {bag.name}")
            except OSError as exc:
                print(f"  [warn] Could not remove {bag.name}: {exc}", file=sys.stderr)
    else:
        print("Source .bag files kept.")
    print("ROS1 conversion complete.")
