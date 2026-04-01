#!/usr/bin/env python3
"""Generate synthetic /imu topic for TUM ROS2 MCAP bags from /tf ground-truth poses.

Entry point for the synthetic_imu_tum_rgbd package.
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path
from typing import Sequence

from rosbags.typesys import Stores, get_typestore

from lib.bag_io import (
    bag_has_imu,
    build_imu_raw_messages,
    discover_input_bags,
    extract_tf_pose_samples,
    write_output_bag,
)
from lib.config_loader import load_imu_config
from lib.imu_synthesis import (
    choose_child_frame,
    normalize_frame,
    stable_seed_from_name,
    synthesize_imu_samples,
)
from lib.models import ImuConfig, ProcessingError
from lib.ros1_converter import handle_ros1_bags

import numpy as np


def _process_one_bag(
    bag_path: Path,
    output_dir: Path,
    typestore,
    imu_config: ImuConfig,
    compress: bool,
) -> None:
    bag_name = bag_path.name
    print(f"\n=== Processing bag: {bag_name} ===")

    if bag_has_imu(bag_path):
        print(f"Skipping '{bag_name}': bag already contains IMU messages.")
        return

    samples, tf_msg_count, bag_start_ns, bag_end_ns = extract_tf_pose_samples(bag_path, typestore)
    print(f"Found {tf_msg_count} /tf messages.")

    target_frame = imu_config.imu_frame if imu_config.imu_frame_mode else imu_config.body_frame
    frame = choose_child_frame(samples, target_frame)
    pose_samples = [s for s in samples if normalize_frame(s.child_frame_id) == frame]
    if len(pose_samples) < 2:
        raise ProcessingError(
            f"Not enough pose samples for child frame '{frame}' ({len(pose_samples)})."
        )

    seed = stable_seed_from_name(bag_name)
    rng = np.random.default_rng(seed)

    imu_samples = synthesize_imu_samples(
        pose_samples=pose_samples,
        bag_start_ns=bag_start_ns,
        bag_end_ns=bag_end_ns,
        imu_config=imu_config,
        rng=rng,
    )

    mode_label = "imu_frame_mode (T_i_b disregarded)" if imu_config.imu_frame_mode else "body_frame_mode"
    print(
        f"Using TF child frame '{frame}' [{mode_label}]. "
        f"Generated {len(imu_samples)} IMU messages at {imu_config.update_rate:.1f} Hz."
    )

    output_frame_id = imu_config.imu_frame if imu_config.imu_frame_mode else imu_config.body_frame
    imu_raw_messages, gyro_var, accel_var = build_imu_raw_messages(
        imu_samples,
        frame_id=output_frame_id,
        imu_config=imu_config,
        typestore=typestore,
    )
    print(f"IMU covariance diag: gyro={gyro_var:.6e}, accel={accel_var:.6e}.")

    output_bag = output_dir / f"{bag_path.name}_imu"
    write_output_bag(
        input_bag=bag_path,
        output_bag=output_bag,
        imu_raw_messages=imu_raw_messages,
        typestore=typestore,
        compress=compress,
        imu_topic=imu_config.rostopic,
    )
    print(f"Wrote output bag: {output_bag}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="generate_imu_bag.py",
        description="Synthesize /imu topic for TUM MCAP bags from /tf ground truth.",
    )
    parser.add_argument("--compress", action="store_true", help="Compress output MCAP bag with zstd.")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    script_dir = Path(__file__).resolve().parent
    config_path = script_dir / "config" / "kalibr_imu_chain.yaml"
    input_dir  = script_dir / "Input"
    output_dir = script_dir / "output"

    try:
        imu_config = load_imu_config(config_path)
    except ProcessingError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    if not input_dir.exists() or not input_dir.is_dir():
        print(f"Error: Input directory not found: {input_dir}", file=sys.stderr)
        return 2

    handle_ros1_bags(input_dir, script_dir)

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    bags = discover_input_bags(input_dir)

    if not bags:
        print(f"No input bag directories found in {input_dir}.")
        return 0

    if output_dir.exists():
        clear_response = input(f"\nOutput directory already exists: {output_dir}\nClear it before processing? [Y/n] ").strip().lower()
        if clear_response in {"", "y", "yes"}:
            shutil.rmtree(output_dir)
            output_dir.mkdir(parents=True)
        else:
            print("Keeping existing output directory. Existing bags with the same name will fail.")
    else:
        output_dir.mkdir(parents=True)

    failures: list[tuple[Path, Exception]] = []
    for bag in bags:
        try:
            _process_one_bag(
                bag_path=bag,
                output_dir=output_dir,
                typestore=typestore,
                imu_config=imu_config,
                compress=args.compress,
            )
        except KeyboardInterrupt:
            print("Interrupted by user.", file=sys.stderr)
            return 130
        except Exception as exc:
            failures.append((bag, exc))
            print(f"Failed '{bag}': {exc}", file=sys.stderr)

    if failures:
        print("\nSome bags failed:", file=sys.stderr)
        for bag, exc in failures:
            print(f"  - {bag}: {exc}", file=sys.stderr)
        return 1

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
