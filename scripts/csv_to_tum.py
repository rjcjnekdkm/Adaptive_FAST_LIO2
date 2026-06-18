#!/usr/bin/env python3
"""Convert experiment CSV trajectories to evo TUM format.

Output columns:
timestamp tx ty tz qx qy qz qw
"""

import argparse
import csv
from pathlib import Path


def timestamp_to_sec(value: str) -> float:
    value = value.strip()
    if not value:
        raise ValueError("empty timestamp")
    timestamp = float(value)
    # SubT ground_truth_path.csv stores timestamps in nanoseconds.
    if timestamp > 1e12:
        timestamp *= 1e-9
    return timestamp


def convert(input_path: Path, output_path: Path, time_col: str,
            pos_cols: tuple[str, str, str], quat_cols: tuple[str, str, str, str]) -> int:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    count = 0

    with input_path.open(newline="") as f_in, output_path.open("w", newline="") as f_out:
        reader = csv.DictReader(f_in)
        for row in reader:
            try:
                timestamp = timestamp_to_sec(row[time_col])
                px, py, pz = (float(row[col]) for col in pos_cols)
                qx, qy, qz, qw = (float(row[col]) for col in quat_cols)
            except (KeyError, ValueError):
                continue

            f_out.write(
                f"{timestamp:.9f} "
                f"{px:.9f} {py:.9f} {pz:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )
            count += 1

    return count


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "kind",
        choices=("adaptive", "fastlio_external", "subt_gt"),
        help="input CSV schema",
    )
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_tum", type=Path)
    args = parser.parse_args()

    if args.kind == "adaptive":
        count = convert(
            args.input_csv,
            args.output_tum,
            "lidar_end_time",
            ("pos_x", "pos_y", "pos_z"),
            ("quat_x", "quat_y", "quat_z", "quat_w"),
        )
    elif args.kind == "fastlio_external":
        count = convert(
            args.input_csv,
            args.output_tum,
            "stamp",
            ("pos_x", "pos_y", "pos_z"),
            ("quat_x", "quat_y", "quat_z", "quat_w"),
        )
    else:
        count = convert(
            args.input_csv,
            args.output_tum,
            "timestamp",
            ("p_w_b_x", "p_w_b_y", "p_w_b_z"),
            ("q_w_b_x", "q_w_b_y", "q_w_b_z", "q_w_b_w"),
        )

    print(f"Wrote {count} poses -> {args.output_tum}")


if __name__ == "__main__":
    main()
