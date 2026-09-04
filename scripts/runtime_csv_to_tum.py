#!/usr/bin/env python3
"""Export frontend poses from an Adaptive FAST-LIO2 runtime CSV to TUM."""

import argparse
import csv
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("runtime_csv", type=Path)
    parser.add_argument("output_tum", type=Path)
    args = parser.parse_args()

    required = (
        "lidar_begin_time", "pos_x", "pos_y", "pos_z",
        "quat_x", "quat_y", "quat_z", "quat_w",
    )
    args.output_tum.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with args.runtime_csv.open(newline="") as source, args.output_tum.open("w") as target:
        reader = csv.DictReader(source)
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in required):
            raise ValueError("runtime CSV does not contain the required pose columns")
        for row in reader:
            target.write(
                f"{float(row['lidar_begin_time']):.9f} "
                f"{float(row['pos_x']):.9f} {float(row['pos_y']):.9f} {float(row['pos_z']):.9f} "
                f"{float(row['quat_x']):.9f} {float(row['quat_y']):.9f} "
                f"{float(row['quat_z']):.9f} {float(row['quat_w']):.9f}\n"
            )
            count += 1
    print(f"wrote {count} poses: {args.output_tum}")


if __name__ == "__main__":
    main()
