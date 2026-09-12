#!/usr/bin/env python3
"""Export Adaptive FAST-LIO2 runtime CSV poses to a TUM trajectory."""

import argparse
import csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("tum", type=Path)
    args = parser.parse_args()

    with args.csv.open(newline="") as source, args.tum.open("w") as destination:
        for row in csv.DictReader(source):
            # The frontend state is timestamped at the end of the LiDAR scan.
            destination.write(
                f"{float(row['lidar_end_time']):.9f} "
                f"{float(row['pos_x']):.9f} {float(row['pos_y']):.9f} "
                f"{float(row['pos_z']):.9f} {float(row['quat_x']):.9f} "
                f"{float(row['quat_y']):.9f} {float(row['quat_z']):.9f} "
                f"{float(row['quat_w']):.9f}\n"
            )


if __name__ == "__main__":
    main()
