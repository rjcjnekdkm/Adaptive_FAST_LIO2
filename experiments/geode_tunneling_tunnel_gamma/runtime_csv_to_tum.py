#!/usr/bin/env python3
"""Export a FAST-LIO runtime recorder CSV to TUM trajectory format."""

import argparse
import csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("tum", type=Path)
    args = parser.parse_args()

    with args.csv.open(newline="") as fin, args.tum.open("w") as fout:
        reader = csv.DictReader(fin)
        # FAST-LIO baseline recorder uses ``stamp``; the adaptive front-end
        # records the equivalent scan timestamp as ``lidar_begin_time``.
        time_key = "stamp" if "stamp" in (reader.fieldnames or []) else "lidar_begin_time"
        if time_key not in (reader.fieldnames or []):
            raise KeyError("CSV requires either 'stamp' or 'lidar_begin_time'")
        for row in reader:
            fout.write(
                f"{float(row[time_key]):.9f} "
                f"{float(row['pos_x']):.9f} {float(row['pos_y']):.9f} {float(row['pos_z']):.9f} "
                f"{float(row['quat_x']):.9f} {float(row['quat_y']):.9f} "
                f"{float(row['quat_z']):.9f} {float(row['quat_w']):.9f}\n"
            )


if __name__ == "__main__":
    main()
