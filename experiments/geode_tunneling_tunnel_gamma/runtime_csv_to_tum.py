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
        for row in csv.DictReader(fin):
            fout.write(
                f"{float(row['stamp']):.9f} "
                f"{float(row['pos_x']):.9f} {float(row['pos_y']):.9f} {float(row['pos_z']):.9f} "
                f"{float(row['quat_x']):.9f} {float(row['quat_y']):.9f} "
                f"{float(row['quat_z']):.9f} {float(row['quat_w']):.9f}\n"
            )


if __name__ == "__main__":
    main()
