#!/usr/bin/env python3
"""Reconstruct a full-rate backend trajectory from optimized keyframes.

The backend pose graph optimizes sparse keyframes, whereas the adaptive
frontend runtime CSV stores a pose for every LiDAR frame.  This tool transfers
the nearest optimized-keyframe correction to each frontend pose:

    Delta_k = T_opt,k * inverse(T_front,k)
    T_full_opt(t) = Delta_nearest(t) * T_front(t)

The output preserves the frontend CSV timestamps, so GEODE's official ATE
script evaluates the same temporal support for frontend and full-system runs.
"""

import argparse
import bisect
import csv
import math
from pathlib import Path


def quat_to_matrix(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("zero quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )


def matrix_to_quat(r):
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r[2][1] - r[1][2]) / scale
        qy = (r[0][2] - r[2][0]) / scale
        qz = (r[1][0] - r[0][1]) / scale
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        scale = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / scale
        qx = 0.25 * scale
        qy = (r[0][1] + r[1][0]) / scale
        qz = (r[0][2] + r[2][0]) / scale
    elif r[1][1] > r[2][2]:
        scale = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / scale
        qx = (r[0][1] + r[1][0]) / scale
        qy = 0.25 * scale
        qz = (r[1][2] + r[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / scale
        qx = (r[0][2] + r[2][0]) / scale
        qy = (r[1][2] + r[2][1]) / scale
        qz = 0.25 * scale
    return qx, qy, qz, qw


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def matvec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def transpose(r):
    return tuple(zip(*r))


def nearest_index(times, stamp):
    index = bisect.bisect_left(times, stamp)
    if index == 0:
        return 0
    if index == len(times):
        return len(times) - 1
    return index if abs(times[index] - stamp) < abs(stamp - times[index - 1]) else index - 1


def load_frontend(csv_path):
    samples = []
    with csv_path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            samples.append((
                float(row["lidar_begin_time"]),
                (float(row["pos_x"]), float(row["pos_y"]), float(row["pos_z"])),
                quat_to_matrix(float(row["quat_x"]), float(row["quat_y"]),
                               float(row["quat_z"]), float(row["quat_w"])),
            ))
    return samples


def load_keyframes(tum_path):
    keyframes = []
    for line in tum_path.read_text().splitlines():
        fields = line.split()
        if len(fields) != 8:
            continue
        stamp, x, y, z, qx, qy, qz, qw = map(float, fields)
        keyframes.append((stamp, (x, y, z), quat_to_matrix(qx, qy, qz, qw)))
    return keyframes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runtime_csv", type=Path)
    parser.add_argument("optimized_keyframes_tum", type=Path)
    parser.add_argument("output_tum", type=Path)
    parser.add_argument("--keyframe-match-tolerance", type=float, default=0.2)
    args = parser.parse_args()

    frontend = load_frontend(args.runtime_csv)
    keyframes = load_keyframes(args.optimized_keyframes_tum)
    if not frontend or not keyframes:
        raise RuntimeError("frontend runtime CSV or optimized keyframe trajectory is empty")

    frontend_times = [sample[0] for sample in frontend]
    corrections = []
    for stamp, opt_t, opt_r in keyframes:
        index = nearest_index(frontend_times, stamp)
        front_stamp, front_t, front_r = frontend[index]
        if abs(front_stamp - stamp) > args.keyframe_match_tolerance:
            continue
        corr_r = matmul(opt_r, transpose(front_r))
        rotated_front_t = matvec(corr_r, front_t)
        corr_t = tuple(opt_t[i] - rotated_front_t[i] for i in range(3))
        corrections.append((stamp, corr_t, corr_r))

    if not corrections:
        raise RuntimeError("no optimized keyframe can be matched to frontend runtime CSV")
    correction_times = [correction[0] for correction in corrections]
    with args.output_tum.open("w") as output:
        for stamp, front_t, front_r in frontend:
            correction = corrections[nearest_index(correction_times, stamp)]
            _, corr_t, corr_r = correction
            full_r = matmul(corr_r, front_r)
            rotated_t = matvec(corr_r, front_t)
            full_t = tuple(rotated_t[i] + corr_t[i] for i in range(3))
            qx, qy, qz, qw = matrix_to_quat(full_r)
            output.write(
                f"{stamp:.9f} {full_t[0]:.9f} {full_t[1]:.9f} {full_t[2]:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )
    print(f"frontend frames: {len(frontend)}")
    print(f"optimized keyframes: {len(keyframes)}")
    print(f"matched correction anchors: {len(corrections)}")
    print(f"wrote: {args.output_tum}")


if __name__ == "__main__":
    main()
