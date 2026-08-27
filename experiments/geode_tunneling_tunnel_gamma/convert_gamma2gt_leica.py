#!/usr/bin/env python3
"""Convert GEODE Gamma trajectories to the Leica evaluation frame.

This is a dependency-free, batch-friendly implementation of the transform in
the official ``gamma2GT_leica.py``.  It preserves the official Carol sensor
extrinsic and applies ``T_eval = T_device * inverse(T_imu_lidar)`` to every
estimated pose.  Leica reference files contain zero quaternions because only
position is measured; they are copied with an identity quaternion so evo can
parse the TUM trajectory format.
"""

import argparse
import math
from pathlib import Path


# Official Gamma/Carol T_IMU_LiDAR used by gamma2GT_leica.py.
T_X, T_Y, T_Z = 0.00947221, -0.308202, -0.365733
T_QW, T_QX, T_QY, T_QZ = 0.999901, -0.00492765, 0.00575961, 0.0117651


def quat_to_matrix(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        raise ValueError("zero quaternion in estimated trajectory")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def matvec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def matrix_to_quat(r):
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def convert_estimate(src, dst):
    r_ext = quat_to_matrix(T_QX, T_QY, T_QZ, T_QW)
    with src.open() as fin, dst.open("w") as fout:
        for line in fin:
            fields = line.split()
            if len(fields) != 8:
                continue
            stamp, x, y, z, qx, qy, qz, qw = map(float, fields)
            r_device = quat_to_matrix(qx, qy, qz, qw)
            r_eval = matmul(r_device, tuple(zip(*r_ext)))
            dx, dy, dz = matvec(r_eval, (T_X, T_Y, T_Z))
            qx_e, qy_e, qz_e, qw_e = matrix_to_quat(r_eval)
            fout.write(f"{stamp:.9f} {x - dx:.9f} {y - dy:.9f} {z - dz:.9f} "
                       f"{qx_e:.9f} {qy_e:.9f} {qz_e:.9f} {qw_e:.9f}\n")


def normalize_gt(src, dst):
    with src.open() as fin, dst.open("w") as fout:
        for line in fin:
            fields = line.split()
            if len(fields) < 4:
                continue
            fout.write(f"{fields[0]} {fields[1]} {fields[2]} {fields[3]} 0 0 0 1\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True,
                        help="experiments/geode_tunneling_tunnel_gamma directory")
    parser.add_argument("--gt-dir", type=Path, required=True,
                        help="directory containing Tunneling_tunnel{1..5}.txt")
    parser.add_argument("--estimate-name", default="adaptive_backend_optimized.tum",
                        help="input TUM filename inside each results directory")
    parser.add_argument("--converted-name", default="adaptive_backend_leica.tum",
                        help="output TUM filename inside each results directory")
    parser.add_argument("--sequence", type=int, choices=range(1, 6), nargs="*",
                        default=range(1, 6))
    args = parser.parse_args()

    for index in args.sequence:
        result_dir = args.root / f"tunnel{index}" / "results"
        estimate = result_dir / args.estimate_name
        gt = args.gt_dir / f"Tunneling_tunnel{index}.txt"
        if not estimate.exists() or not gt.exists():
            raise FileNotFoundError(f"Tunnel{index}: estimate or GT missing")
        convert_estimate(estimate, result_dir / args.converted_name)
        normalize_gt(gt, result_dir / "leica_ground_truth.tum")
        print(f"Tunnel{index}: converted estimate and normalized Leica GT")


if __name__ == "__main__":
    main()
