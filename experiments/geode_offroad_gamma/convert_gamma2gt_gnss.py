#!/usr/bin/env python3
"""Convert GEODE Gamma (Livox AVIA) TUM trajectories to the GNSS/Beta frame.

This is a command-line adaptation of GEODE's official ``gamma2GT_gnss.py``.
It preserves the official Gamma/Carol extrinsic and transformation order:
``T_eval = T_device @ inverse(T_gamma_to_beta)``.
"""

import argparse
from pathlib import Path

import numpy as np


def quat_to_matrix(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy - 2 * qz * qw,
         2 * qx * qz + 2 * qy * qw],
        [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx * qx - 2 * qz * qz,
         2 * qy * qz - 2 * qx * qw],
        [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw,
         1 - 2 * qx * qx - 2 * qy * qy],
    ])


def matrix_to_quat(matrix):
    """Return x, y, z, w, matching TUM and pyquaternion output ordering."""
    trace = np.trace(matrix)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        qw = 0.25 * s
        qx = (matrix[2, 1] - matrix[1, 2]) / s
        qy = (matrix[0, 2] - matrix[2, 0]) / s
        qz = (matrix[1, 0] - matrix[0, 1]) / s
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            qw = (matrix[2, 1] - matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (matrix[0, 1] + matrix[1, 0]) / s
            qz = (matrix[0, 2] + matrix[2, 0]) / s
        elif index == 1:
            s = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            qw = (matrix[0, 2] - matrix[2, 0]) / s
            qx = (matrix[0, 1] + matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            qw = (matrix[1, 0] - matrix[0, 1]) / s
            qx = (matrix[0, 2] + matrix[2, 0]) / s
            qy = (matrix[1, 2] + matrix[2, 1]) / s
            qz = 0.25 * s
    norm = np.linalg.norm([qx, qy, qz, qw])
    return qx / norm, qy / norm, qz / norm, qw / norm


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Gamma trajectory in TUM format")
    parser.add_argument("output", type=Path, help="Output TUM trajectory in GNSS/Beta frame")
    args = parser.parse_args()

    # Official gamma2GT_gnss.py: Gamma/Carol -> Beta/GNSS rigid extrinsic.
    qw, qx, qy, qz = 0.9998828, -0.0057758, 0.0022253, 0.0140019
    translation = np.array([0.0305, -0.5959, 0.0902])
    gamma_to_beta = np.eye(4)
    gamma_to_beta[:3, :3] = quat_to_matrix(qx, qy, qz, qw)
    gamma_to_beta[:3, 3] = translation
    beta_from_gamma = np.linalg.inv(gamma_to_beta)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    converted = 0
    with args.input.open() as source, args.output.open("w") as destination:
        for line in source:
            fields = line.split()
            if not fields or fields[0].startswith("#"):
                continue
            if len(fields) != 8:
                raise ValueError(f"Expected 8 TUM fields, got {len(fields)}: {line.strip()}")
            stamp, x, y, z, qx, qy, qz, qw = map(float, fields)
            device_pose = np.eye(4)
            device_pose[:3, :3] = quat_to_matrix(qx, qy, qz, qw)
            device_pose[:3, 3] = [x, y, z]
            evaluation_pose = device_pose @ beta_from_gamma
            ex, ey, ez = evaluation_pose[:3, 3]
            eqx, eqy, eqz, eqw = matrix_to_quat(evaluation_pose[:3, :3])
            destination.write(
                f"{stamp:.9f} {ex:.9f} {ey:.9f} {ez:.9f} "
                f"{eqx:.9f} {eqy:.9f} {eqz:.9f} {eqw:.9f}\n"
            )
            converted += 1
    print(f"Converted {converted} poses: {args.input} -> {args.output}")


if __name__ == "__main__":
    main()
