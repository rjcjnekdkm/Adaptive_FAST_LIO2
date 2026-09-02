#!/usr/bin/env python3
"""Evaluate a FAST-LIO-style trajectory using the official NTU VIRAL protocol.

The implementation follows ntu-aris/viral_eval/evaluate_one.m:
1. compensate the estimated IMU/body position to the Leica prism position;
2. linearly interpolate Leica positions at estimate timestamps when adjacent
   ground-truth samples are less than 0.1 s apart;
3. apply a rigid SE(3), no-scale least-squares alignment; and
4. report the official position ATE: norm(rms(axis-wise position error)).

FAST-LIO's published odometry quaternion represents R_W_B, so the body-frame
prism lever arm is rotated directly by that quaternion.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


PRISM_IN_BODY = np.array([-0.293656, -0.012288, -0.273095], dtype=float)


def read_gt(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float,
                         encoding="utf-8")
    if data.size == 0:
        raise ValueError(f"empty ground truth: {path}")
    data = np.atleast_1d(data)
    times = data["time"]
    positions = np.column_stack((
        data["fieldposepositionx"], data["fieldposepositiony"],
        data["fieldposepositionz"],
    ))

    # Equivalent to the union-of-unique-axis indices in viral_eval.
    indices = np.unique(np.concatenate([
        np.unique(positions[:, axis], return_index=True)[1]
        for axis in range(3)
    ]))
    return (times[indices] - times[0]) * 1e-9, positions[indices]


def quat_to_rotmat(q_xyzw: np.ndarray) -> np.ndarray:
    q = q_xyzw / np.linalg.norm(q_xyzw)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def read_estimate(path: Path, gt_time_zero_ns: float) -> tuple[np.ndarray, np.ndarray]:
    if path.suffix == ".tum":
        return read_tum_estimate(path, gt_time_zero_ns)
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"empty estimate: {path}")

    time_key = "stamp" if "stamp" in rows[0] else "lidar_end_time"
    times, prism_positions = [], []
    for row in rows:
        try:
            stamp = float(row[time_key])
            pos = np.array([float(row[f"pos_{axis}"]) for axis in "xyz"])
            quat = np.array([float(row[f"quat_{axis}"]) for axis in "xyzw"])
        except (KeyError, ValueError):
            continue
        if np.linalg.norm(quat) < 1e-12:
            continue
        # FAST-LIO publishes T_W_B. Leica observes the prism rigidly mounted
        # in B, so p_W_prism = p_W_B + R_W_B * p_B_prism.
        prism_positions.append(pos + quat_to_rotmat(quat) @ PRISM_IN_BODY)
        times.append(stamp - gt_time_zero_ns * 1e-9)
    return np.asarray(times), np.asarray(prism_positions)


def read_tum_estimate(path: Path, gt_time_zero_ns: float) -> tuple[np.ndarray, np.ndarray]:
    times, prism_positions = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) != 8 or fields[0].startswith("#"):
            continue
        values = np.asarray(fields, dtype=float)
        stamp, pos, quat = values[0], values[1:4], values[4:8]
        if np.linalg.norm(quat) < 1e-12:
            continue
        prism_positions.append(pos + quat_to_rotmat(quat) @ PRISM_IN_BODY)
        times.append(stamp - gt_time_zero_ns * 1e-9)
    if not times:
        raise ValueError(f"no valid TUM poses: {path}")
    return np.asarray(times), np.asarray(prism_positions)


def interpolate_gt(gt_time: np.ndarray, gt_pos: np.ndarray,
                   est_time: np.ndarray, est_pos: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    indices = np.searchsorted(gt_time, est_time, side="right") - 1
    valid = (indices >= 0) & (indices < len(gt_time) - 1)
    valid &= (gt_time[np.clip(indices + 1, 0, len(gt_time) - 1)] -
              gt_time[np.clip(indices, 0, len(gt_time) - 1)] < 0.1)
    indices = indices[valid]
    est_time, est_pos = est_time[valid], est_pos[valid]
    alpha = ((est_time - gt_time[indices]) /
             (gt_time[indices + 1] - gt_time[indices]))[:, None]
    return est_time, est_pos, gt_pos[indices] + alpha * (gt_pos[indices + 1] - gt_pos[indices])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("ground_truth_csv", type=Path)
    parser.add_argument("estimate_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()

    gt_time, gt_pos = read_gt(args.ground_truth_csv)
    gt_time_zero_ns = float(np.genfromtxt(args.ground_truth_csv, delimiter=",", names=True,
                                          max_rows=1, dtype=float, encoding="utf-8")["time"])
    est_time, est_prism = read_estimate(args.estimate_csv, gt_time_zero_ns)
    time, est_prism, gt_interp = interpolate_gt(gt_time, gt_pos, est_time, est_prism)
    if len(time) < 3:
        raise RuntimeError("fewer than three time-associated trajectory samples")

    ref_mean, est_mean = gt_interp.mean(axis=0), est_prism.mean(axis=0)
    covariance = (gt_interp - ref_mean).T @ (est_prism - est_mean) / len(gt_interp)
    u, _, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[2, 2] = -1.0
    rotation = u @ correction @ vt
    translation = ref_mean - rotation @ est_mean
    aligned = (rotation @ est_prism.T).T + translation
    error = gt_interp - aligned
    norms = np.linalg.norm(error, axis=1)
    ate = float(np.sqrt(np.mean(np.sum(error * error, axis=1))))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    np.savetxt(
        args.output_dir / "ntu_aligned_trajectory.csv",
        np.column_stack((time, gt_interp, aligned, error, norms)),
        delimiter=",",
        header="time_s,gt_x,gt_y,gt_z,est_prism_x,est_prism_y,est_prism_z,err_x,err_y,err_z,err_norm",
        comments="",
    )
    metrics = {
        "protocol": "NTU VIRAL official: prism compensation + <0.1s interpolation + SE(3) alignment",
        "associated_samples": int(len(time)),
        "ate_rmse_m": ate,
        "mean_position_error_m": float(norms.mean()),
        "median_position_error_m": float(np.median(norms)),
        "max_position_error_m": float(norms.max()),
        "prism_in_body_m": PRISM_IN_BODY.tolist(),
        "rotation_estimate_to_leica": rotation.tolist(),
        "translation_estimate_to_leica_m": translation.tolist(),
    }
    (args.output_dir / "ntu_ate_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
