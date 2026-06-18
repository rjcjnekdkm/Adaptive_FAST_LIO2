#!/usr/bin/env python3
"""Plot aligned TUM trajectories for experiment comparison."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(path)
    return data[:, 0], data[:, 1:4]


def associate(
    ref_time: np.ndarray,
    est_time: np.ndarray,
    max_diff: float,
) -> tuple[np.ndarray, np.ndarray]:
    ref_indices = []
    est_indices = []
    for est_index, timestamp in enumerate(est_time):
        ref_index = int(np.searchsorted(ref_time, timestamp))
        candidates = []
        if ref_index < len(ref_time):
            candidates.append(ref_index)
        if ref_index > 0:
            candidates.append(ref_index - 1)
        if not candidates:
            continue
        best = min(candidates, key=lambda index: abs(ref_time[index] - timestamp))
        if abs(ref_time[best] - timestamp) <= max_diff:
            ref_indices.append(best)
            est_indices.append(est_index)
    return np.asarray(ref_indices), np.asarray(est_indices)


def rigid_align(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def align_to_reference(
    ref_time: np.ndarray,
    ref_xyz: np.ndarray,
    est_time: np.ndarray,
    est_xyz: np.ndarray,
    max_diff: float,
) -> np.ndarray:
    ref_indices, est_indices = associate(ref_time, est_time, max_diff)
    rotation, translation = rigid_align(
        est_xyz[est_indices],
        ref_xyz[ref_indices],
    )
    return (rotation @ est_xyz.T).T + translation


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument(
        "--trajectory",
        action="append",
        nargs=3,
        metavar=("LABEL", "COLOR", "TUM_FILE"),
        required=True,
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-diff", type=float, default=0.12)
    args = parser.parse_args()

    ref_time, ref_xyz = load_tum(args.reference)

    fig, ax = plt.subplots(figsize=(10, 6.2), constrained_layout=True)
    ax.plot(
        ref_xyz[:, 0],
        ref_xyz[:, 1],
        color="black",
        linewidth=2.4,
        label="Ground Truth",
        zorder=5,
    )

    for label, color, path_value in args.trajectory:
        est_time, est_xyz = load_tum(Path(path_value))
        aligned = align_to_reference(
            ref_time,
            ref_xyz,
            est_time,
            est_xyz,
            args.max_diff,
        )
        ax.plot(
            aligned[:, 0],
            aligned[:, 1],
            color=color,
            linewidth=1.7,
            label=label,
        )

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("SubT-MRS Hawkins Long Corridor: XY Trajectory Comparison")
    ax.axis("equal")
    ax.grid(True, color="#d0d0d0", linewidth=0.6, alpha=0.7)
    ax.legend(loc="best", framealpha=0.95)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=300)
    fig.savefig(args.output.with_suffix(".pdf"))
    print(f"Saved {args.output} and {args.output.with_suffix('.pdf')}")


if __name__ == "__main__":
    main()
