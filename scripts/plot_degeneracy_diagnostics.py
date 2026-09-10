#!/usr/bin/env python3
"""Plot Adaptive Map localizability and map-update diagnostics."""

import argparse
import csv
import json
import math
import os
import statistics
from pathlib import Path

# WSL/沙箱环境中的 ~/.config 可能不可写；仅将 Matplotlib 缓存放到临时目录。
os.environ.setdefault("MPLCONFIGDIR", "/tmp/adaptive_fastlio_matplotlib")

import matplotlib.pyplot as plt


REQUIRED_COLUMNS = (
    "lidar_end_time",
    "degeneracy_mode",
    "normal_eigen_ratio",
    "localizability_observed_voxels",
    "localizability_f0",
    "localizability_lambda0",
    "translation_cov_eigen_max",
    "rotation_cov_eigen_max",
    "effective_ratio",
    "residual_mean",
    "insert_ratio",
)

def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot localizability and posterior-covariance diagnostics."
    )
    parser.add_argument("runtime_csv", type=Path, help="adaptive_runtime.csv")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path (default: <csv stem>_degeneracy_diagnostics.png)",
    )
    parser.add_argument(
        "--summary",
        type=Path,
        default=None,
        help="Output JSON path (default: <csv stem>_degeneracy_summary.json)",
    )
    parser.add_argument(
        "--median-window",
        type=int,
        default=20,
        help="Trailing median window in frames (default: 20)",
    )
    return parser.parse_args()


def load_rows(path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = [name for name in REQUIRED_COLUMNS if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(
                "CSV does not contain stage-1 columns: " + ", ".join(missing)
            )
        rows = list(reader)
    if not rows:
        raise ValueError("CSV contains no data rows")
    return rows


def float_column(rows, name):
    values = []
    for row in rows:
        try:
            value = float(row[name])
        except (KeyError, TypeError, ValueError):
            value = math.nan
        values.append(value)
    return values


def optional_float_column(rows, name, default=math.nan):
    if not rows or name not in rows[0]:
        return [default] * len(rows)
    return float_column(rows, name)


def trailing_median(values, window):
    window = max(1, window)
    result = []
    for index in range(len(values)):
        start = max(0, index - window + 1)
        finite = [value for value in values[start : index + 1] if math.isfinite(value)]
        result.append(statistics.median(finite) if finite else math.nan)
    return result


def finite_summary(values):
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return {"count": 0, "min": None, "median": None, "max": None}

    def percentile(fraction):
        index = int(round(fraction * (len(finite) - 1)))
        return finite[index]

    return {
        "count": len(finite),
        "min": finite[0],
        "p10": percentile(0.10),
        "median": statistics.median(finite),
        "p90": percentile(0.90),
        "max": finite[-1],
    }


def shade_modes(axis, times, modes):
    colors = {1: "#f6c85f", 2: "#e45756"}
    start = 0
    while start < len(modes):
        mode = modes[start]
        end = start + 1
        while end < len(modes) and modes[end] == mode:
            end += 1
        if mode in colors:
            left = times[start]
            right = times[end - 1]
            axis.axvspan(left, right, color=colors[mode], alpha=0.13, linewidth=0)
        start = end


def main():
    args = parse_args()
    rows = load_rows(args.runtime_csv)

    timestamps = float_column(rows, "lidar_end_time")
    first_time = next(value for value in timestamps if math.isfinite(value))
    times = [value - first_time for value in timestamps]
    modes = [int(round(value)) if math.isfinite(value) else 0 for value in float_column(rows, "degeneracy_mode")]

    f0 = float_column(rows, "localizability_f0")
    lambda0 = float_column(rows, "localizability_lambda0")
    normal_ratio = float_column(rows, "normal_eigen_ratio")
    observed_voxels = float_column(rows, "localizability_observed_voxels")
    translation_cov = float_column(rows, "translation_cov_eigen_max")
    rotation_cov = float_column(rows, "rotation_cov_eigen_max")
    effective_ratio = float_column(rows, "effective_ratio")
    residual_mean = float_column(rows, "residual_mean")
    insert_ratio = float_column(rows, "insert_ratio")
    f0_ratio = optional_float_column(rows, "localizability_f0_ratio")
    lambda0_ratio = optional_float_column(rows, "localizability_lambda0_ratio")

    median_window = max(1, args.median_window)
    f0_median = trailing_median(f0, median_window)
    lambda0_median = trailing_median(lambda0, median_window)

    output = args.output or args.runtime_csv.with_name(
        args.runtime_csv.stem + "_degeneracy_diagnostics.png"
    )
    summary_path = args.summary or args.runtime_csv.with_name(
        args.runtime_csv.stem + "_degeneracy_summary.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    figure, axes = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
    for axis in axes:
        shade_modes(axis, times, modes)
        axis.grid(True, alpha=0.3)

    axes[0].plot(times, f0, color="#4c78a8", alpha=0.35, linewidth=0.8, label="f0 raw")
    axes[0].plot(times, f0_median, color="#1f4e79", linewidth=1.6, label=f"f0 median ({median_window})")
    axes[0].plot(times, normal_ratio, color="#59a14f", alpha=0.7, linewidth=1.0, label="legacy normal ratio")
    axes[0].set_ylabel("anisotropy ratio")
    axes[0].legend(loc="upper right")

    axes[1].plot(times, lambda0, color="#b279a2", alpha=0.35, linewidth=0.8, label="lambda0 raw")
    axes[1].plot(times, lambda0_median, color="#7b2c70", linewidth=1.6, label=f"lambda0 median ({median_window})")
    if any(math.isfinite(value) for value in f0_ratio):
        ratio_axis = axes[1].twinx()
        ratio_axis.plot(times, f0_ratio, color="#4c78a8", alpha=0.8, linewidth=1.0, label="f0/reference")
        ratio_axis.plot(times, lambda0_ratio, color="#e15759", alpha=0.8, linewidth=1.0, label="lambda0/reference")
        ratio_axis.axhline(0.65, color="#555555", linestyle="--", linewidth=0.8, label="default drop threshold")
        ratio_axis.set_ylabel("relative localizability")
        ratio_axis.legend(loc="upper right")
    axes[1].set_ylabel("weak-axis mass / voxel")
    axes[1].legend(loc="upper left")

    axes[2].semilogy(times, [max(value, 1e-18) for value in translation_cov], color="#e15759", label="translation max covariance (m²)")
    axes[2].semilogy(times, [max(value, 1e-18) for value in rotation_cov], color="#76b7b2", label="rotation max covariance (rad²)")
    axes[2].set_ylabel("posterior covariance")
    axes[2].legend(loc="upper right")

    frontend_axis = axes[3]
    frontend_axis.plot(times, effective_ratio, color="#4c78a8", label="effective ratio")
    frontend_axis.plot(times, insert_ratio, color="#59a14f", label="insert ratio")
    frontend_axis.plot(times, residual_mean, color="#f28e2b", label="mean residual (m)")
    frontend_axis.set_ylabel("frontend diagnostics")
    frontend_axis.set_xlabel("time from first logged frame (s)")
    frontend_axis.legend(loc="upper right")

    figure.suptitle(
        "Adaptive Map degeneracy diagnostics\n"
        "yellow=Transient, red=Persistent"
    )
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(output, dpi=180)
    plt.close(figure)

    mode_counts = {str(mode): modes.count(mode) for mode in sorted(set(modes))}
    summary = {
        "source_csv": str(args.runtime_csv.resolve()),
        "frames": len(rows),
        "duration_seconds": max(times) - min(times),
        "median_window_frames": median_window,
        "mode_counts": mode_counts,
        "localizability_f0": finite_summary(f0),
        "localizability_lambda0": finite_summary(lambda0),
        "localizability_f0_ratio": finite_summary(f0_ratio),
        "localizability_lambda0_ratio": finite_summary(lambda0_ratio),
        "localizability_observed_voxels": finite_summary(observed_voxels),
        "translation_cov_eigen_max": finite_summary(translation_cov),
        "rotation_cov_eigen_max": finite_summary(rotation_cov),
        "effective_ratio": finite_summary(effective_ratio),
        "residual_mean": finite_summary(residual_mean),
        "insert_ratio": finite_summary(insert_ratio),
    }
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    print(output.resolve())
    print(summary_path.resolve())


if __name__ == "__main__":
    main()
