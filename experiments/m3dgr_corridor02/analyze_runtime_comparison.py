"""对比 Adaptive FAST-LIO2 开启/关闭 adaptive_map 时的运行统计。"""

import os
import warnings
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR",
    "/home/romi/ros2_ws/experiments/m3dgr_corridor02/analysis/.matplotlib",
)
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*", category=UserWarning)

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path("/home/romi/ros2_ws/experiments/m3dgr_corridor02")
RESULT_DIR = ROOT / "results"
ANALYSIS_DIR = ROOT / "analysis"

OFF_CSV = RESULT_DIR / "adaptive_off_runtime.csv"
ON_CSV = RESULT_DIR / "adaptive_runtime.csv"


def path_length(data: pd.DataFrame) -> float:
    """计算估计轨迹相邻位置之间的累计长度。"""
    positions = data[["pos_x", "pos_y", "pos_z"]].to_numpy()
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def endpoint_displacement(data: pd.DataFrame) -> float:
    """计算终点到起点的直线距离，仅作为闭环一致性参考，不等同于 ATE。"""
    positions = data[["pos_x", "pos_y", "pos_z"]].to_numpy()
    return float(np.linalg.norm(positions[-1] - positions[0]))


def summarize(name: str, data: pd.DataFrame) -> dict:
    """提取一组实验的核心统计指标。"""
    return {
        "experiment": name,
        "frames": len(data),
        "degenerate_frames": int(data["degenerate"].sum()),
        "degenerate_ratio": float(data["degenerate"].mean()),
        "residual_mean_avg": float(data["residual_mean"].mean()),
        "residual_mean_median": float(data["residual_mean"].median()),
        "effective_points_avg": float(data["effective_points"].mean()),
        "map_added_avg": float(data["map_added"].mean()),
        "insert_ratio_avg": float(data["insert_ratio"].mean()),
        "final_map_size": int(data["map_size"].iloc[-1]),
        "total_map_added": int(data["total_map_added"].iloc[-1]),
        "total_quality_rejected": int(data["total_quality_rejected"].iloc[-1]),
        "total_direction_rejected": int(data["total_direction_rejected"].iloc[-1]),
        "total_voxel_rejected": int(data["total_voxel_rejected"].iloc[-1]),
        "path_length": path_length(data),
        "endpoint_displacement": endpoint_displacement(data),
        "final_pos_x": float(data["pos_x"].iloc[-1]),
        "final_pos_y": float(data["pos_y"].iloc[-1]),
        "final_pos_z": float(data["pos_z"].iloc[-1]),
    }


def save_line_plot(off: pd.DataFrame, on: pd.DataFrame, column: str, ylabel: str, filename: str):
    """绘制两组实验的逐帧曲线，并使用滑动平均降低显示噪声。"""
    plt.figure(figsize=(10, 4.8))
    window = 30
    plt.plot(off["frame"], off[column].rolling(window, min_periods=1).mean(), label="Adaptive OFF")
    plt.plot(on["frame"], on[column].rolling(window, min_periods=1).mean(), label="Adaptive ON")
    plt.xlabel("Frame")
    plt.ylabel(ylabel)
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(ANALYSIS_DIR / filename, dpi=180)
    plt.close()


def main():
    ANALYSIS_DIR.mkdir(parents=True, exist_ok=True)

    off = pd.read_csv(OFF_CSV)
    on = pd.read_csv(ON_CSV)
    if len(off) != len(on):
        raise RuntimeError(f"ON/OFF frame count mismatch: {len(on)} != {len(off)}")

    summary = pd.DataFrame([
        summarize("Adaptive OFF", off),
        summarize("Adaptive ON", on),
    ])
    summary.to_csv(ANALYSIS_DIR / "runtime_summary.csv", index=False)

    off_summary = summary.iloc[0]
    on_summary = summary.iloc[1]
    report = [
        "M3DGR Corridor02 Adaptive Map Runtime Comparison",
        "",
        f"Frames: OFF={len(off)}, ON={len(on)}",
        f"ON degenerate frames: {int(on['degenerate'].sum())} "
        f"({on['degenerate'].mean() * 100:.2f}%)",
        f"Average residual_mean: OFF={off['residual_mean'].mean():.6f}, "
        f"ON={on['residual_mean'].mean():.6f}",
        f"Final map size: OFF={int(off['map_size'].iloc[-1])}, "
        f"ON={int(on['map_size'].iloc[-1])}",
        f"Total map added: OFF={int(off['total_map_added'].iloc[-1])}, "
        f"ON={int(on['total_map_added'].iloc[-1])}",
        f"ON total quality rejected: {int(on['total_quality_rejected'].iloc[-1])}",
        f"ON total direction rejected: {int(on['total_direction_rejected'].iloc[-1])}",
        f"Endpoint displacement: OFF={off_summary['endpoint_displacement']:.3f} m, "
        f"ON={on_summary['endpoint_displacement']:.3f} m",
        "",
        f"Residual mean change: {(on['residual_mean'].mean() / off['residual_mean'].mean() - 1) * 100:.2f}%",
        f"Effective points change: {(on['effective_points'].mean() / off['effective_points'].mean() - 1) * 100:.2f}%",
        f"Total map added change: {(on['total_map_added'].iloc[-1] / off['total_map_added'].iloc[-1] - 1) * 100:.2f}%",
        f"Final map size change: {(on['map_size'].iloc[-1] / off['map_size'].iloc[-1] - 1) * 100:.2f}%",
        f"Degenerate trigger counts: effective_points={(on['effective_points'] < 200).sum()}, "
        f"effective_ratio={(on['effective_ratio'] < 0.1).sum()}, "
        f"mean_residual={(on['residual_mean'] > 0.15).sum()}, "
        f"normal_ratio={(on['normal_eigen_ratio'] < 0.02).sum()}",
        "",
        "Note: endpoint displacement is only a loop-closure consistency reference, not ATE.",
        "OFF effective_ratio and normal_eigen_ratio are zero because degeneracy statistics",
        "are currently skipped when adaptive_map is disabled.",
    ]
    (ANALYSIS_DIR / "runtime_report.txt").write_text("\n".join(report) + "\n", encoding="utf-8")

    save_line_plot(off, on, "residual_mean", "Mean point-to-plane residual (m)", "residual_mean.png")
    save_line_plot(off, on, "effective_points", "Effective matched points", "effective_points.png")
    save_line_plot(off, on, "map_added", "Map points added per frame", "map_added.png")
    save_line_plot(off, on, "map_size", "ikd-tree map size", "map_size.png")

    # 绘制俯视轨迹，用于观察累计漂移和终点回到起点附近的程度。
    plt.figure(figsize=(7, 7))
    plt.plot(off["pos_x"], off["pos_y"], label="Adaptive OFF")
    plt.plot(on["pos_x"], on["pos_y"], label="Adaptive ON")
    plt.scatter([off["pos_x"].iloc[0]], [off["pos_y"].iloc[0]], marker="o", label="Start")
    plt.scatter([off["pos_x"].iloc[-1]], [off["pos_y"].iloc[-1]], marker="x", label="OFF end")
    plt.scatter([on["pos_x"].iloc[-1]], [on["pos_y"].iloc[-1]], marker="+", label="ON end")
    plt.axis("equal")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(ANALYSIS_DIR / "trajectory_xy.png", dpi=180)
    plt.close()

    print(summary.to_string(index=False))
    print(f"\nAnalysis outputs: {ANALYSIS_DIR}")


if __name__ == "__main__":
    main()
