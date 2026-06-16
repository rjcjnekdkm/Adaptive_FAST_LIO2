"""使用 Corridor02 提供的最终相对位姿真值评估终点误差。

GTCorridor02.txt 只包含一个 3x3 旋转矩阵、一个 3x1 平移向量和 bag 时长，
因此本脚本计算的是最终相对位姿误差，不是整条轨迹的 ATE/RPE。
"""

from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path("/home/romi/ros2_ws/experiments/m3dgr_corridor02")
GT_FILE = ROOT / "groud_truth/GTCorridor02.txt"
RESULT_DIR = ROOT / "results"
OUTPUT_FILE = ROOT / "analysis/endpoint_ground_truth_evaluation.csv"


def load_ground_truth(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """读取文件中的前三行旋转矩阵和后三行平移向量。"""
    numeric_lines = []
    for line in path.read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if not text or text.startswith("bag_time"):
            continue
        numeric_lines.append([float(value) for value in text.split()])

    if len(numeric_lines) != 6 or any(len(row) != 3 for row in numeric_lines[:3]):
        raise ValueError(f"Unsupported ground-truth format: {path}")

    rotation = np.asarray(numeric_lines[:3], dtype=float)
    translation = np.asarray([row[0] for row in numeric_lines[3:]], dtype=float)
    return rotation, translation


def quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    """将按 x、y、z、w 排列的单位四元数转换为旋转矩阵。"""
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_error_degrees(estimated: np.ndarray, ground_truth: np.ndarray) -> float:
    """计算两个旋转矩阵之间的夹角，单位为度。"""
    relative = ground_truth.T @ estimated
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def evaluate(csv_path: Path, gt_rotation: np.ndarray, gt_translation: np.ndarray) -> dict:
    """计算起点到终点的相对平移误差，并在 CSV 有姿态时计算旋转误差。"""
    data = pd.read_csv(csv_path)
    position_columns = ["pos_x", "pos_y", "pos_z"]
    estimated_translation = (
        data[position_columns].iloc[-1].to_numpy()
        - data[position_columns].iloc[0].to_numpy()
    )

    result = {
        "experiment": csv_path.stem,
        "frames": len(data),
        "estimated_tx": estimated_translation[0],
        "estimated_ty": estimated_translation[1],
        "estimated_tz": estimated_translation[2],
        "gt_tx": gt_translation[0],
        "gt_ty": gt_translation[1],
        "gt_tz": gt_translation[2],
        "translation_endpoint_error_m": np.linalg.norm(estimated_translation - gt_translation),
        "rotation_endpoint_error_deg": np.nan,
    }

    quaternion_columns = ["quat_x", "quat_y", "quat_z", "quat_w"]
    if all(column in data.columns for column in quaternion_columns):
        start_rotation = quaternion_to_rotation(data[quaternion_columns].iloc[0].to_numpy())
        end_rotation = quaternion_to_rotation(data[quaternion_columns].iloc[-1].to_numpy())
        estimated_relative_rotation = start_rotation.T @ end_rotation
        result["rotation_endpoint_error_deg"] = rotation_error_degrees(
            estimated_relative_rotation,
            gt_rotation,
        )

    return result


def main():
    gt_rotation, gt_translation = load_ground_truth(GT_FILE)
    evaluations = [
        evaluate(RESULT_DIR / "adaptive_off_runtime.csv", gt_rotation, gt_translation),
        evaluate(RESULT_DIR / "adaptive_runtime.csv", gt_rotation, gt_translation),
    ]

    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    output = pd.DataFrame(evaluations)
    output.to_csv(OUTPUT_FILE, index=False)
    print(output.to_string(index=False))
    print(f"\nGround-truth translation norm: {np.linalg.norm(gt_translation):.6f} m")
    print("This is endpoint relative-pose evaluation, not ATE/RPE.")


if __name__ == "__main__":
    main()
