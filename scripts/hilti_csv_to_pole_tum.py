#!/usr/bin/env python3
"""Export FAST-LIO runtime CSV poses to HILTI 2021 ``pole`` TUM poses.

Both FAST-LIO2's runtime recorder and Adaptive FAST-LIO2's runtime logger
record the global IMU/body pose.  HILTI Basement_1 ground truth is measured at
the total-station pole tip.  This script applies the fixed calibrated transform
``T_imu_pole_tip`` to each estimated IMU pose before evaluation.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


# HILTI calibration.yaml, quaternions use (x, y, z, w).  Each listed transform
# follows p_parent = R_parent_child * p_child + t_parent_child.
T_IMU_CAM0 = (
    (0.05067834857850693, 0.0458784339890185, -0.005943648304780761),
    (-0.5003218001035493, 0.5012125349997221, -0.5001966939080825, 0.49826434600894337),
)
T_CAM0_MARKER = (
    (0.05354253273380533, -0.2786560903711294, -0.057329537886130204),
    (-0.702423432982067, -0.06685205948137603, -0.06048368390116566, 0.706026803260705),
)
T_MARKER_POLE = ((-0.0043, 0.0010, -1.3943), (0.0, 0.0, 0.0, 1.0))


def normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = sum(value * value for value in q) ** 0.5
    return tuple(value / norm for value in q)  # type: ignore[return-value]


def multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Quaternion product a*b, with both quaternions in xyzw ordering."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate vector v by xyzw quaternion q."""
    x, y, z, w = normalize(q)
    vx, vy, vz = v
    # Equivalent to q * [v, 0] * q^-1, expanded to avoid a temporary object.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compose(first, second):
    """Compose parent<-middle and middle<-child into parent<-child."""
    t1, q1 = first
    t2, q2 = second
    rt2 = rotate(q1, t2)
    return (
        (t1[0] + rt2[0], t1[1] + rt2[1], t1[2] + rt2[2]),
        multiply(q1, q2),
    )


def timestamp_column(row: dict[str, str]) -> str:
    if "stamp" in row:
        return "stamp"
    if "lidar_end_time" in row:
        return "lidar_end_time"
    raise KeyError("CSV has neither stamp nor lidar_end_time")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_tum", type=Path)
    args = parser.parse_args()

    t_imu_pole, q_imu_pole = compose(compose(T_IMU_CAM0, T_CAM0_MARKER), T_MARKER_POLE)
    args.output_tum.parent.mkdir(parents=True, exist_ok=True)

    count = 0
    with args.input_csv.open(newline="") as src, args.output_tum.open("w") as dst:
        reader = csv.DictReader(src)
        for row in reader:
            stamp = float(row[timestamp_column(row)])
            p_global_imu = (float(row["pos_x"]), float(row["pos_y"]), float(row["pos_z"]))
            q_global_imu = normalize((
                float(row["quat_x"]), float(row["quat_y"]),
                float(row["quat_z"]), float(row["quat_w"]),
            ))
            lever_arm = rotate(q_global_imu, t_imu_pole)
            p_global_pole = tuple(a + b for a, b in zip(p_global_imu, lever_arm))
            q_global_pole = multiply(q_global_imu, q_imu_pole)
            dst.write(
                f"{stamp:.9f} {p_global_pole[0]:.9f} {p_global_pole[1]:.9f} {p_global_pole[2]:.9f} "
                f"{q_global_pole[0]:.9f} {q_global_pole[1]:.9f} {q_global_pole[2]:.9f} {q_global_pole[3]:.9f}\n"
            )
            count += 1

    print(f"T_imu_pole_tip translation: {t_imu_pole}")
    print(f"Exported {count} poses -> {args.output_tum}")


if __name__ == "__main__":
    main()
