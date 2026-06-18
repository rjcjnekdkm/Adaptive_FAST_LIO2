#!/usr/bin/env python3
"""Build a corrected SubT_MRS PointCloud2 bag.

The online velodyne packet -> PointCloud2 conversion produced valid point
cloud messages, but rosbag2 recorded wall-clock timestamps instead of message
header timestamps and missed a few early IMU messages. This script rebuilds the
final bag from:

* the full IMU stream in the raw ROS 2 bag, and
* the converted /velodyne_points stream.

It writes each record using msg.header.stamp as the rosbag2 timestamp, so
ros2 bag play --clock publishes the same time base that the SLAM nodes see in
message headers.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, SequentialWriter
from rosbag2_py import StorageOptions, TopicMetadata
from sensor_msgs.msg import Imu, PointCloud2


def header_stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def open_reader(uri: Path) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(uri), storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    return reader


def create_writer(uri: Path) -> SequentialWriter:
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(uri), storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    writer.create_topic(
        TopicMetadata(
            name="/imu/data",
            type="sensor_msgs/msg/Imu",
            serialization_format="cdr",
        )
    )
    writer.create_topic(
        TopicMetadata(
            name="/velodyne_points",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )
    )
    return writer


def write_imu(raw_bag: Path, writer: SequentialWriter) -> int:
    reader = open_reader(raw_bag)
    count = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/imu/data":
            continue
        msg = deserialize_message(data, Imu)
        writer.write(topic, data, header_stamp_ns(msg))
        count += 1
    return count


def write_points(points_bag: Path, writer: SequentialWriter) -> int:
    reader = open_reader(points_bag)
    count = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/velodyne_points":
            continue
        msg = deserialize_message(data, PointCloud2)
        writer.write(topic, data, header_stamp_ns(msg))
        count += 1
    return count


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--raw-ros2-bag",
        type=Path,
        default=Path("bag/SubT_MRS/SubT_ros2"),
        help="ROS 2 bag converted directly from the ROS 1 bag.",
    )
    parser.add_argument(
        "--points-bag",
        type=Path,
        default=Path("bag/SubT_MRS/SubT_points_ros2"),
        help="Bag containing converted /velodyne_points.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("bag/SubT_MRS/SubT_points_ros2_fixed"),
        help="Output bag path. Must not already exist.",
    )
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit(f"output already exists, refusing to overwrite: {args.output}")

    writer = create_writer(args.output)
    imu_count = write_imu(args.raw_ros2_bag, writer)
    point_count = write_points(args.points_bag, writer)
    print(
        f"wrote {args.output}: /imu/data={imu_count}, "
        f"/velodyne_points={point_count}"
    )


if __name__ == "__main__":
    main()
