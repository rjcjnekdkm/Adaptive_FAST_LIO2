#!/usr/bin/env python3
"""Convert HILTI 2021 Basement_1 ROS1 data for ROS2 FAST-LIO2.

The source bag records a ROS1 ``livox_ros_driver/CustomMsg``.  ROS2 FAST-LIO2
uses the field-compatible ``livox_ros_driver2/msg/CustomMsg`` but rosbag2
cannot play the ROS1 package type.  This tool reserializes only the LiDAR and
the calibrated Alphasense IMU stream to their ROS2 CDR definitions.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


LIVOX_POINT = """uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""

LIVOX_MESSAGE = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver2/msg/CustomPoint[] points
"""

LIVOX_SOURCE_TOPIC = "/livox/lidar"
IMU_SOURCE_TOPIC = "/alphasense/imu"
LIVOX_TARGET_TYPE = "livox_ros_driver2/msg/CustomMsg"
IMU_TARGET_TYPE = "sensor_msgs/msg/Imu"


def create_typestore():
    """Create a ROS2 Humble typestore containing Livox driver2 messages."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    typestore.register(
        get_types_from_msg(LIVOX_POINT, "livox_ros_driver2/msg/CustomPoint")
    )
    typestore.register(get_types_from_msg(LIVOX_MESSAGE, LIVOX_TARGET_TYPE))
    return typestore


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("src", type=Path, help="Input ROS1 .bag")
    parser.add_argument("dst", type=Path, help="Output ROS2 bag directory")
    args = parser.parse_args()

    if not args.src.is_file():
        raise SystemExit(f"Input bag does not exist: {args.src}")
    if args.dst.exists():
        raise SystemExit(f"Output already exists; remove it first: {args.dst}")

    typestore = create_typestore()
    with AnyReader([args.src]) as reader:
        source_connections = {
            connection.topic: connection
            for connection in reader.connections
            if connection.topic in {LIVOX_SOURCE_TOPIC, IMU_SOURCE_TOPIC}
        }
        missing = {LIVOX_SOURCE_TOPIC, IMU_SOURCE_TOPIC} - source_connections.keys()
        if missing:
            raise SystemExit(f"Missing required topics: {sorted(missing)}")

        try:
            with Writer(args.dst, version=9) as writer:
                livox_connection = writer.add_connection(
                    LIVOX_SOURCE_TOPIC, LIVOX_TARGET_TYPE, typestore=typestore
                )
                imu_connection = writer.add_connection(
                    IMU_SOURCE_TOPIC, IMU_TARGET_TYPE, typestore=typestore
                )
                output_connections = {
                    LIVOX_SOURCE_TOPIC: livox_connection,
                    IMU_SOURCE_TOPIC: imu_connection,
                }

                counters = {LIVOX_SOURCE_TOPIC: 0, IMU_SOURCE_TOPIC: 0}
                selected = list(source_connections.values())
                for connection, timestamp, rawdata in reader.messages(connections=selected):
                    message = reader.deserialize(rawdata, connection.msgtype)
                    if connection.topic == LIVOX_SOURCE_TOPIC:
                        data = typestore.serialize_cdr(message, LIVOX_TARGET_TYPE)
                    else:
                        data = typestore.serialize_cdr(message, IMU_TARGET_TYPE)
                    writer.write(output_connections[connection.topic], timestamp, data)
                    counters[connection.topic] += 1
        except Exception:
            shutil.rmtree(args.dst, ignore_errors=True)
            raise

    print("ROS2 conversion complete:")
    print(f"  LiDAR {LIVOX_SOURCE_TOPIC}: {counters[LIVOX_SOURCE_TOPIC]} messages")
    print(f"  IMU   {IMU_SOURCE_TOPIC}: {counters[IMU_SOURCE_TOPIC]} messages")
    print(f"  Output: {args.dst}")


if __name__ == "__main__":
    main()
