#!/usr/bin/env python3
"""Short manual ROS e2e for accepted-scan snapshots and matcher corrections."""

import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray

from pure_lidar_msgs.msg import SubmapCorrection, SubmapScan


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Harness(Node):
    def __init__(self):
        super().__init__("synthetic_snapshot_matcher_e2e")
        self.publisher = self.create_publisher(
            PointCloud2, "/localization/points_undistorted", qos_profile_sensor_data
        )
        self.create_subscription(
            SubmapScan,
            "/localization/submap_scan",
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SubmapCorrection,
            "/localization/submap_correction",
            self.on_correction,
            10,
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self.on_diagnostics, 10
        )
        self.scans = {}
        self.correction_count = 0
        self.last_correction_id = 0
        self.errors = []
        self.matcher_diagnostics = {}
        self.odometer_diagnostics = {}
        rng = random.Random(193)
        self.world_points = [
            (rng.uniform(3.0, 25.0), rng.uniform(-8.0, 8.0), rng.uniform(-2.0, 2.0))
            for _ in range(1400)
        ]
        self.cloud_index = 0

    @staticmethod
    def key(message):
        return (
            int(message.odom_session_id),
            int(message.odom_generation),
            int(message.sequence),
            stamp_ns(message.header.stamp),
        )

    def publish_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        vehicle_x = 0.02 * self.cloud_index
        points = [(x - vehicle_x, y, z) for x, y, z in self.world_points]
        self.cloud_index += 1
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, points))

    def on_scan(self, message):
        if stamp_ns(message.header.stamp) != stamp_ns(message.cloud.header.stamp):
            self.errors.append("snapshot header/cloud stamp mismatch")
        self.scans[self.key(message)] = message

    def on_correction(self, message):
        key = self.key(message)
        scan = self.scans.get(key)
        if scan is None:
            self.errors.append("correction has no exact-key snapshot")
            return
        if message.precision_frame_id != "odom_precision":
            self.errors.append("unexpected precision frame")
        if int(message.correction_id) <= self.last_correction_id:
            self.errors.append("nonmonotonic correction id")
        self.last_correction_id = int(message.correction_id)

        transform = message.precision_from_raw
        raw = scan.raw_pose.pose
        transform_yaw = yaw(transform.rotation)
        raw_yaw = yaw(raw.orientation)
        expected_x = (
            transform.translation.x
            + math.cos(transform_yaw) * raw.position.x
            - math.sin(transform_yaw) * raw.position.y
        )
        expected_y = (
            transform.translation.y
            + math.sin(transform_yaw) * raw.position.x
            + math.cos(transform_yaw) * raw.position.y
        )
        corrected = message.corrected_pose.pose
        if math.hypot(corrected.position.x - expected_x, corrected.position.y - expected_y) > 1e-5:
            self.errors.append("corrected pose/transform translation mismatch")
        if abs(angle_error(yaw(corrected.orientation), transform_yaw + raw_yaw)) > 1e-5:
            self.errors.append("corrected pose/transform yaw mismatch")
        self.correction_count += 1

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name == "localization/submap_matcher":
                self.matcher_diagnostics = {item.key: item.value for item in status.values}
            elif status.name == "localization/gyro_odometer":
                self.odometer_diagnostics = {item.key: item.value for item in status.values}


def main():
    rclpy.init()
    node = Harness()
    discovery_deadline = time.monotonic() + 1.0
    while time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

    deadline = time.monotonic() + 7.0
    next_publish = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_publish:
            node.publish_cloud()
            next_publish += 0.05
        rclpy.spin_once(node, timeout_sec=0.005)

    drain_deadline = time.monotonic() + 1.0
    while time.monotonic() < drain_deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

    print(
        f"snapshots={len(node.scans)} corrections={node.correction_count} "
        f"errors={len(node.errors)}"
    )
    print(f"matcher_diagnostics={node.matcher_diagnostics}")
    print(
        "snapshot_conversion_ms="
        f"last:{node.odometer_diagnostics.get('external_submap_snapshot_conversion_last_ms')} "
        f"mean:{node.odometer_diagnostics.get('external_submap_snapshot_conversion_mean_ms')} "
        f"max:{node.odometer_diagnostics.get('external_submap_snapshot_conversion_max_ms')}"
    )
    for error in node.errors:
        print(f"ERROR: {error}", file=sys.stderr)
    passed = len(node.scans) >= 4 and node.correction_count >= 1 and not node.errors
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
