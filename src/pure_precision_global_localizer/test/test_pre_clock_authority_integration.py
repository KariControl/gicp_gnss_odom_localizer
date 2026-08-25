#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Exercise the real node's authority startup queue before positive ROS time."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from pure_gnss_msgs.msg import FusionAuthority
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock


STATUS_NAME = "localization/precision_global_localizer"


def authority_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def diagnostic_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def clock_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def make_authority(sequence: int) -> FusionAuthority:
    message = FusionAuthority()
    message.header.stamp.sec = 100
    message.header.frame_id = "map"
    message.source_stamp.sec = 99
    message.source_stamp.nanosec = 900_000_000
    message.session_id = 4242
    message.sequence = sequence
    message.state = FusionAuthority.FULL_SE2_HEALTHY
    message.reason = "pre_clock_integration_fixture"
    message.recovery_state = "tracking"
    message.anchor_valid = True
    message.position_fused = True
    message.yaw_fused = True
    message.last_fix_state = FusionAuthority.FIX_GOOD
    return message


def wait_until(
    node: Node,
    predicate: Callable[[], bool],
    timeout_sec: float,
    pump: Callable[[], None] | None = None,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if pump is not None:
            pump()
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError("timed out waiting for integration-test condition")


def value_int(values: dict[str, str], key: str) -> int:
    try:
        return int(values[key])
    except (KeyError, ValueError) as error:
        raise AssertionError(f"missing or malformed diagnostic counter {key}") from error


class Scenario:
    def __init__(self, node: Node, executable: Path, suffix: str) -> None:
        self._node = node
        self._authority_topic = f"/test/pre_clock/{suffix}/authority"
        self._diagnostic_topic = f"/test/pre_clock/{suffix}/diagnostics"
        self._snapshots: list[dict[str, Any]] = []
        self._authority_publisher = node.create_publisher(
            FusionAuthority, self._authority_topic, authority_qos()
        )
        self._clock_publisher = node.create_publisher(
            Clock, "/clock", clock_qos()
        )
        self._diagnostic_subscription = node.create_subscription(
            DiagnosticArray,
            self._diagnostic_topic,
            self._on_diagnostic,
            diagnostic_qos(),
        )
        command = [
            str(executable),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "diagnostics_period_sec:=0.1",
            "-r",
            f"__node:=precision_pre_clock_{suffix}",
            "-r",
            (
                "/localization/gnss_map_odom_fusion_authority:="
                f"{self._authority_topic}"
            ),
            "-r",
            f"/diagnostics:={self._diagnostic_topic}",
        ]
        self._process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

    def _on_diagnostic(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != STATUS_NAME and not status.name.endswith(
                "/" + STATUS_NAME
            ):
                continue
            self._snapshots.append(
                {
                    "stamp_ns": (
                        int(message.header.stamp.sec) * 1_000_000_000
                        + int(message.header.stamp.nanosec)
                    ),
                    "values": {item.key: item.value for item in status.values},
                }
            )

    def wait_for_authority_subscription(self) -> None:
        wait_until(
            self._node,
            lambda: self._authority_publisher.get_subscription_count() == 1,
            5.0,
        )

    def publish_authority(self, sequence: int) -> None:
        self._authority_publisher.publish(make_authority(sequence))

    def publish_positive_clock(self) -> None:
        message = Clock()
        message.clock.sec = 100
        message.clock.nanosec = 100_000_000
        self._clock_publisher.publish(message)

    def matching_snapshot(
        self, predicate: Callable[[dict[str, Any]], bool]
    ) -> dict[str, Any] | None:
        return next((item for item in reversed(self._snapshots) if predicate(item)), None)

    def wait_for_snapshot(
        self,
        predicate: Callable[[dict[str, Any]], bool],
        *,
        pump: Callable[[], None] | None = None,
    ) -> dict[str, Any]:
        wait_until(
            self._node,
            lambda: self.matching_snapshot(predicate) is not None,
            7.0,
            pump,
        )
        result = self.matching_snapshot(predicate)
        assert result is not None
        return result

    def close(self) -> None:
        if self._process.poll() is None:
            self._process.send_signal(signal.SIGINT)
            try:
                self._process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=3.0)
        output = self._process.stdout.read() if self._process.stdout else ""
        if self._process.returncode not in (0, -signal.SIGINT):
            raise AssertionError(
                f"precision node exited with {self._process.returncode}: {output[-2000:]}"
            )
        self._node.destroy_subscription(self._diagnostic_subscription)
        self._node.destroy_publisher(self._authority_publisher)
        self._node.destroy_publisher(self._clock_publisher)


def assert_startup_defer_and_drain(node: Node, executable: Path) -> None:
    scenario = Scenario(node, executable, "drain")
    try:
        scenario.wait_for_authority_subscription()
        scenario.publish_authority(1)
        queued = scenario.wait_for_snapshot(
            lambda item: value_int(item["values"], "fusion.authority.deferred") == 1
        )
        queued_values = queued["values"]
        assert queued["stamp_ns"] == 0
        assert value_int(queued_values, "fusion.authority.received") == 1
        assert value_int(queued_values, "fusion.authority.accepted") == 0
        assert value_int(queued_values, "fusion.authority.rejected") == 0
        assert value_int(queued_values, "fusion.authority.pending") == 1
        assert queued_values["fusion.authority.receive_clock_initialized"] == "false"

        drained = scenario.wait_for_snapshot(
            lambda item: (
                item["stamp_ns"] == 100_100_000_000
                and value_int(item["values"], "fusion.authority.accepted") == 1
            ),
            pump=scenario.publish_positive_clock,
        )
        drained_values = drained["values"]
        assert value_int(drained_values, "fusion.authority.received") == 1
        assert value_int(drained_values, "fusion.authority.rejected") == 0
        assert value_int(drained_values, "fusion.authority.deferred") == 1
        assert value_int(drained_values, "fusion.authority.pending") == 0
        assert value_int(drained_values, "fusion.authority.deferred_overflow") == 0
        assert drained_values["fusion.authority.receive_clock_initialized"] == "true"
        assert drained_values["fusion.authority.startup_overflow_latched"] == "false"
        assert drained_values["fusion.health.authority_session_id"] == "4242"
        assert drained_values["fusion.health.authority_sequence"] == "1"
        assert drained_values["fusion.health.authority_received_stamp_sec"] == "100.1"
    finally:
        scenario.close()


def assert_startup_overflow_fails_closed(node: Node, executable: Path) -> None:
    scenario = Scenario(node, executable, "overflow")
    try:
        scenario.wait_for_authority_subscription()
        for sequence in range(1, 66):
            scenario.publish_authority(sequence)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.005)
        overflow = scenario.wait_for_snapshot(
            lambda item: (
                value_int(item["values"], "fusion.authority.deferred_overflow") == 1
            )
        )
        values = overflow["values"]
        assert overflow["stamp_ns"] == 0
        assert value_int(values, "fusion.authority.received") == 65
        assert value_int(values, "fusion.authority.accepted") == 0
        assert value_int(values, "fusion.authority.rejected") == 65
        assert value_int(values, "fusion.authority.deferred") == 65
        assert value_int(values, "fusion.authority.pending") == 0
        assert values["fusion.authority.receive_clock_initialized"] == "false"
        assert values["fusion.authority.startup_overflow_latched"] == "true"

        scenario.publish_authority(66)
        latched = scenario.wait_for_snapshot(
            lambda item: (
                value_int(item["values"], "fusion.authority.received") == 66
            )
        )
        latched_values = latched["values"]
        assert value_int(latched_values, "fusion.authority.rejected") == 66
        assert value_int(latched_values, "fusion.authority.deferred") == 65
        assert value_int(latched_values, "fusion.authority.pending") == 0
        assert latched_values["fusion.authority.startup_overflow_latched"] == "true"
    finally:
        scenario.close()


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_pre_clock_authority_integration.py NODE_EXECUTABLE")
    executable = Path(sys.argv[1]).resolve()
    if not executable.is_file():
        raise AssertionError(f"node executable does not exist: {executable}")
    log_dir = Path(os.environ.get("ROS_LOG_DIR", "/tmp/pre_clock_authority_test_logs"))
    log_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Node("pre_clock_authority_integration_test")
    try:
        assert_startup_defer_and_drain(node, executable)
        assert_startup_overflow_fails_closed(node, executable)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("PASS: pre-clock authority node integration")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
