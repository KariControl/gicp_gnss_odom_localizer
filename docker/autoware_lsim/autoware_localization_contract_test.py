#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Drive and verify the pinned Autoware localization-monitor contract."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import (
    AccelWithCovarianceStamped,
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


POSE_DIAGNOSTIC = "localization: pose_instability_detector"
ELLIPSE_DIAGNOSTIC = "localization_error_monitor: ellipse_error_status"
ADAPTER_DIAGNOSTIC = "localization/localization_interface_adapter"
DT_SEC = 0.05
LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def diagnostic_level(value: object) -> int:
    # Jazzy's generated uint8 field can be exposed as a one-byte object,
    # whereas other generators expose an int.
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else 0
    return int(value)


@dataclass
class DiagnosticSample:
    phase: str
    header_stamp_sec: int
    header_stamp_nanosec: int
    received_index: int
    level: int
    message: str
    values: dict[str, str]

    def as_dict(self) -> dict[str, object]:
        return {
            "phase": self.phase,
            "header_stamp": {
                "sec": self.header_stamp_sec,
                "nanosec": self.header_stamp_nanosec,
            },
            "received_index": self.received_index,
            "level": self.level,
            "level_name": LEVEL_NAMES.get(self.level, str(self.level)),
            "message": self.message,
            "values": dict(sorted(self.values.items())),
        }


class ContractProbe(Node):
    def __init__(self) -> None:
        super().__init__("autoware_localization_contract_test")
        self.phase = "startup"
        self.sequence = 0
        self.x = 0.0
        self.input_by_stamp: dict[tuple[int, int], Odometry] = {}
        self.diagnostic_history: dict[str, list[DiagnosticSample]] = {}
        self.latest_diagnostic: dict[str, DiagnosticSample] = {}
        self.diagnostic_sequence = 0
        self.selected_evidence: dict[str, dict[str, DiagnosticSample]] = {}
        self.failures: set[str] = set()
        self.counts = {
            "kinematic_state": 0,
            "twist": 0,
            "pose": 0,
            "acceleration": 0,
            "base_link_tf": 0,
        }

        self.input_publisher = self.create_publisher(
            Odometry, "/test/localization/ekf_odom", 10
        )
        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.on_kinematic_state,
            20,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/localization/twist_with_covariance",
            self.on_twist,
            20,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/pose_with_covariance",
            self.on_pose,
            20,
        )
        self.create_subscription(
            AccelWithCovarianceStamped,
            "/localization/acceleration",
            self.on_acceleration,
            20,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self.on_diagnostics, 100
        )

    @staticmethod
    def stamp_key(message: object) -> tuple[int, int]:
        stamp = message.header.stamp
        return int(stamp.sec), int(stamp.nanosec)

    def note_failure(self, message: str) -> None:
        self.failures.add(message)

    def expected_input(self, message: object, label: str) -> Odometry | None:
        key = self.stamp_key(message)
        expected = self.input_by_stamp.get(key)
        if expected is None:
            self.note_failure(f"{label}: output stamp {key} has no input sample")
        return expected

    @staticmethod
    def same_number(lhs: float, rhs: float) -> bool:
        return math.isclose(lhs, rhs, rel_tol=0.0, abs_tol=1.0e-12)

    def on_kinematic_state(self, message: Odometry) -> None:
        self.counts["kinematic_state"] += 1
        expected = self.expected_input(message, "kinematic_state")
        if expected is None:
            return
        if message.header.frame_id != "map" or message.child_frame_id != "base_link":
            self.note_failure("kinematic_state: expected map -> base_link frames")
        if message.pose != expected.pose:
            self.note_failure("kinematic_state: pose or covariance was not preserved")
        if message.twist != expected.twist:
            self.note_failure("kinematic_state: twist was not preserved")

    def on_twist(self, message: TwistWithCovarianceStamped) -> None:
        self.counts["twist"] += 1
        expected = self.expected_input(message, "twist")
        if expected is None:
            return
        if message.header.frame_id != "base_link":
            self.note_failure("twist: frame_id must be the odometry child/base frame")
        if message.twist != expected.twist:
            self.note_failure("twist: values or covariance were not preserved")

    def on_pose(self, message: PoseWithCovarianceStamped) -> None:
        self.counts["pose"] += 1
        expected = self.expected_input(message, "pose")
        if expected is None:
            return
        if message.header.frame_id != "map":
            self.note_failure("pose: frame_id must be map")
        if message.pose != expected.pose:
            self.note_failure("pose: value or covariance was not preserved")

    def on_acceleration(self, message: AccelWithCovarianceStamped) -> None:
        self.counts["acceleration"] += 1
        expected = self.expected_input(message, "acceleration")
        if expected is None:
            return
        if message.header.frame_id != "base_link":
            self.note_failure("acceleration: frame_id must be base_link")
        for value in (
            message.accel.accel.linear.x,
            message.accel.accel.linear.y,
            message.accel.accel.linear.z,
            message.accel.accel.angular.x,
            message.accel.accel.angular.y,
            message.accel.accel.angular.z,
        ):
            if not math.isfinite(value):
                self.note_failure("acceleration: non-finite value")
        covariance = list(message.accel.covariance)
        if not all(math.isfinite(value) for value in covariance):
            self.note_failure("acceleration: non-finite covariance")
        if any(covariance[index] < 0.0 for index in (0, 7, 14, 21, 28, 35)):
            self.note_failure("acceleration: negative diagonal covariance")

    def on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.child_frame_id != "base_link":
                continue
            self.counts["base_link_tf"] += 1
            if transform.header.frame_id != "map":
                self.note_failure(
                    "TF ownership: base_link received a dynamic parent other than map"
                )
            expected = self.expected_input(transform, "TF")
            if expected is None:
                continue
            translation_matches = all(
                self.same_number(actual, wanted)
                for actual, wanted in (
                    (
                        transform.transform.translation.x,
                        expected.pose.pose.position.x,
                    ),
                    (
                        transform.transform.translation.y,
                        expected.pose.pose.position.y,
                    ),
                    (
                        transform.transform.translation.z,
                        expected.pose.pose.position.z,
                    ),
                )
            )
            if not translation_matches:
                self.note_failure("TF: translation does not match input odometry")
            if transform.transform.rotation != expected.pose.pose.orientation:
                self.note_failure("TF: orientation does not match input odometry")

    def on_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            sample = DiagnosticSample(
                phase=self.phase,
                header_stamp_sec=int(message.header.stamp.sec),
                header_stamp_nanosec=int(message.header.stamp.nanosec),
                received_index=self.diagnostic_sequence,
                level=diagnostic_level(status.level),
                message=status.message,
                values={value.key: value.value for value in status.values},
            )
            self.diagnostic_sequence += 1
            self.latest_diagnostic[status.name] = sample
            self.diagnostic_history.setdefault(status.name, []).append(sample)

    def make_odometry(self, velocity: float, variance_xy: float) -> Odometry:
        message = Odometry()
        total_nanoseconds = self.sequence * int(DT_SEC * 1.0e9)
        message.header.stamp.sec = 1000 + total_nanoseconds // 1_000_000_000
        message.header.stamp.nanosec = total_nanoseconds % 1_000_000_000
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self.x
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = variance_xy
        message.pose.covariance[7] = variance_xy
        message.pose.covariance[14] = 1.0
        message.pose.covariance[21] = 1.0
        message.pose.covariance[28] = 1.0
        message.pose.covariance[35] = 0.001
        message.twist.twist.linear.x = velocity
        message.twist.covariance[0] = 0.01
        message.twist.covariance[7] = 0.01
        message.twist.covariance[14] = 1.0
        message.twist.covariance[21] = 1.0
        message.twist.covariance[28] = 1.0
        message.twist.covariance[35] = 0.001
        return message

    def publish_phase(
        self,
        phase: str,
        duration_sec: float,
        *,
        velocity: float,
        variance_xy: float,
        jump_x: float = 0.0,
    ) -> set[tuple[int, int]]:
        self.phase = phase
        phase_stamps: set[tuple[int, int]] = set()
        if jump_x:
            self.x += jump_x
        sample_count = int(math.ceil(duration_sec / DT_SEC))
        for _ in range(sample_count):
            self.x += velocity * DT_SEC
            message = self.make_odometry(velocity, variance_xy)
            stamp = self.stamp_key(message)
            phase_stamps.add(stamp)
            self.input_by_stamp[stamp] = message
            self.input_publisher.publish(message)
            self.sequence += 1
            deadline = time.monotonic() + DT_SEC
            while time.monotonic() < deadline:
                rclpy.spin_once(
                    self, timeout_sec=min(0.01, deadline - time.monotonic())
                )
        return phase_stamps

    def wait_for_input_subscription(self, timeout_sec: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.input_publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError("adapter did not subscribe to the contract input topic")

    def diagnostic_cursor(self, name: str) -> int:
        return len(self.diagnostic_history.get(name, []))

    def samples_since(self, name: str, cursor: int) -> list[DiagnosticSample]:
        return self.diagnostic_history.get(name, [])[cursor:]

    def require_latest_level_since(
        self,
        name: str,
        cursor: int,
        expected: int,
        context: str,
    ) -> DiagnosticSample:
        samples = self.samples_since(name, cursor)
        if not samples:
            raise RuntimeError(f"{context}: missing new diagnostic {name!r}")
        sample = samples[-1]
        if sample.level != expected:
            raise RuntimeError(
                f"{context}: {name!r} level={sample.level}, expected={expected}; "
                f"message={sample.message!r}"
            )
        return sample

    def require_level_since(
        self,
        name: str,
        cursor: int,
        expected: int,
        context: str,
    ) -> DiagnosticSample:
        samples = [
            sample
            for sample in self.samples_since(name, cursor)
            if sample.level == expected
        ]
        if not samples:
            observed = [sample.level for sample in self.samples_since(name, cursor)]
            raise RuntimeError(
                f"{context}: {name!r} did not produce level={expected}; "
                f"observed={observed}"
            )
        return samples[0]

    def record_evidence(
        self, phase: str, check: str, sample: DiagnosticSample
    ) -> None:
        self.selected_evidence.setdefault(phase, {})[check] = sample

    @staticmethod
    def require_finite_value(
        sample: DiagnosticSample, key: str, context: str
    ) -> float:
        value = sample.values.get(key)
        if value is None:
            raise RuntimeError(f"{context}: missing diagnostic key {key!r}")
        try:
            number = float(value)
        except ValueError as exception:
            raise RuntimeError(
                f"{context}: diagnostic key {key!r} is not numeric: {value!r}"
            ) from exception
        if not math.isfinite(number):
            raise RuntimeError(
                f"{context}: diagnostic key {key!r} is non-finite: {value!r}"
            )
        return number

    def assert_pose_x_error(self, sample: DiagnosticSample) -> None:
        context = "pose fault phase"
        if sample.values.get("diff_position_x:status") != "ERROR":
            raise RuntimeError(
                f"{context}: x status was not ERROR: "
                f"{sample.values.get('diff_position_x:status')!r}"
            )
        value = self.require_finite_value(
            sample, "diff_position_x:value", context
        )
        threshold = self.require_finite_value(
            sample, "diff_position_x:threshold", context
        )
        if abs(value) < threshold:
            raise RuntimeError(
                f"{context}: abs(x difference)={abs(value)} was below "
                f"threshold={threshold}"
            )

    def assert_ellipse_radius(
        self,
        sample: DiagnosticSample,
        expected_radius: float,
        context: str,
    ) -> None:
        for key in (
            "localization_error_ellipse",
            "localization_error_ellipse_lateral_direction",
        ):
            value = self.require_finite_value(sample, key, context)
            if not math.isclose(
                value, expected_radius, rel_tol=1.0e-3, abs_tol=1.0e-3
            ):
                raise RuntimeError(
                    f"{context}: {key}={value} did not match the injected "
                    f"3-sigma radius {expected_radius}"
                )

    def assert_pose_stamp_belongs_to_phase(
        self,
        sample: DiagnosticSample,
        phase_stamps: set[tuple[int, int]],
        context: str,
    ) -> None:
        # Autoware 1.9.0's pose-instability detector stamps diagnostics with
        # the latest odometry stamp. The ellipse monitor instead uses now(),
        # so only pose diagnostics can be correlated this way.
        stamp = (sample.header_stamp_sec, sample.header_stamp_nanosec)
        if stamp not in phase_stamps:
            raise RuntimeError(
                f"{context}: pose diagnostic stamp {stamp} was not produced "
                "from an input in this phase"
            )

    def run_contract(self) -> None:
        self.wait_for_input_subscription()

        # 3 sigma = 0.0949 m, safely below both Autoware warning thresholds.
        pose_cursor = self.diagnostic_cursor(POSE_DIAGNOSTIC)
        ellipse_cursor = self.diagnostic_cursor(ELLIPSE_DIAGNOSTIC)
        normal_stamps = self.publish_phase(
            "normal", 2.0, velocity=1.0, variance_xy=0.001
        )
        normal_pose = self.require_latest_level_since(
            POSE_DIAGNOSTIC, pose_cursor, 0, "normal phase"
        )
        normal_ellipse = self.require_latest_level_since(
            ELLIPSE_DIAGNOSTIC, ellipse_cursor, 0, "normal phase"
        )
        self.record_evidence("normal", "pose", normal_pose)
        self.record_evidence("normal", "ellipse", normal_ellipse)
        self.assert_pose_stamp_belongs_to_phase(
            normal_pose, normal_stamps, "normal phase"
        )
        self.assert_ellipse_radius(
            normal_ellipse, 3.0 * math.sqrt(0.001), "normal phase"
        )
        # A 2 m discontinuity exceeds the default 0.11 m longitudinal limit.
        # Hold the new position: returning to the old x would create a second jump.
        pose_cursor = self.diagnostic_cursor(POSE_DIAGNOSTIC)
        ellipse_cursor = self.diagnostic_cursor(ELLIPSE_DIAGNOSTIC)
        pose_fault_stamps = self.publish_phase(
            "pose_fault",
            1.2,
            velocity=0.0,
            variance_xy=0.001,
            jump_x=2.0,
        )
        pose_fault = self.require_level_since(
            POSE_DIAGNOSTIC, pose_cursor, 2, "pose fault phase"
        )
        ellipse_during_pose_fault = self.require_latest_level_since(
            ELLIPSE_DIAGNOSTIC,
            ellipse_cursor,
            0,
            "pose fault covariance cross-check",
        )
        self.record_evidence("pose_fault", "pose_error", pose_fault)
        self.record_evidence(
            "pose_fault", "ellipse_remained_ok", ellipse_during_pose_fault
        )
        self.assert_pose_stamp_belongs_to_phase(
            pose_fault, pose_fault_stamps, "pose fault phase"
        )
        self.assert_pose_x_error(pose_fault)
        self.assert_ellipse_radius(
            ellipse_during_pose_fault,
            3.0 * math.sqrt(0.001),
            "pose fault covariance cross-check",
        )
        pose_cursor = self.diagnostic_cursor(POSE_DIAGNOSTIC)
        pose_recovery_stamps = self.publish_phase(
            "pose_recovery", 1.2, velocity=0.0, variance_xy=0.001
        )
        pose_recovery = self.require_latest_level_since(
            POSE_DIAGNOSTIC, pose_cursor, 0, "pose recovery phase"
        )
        self.record_evidence("pose_recovery", "pose", pose_recovery)
        self.assert_pose_stamp_belongs_to_phase(
            pose_recovery, pose_recovery_stamps, "pose recovery phase"
        )

        # 3 sigma = 1.5 m, at/above the default global error threshold and far
        # above the 0.30 m lateral error threshold.
        pose_cursor = self.diagnostic_cursor(POSE_DIAGNOSTIC)
        ellipse_cursor = self.diagnostic_cursor(ELLIPSE_DIAGNOSTIC)
        covariance_fault_stamps = self.publish_phase(
            "covariance_fault", 1.0, velocity=0.0, variance_xy=0.25
        )
        covariance_fault = self.require_level_since(
            ELLIPSE_DIAGNOSTIC, ellipse_cursor, 2, "covariance fault phase"
        )
        pose_during_covariance_fault = self.require_latest_level_since(
            POSE_DIAGNOSTIC,
            pose_cursor,
            0,
            "covariance fault pose cross-check",
        )
        self.record_evidence(
            "covariance_fault", "ellipse_error", covariance_fault
        )
        self.record_evidence(
            "covariance_fault", "pose_remained_ok", pose_during_covariance_fault
        )
        self.assert_ellipse_radius(
            covariance_fault, 3.0 * math.sqrt(0.25), "covariance fault phase"
        )
        self.assert_pose_stamp_belongs_to_phase(
            pose_during_covariance_fault,
            covariance_fault_stamps,
            "covariance fault pose cross-check",
        )
        pose_cursor = self.diagnostic_cursor(POSE_DIAGNOSTIC)
        ellipse_cursor = self.diagnostic_cursor(ELLIPSE_DIAGNOSTIC)
        adapter_cursor = self.diagnostic_cursor(ADAPTER_DIAGNOSTIC)
        final_stamps = self.publish_phase(
            "final_recovery", 1.2, velocity=0.0, variance_xy=0.001
        )
        final_pose = self.require_latest_level_since(
            POSE_DIAGNOSTIC, pose_cursor, 0, "final recovery"
        )
        final_ellipse = self.require_latest_level_since(
            ELLIPSE_DIAGNOSTIC, ellipse_cursor, 0, "final recovery"
        )
        final_adapter = self.require_latest_level_since(
            ADAPTER_DIAGNOSTIC, adapter_cursor, 0, "final recovery"
        )
        self.record_evidence("final_recovery", "pose", final_pose)
        self.record_evidence("final_recovery", "ellipse", final_ellipse)
        self.record_evidence("final_recovery", "adapter", final_adapter)
        self.assert_pose_stamp_belongs_to_phase(
            final_pose, final_stamps, "final recovery"
        )
        self.assert_ellipse_radius(
            final_ellipse, 3.0 * math.sqrt(0.001), "final recovery"
        )
        for output_name, count in self.counts.items():
            if count < 10:
                raise RuntimeError(
                    f"interface output {output_name!r} had only {count} samples"
                )
        if self.failures:
            raise RuntimeError("; ".join(sorted(self.failures)))

    def result_document(
        self, result: str, failure: str | None = None
    ) -> dict[str, object]:
        evidence = {
            phase: {
                check: sample.as_dict() for check, sample in sorted(checks.items())
            }
            for phase, checks in self.selected_evidence.items()
        }

        def evidence_level(phase: str, check: str) -> str | None:
            sample = self.selected_evidence.get(phase, {}).get(check)
            if sample is None:
                return None
            return LEVEL_NAMES.get(sample.level, str(sample.level))

        level_counts: dict[str, dict[str, int]] = {}
        last_samples: dict[str, dict[str, object]] = {}
        for name in (POSE_DIAGNOSTIC, ELLIPSE_DIAGNOSTIC, ADAPTER_DIAGNOSTIC):
            counts: dict[str, int] = {}
            for sample in self.diagnostic_history.get(name, []):
                level_name = LEVEL_NAMES.get(sample.level, str(sample.level))
                counts[level_name] = counts.get(level_name, 0) + 1
            level_counts[name] = counts
            latest = self.latest_diagnostic.get(name)
            if latest is not None:
                last_samples[name] = latest.as_dict()

        document: dict[str, object] = {
            "schema_version": 1,
            "result": result,
            "failure": failure,
            "environment": {
                "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
                "autoware_image": os.environ.get("AUTOWARE_IMAGE", "unknown"),
                "localizer_image_id": os.environ.get(
                    "LOCALIZER_IMAGE_ID", "unknown"
                ),
                "autoware_pose_instability_detector_version": os.environ.get(
                    "POSE_INSTABILITY_DETECTOR_VERSION", "unknown"
                ),
                "autoware_localization_error_monitor_version": os.environ.get(
                    "LOCALIZATION_ERROR_MONITOR_VERSION", "unknown"
                ),
                "use_sim_time": False,
            },
            "observed_transitions": {
                "pose_instability": [
                    evidence_level("normal", "pose"),
                    evidence_level("pose_fault", "pose_error"),
                    evidence_level("pose_recovery", "pose"),
                ],
                "covariance_ellipse": [
                    evidence_level("normal", "ellipse"),
                    evidence_level("covariance_fault", "ellipse_error"),
                    evidence_level("final_recovery", "ellipse"),
                ],
            },
            "selected_diagnostic_evidence": evidence,
            "diagnostic_level_counts": level_counts,
            "last_diagnostics": last_samples,
            "interface_counts": self.counts,
            "interface_failures": sorted(self.failures),
            "tf_owner": "localization_interface_adapter: map -> base_link",
            "diagnostic_names": [
                POSE_DIAGNOSTIC,
                ELLIPSE_DIAGNOSTIC,
                ADAPTER_DIAGNOSTIC,
            ],
        }
        return document


def write_json(path: Path, document: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify the pinned Autoware localization-monitor contract."
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Write the PASS/FAIL evidence document to this JSON path.",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    rclpy.init(args=[])
    probe = ContractProbe()
    exit_code = 0
    try:
        probe.run_contract()
        result = probe.result_document("PASS")
    except Exception as exception:  # noqa: BLE001 - test boundary reports all failures
        exit_code = 1
        failure = f"{type(exception).__name__}: {exception}"
        result = probe.result_document("FAIL", failure)
        print(f"FAIL: {exception}", file=sys.stderr)
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    if arguments.output is not None:
        try:
            write_json(arguments.output, result)
        except Exception as exception:  # noqa: BLE001 - artifact is contractual
            print(f"FAIL: could not write contract JSON: {exception}", file=sys.stderr)
            exit_code = 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
