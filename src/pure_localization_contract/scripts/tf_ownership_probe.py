#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Verify one configured ROS 2 publisher endpoint per dynamic TF edge."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from tf2_msgs.msg import TFMessage


class OwnershipError(RuntimeError):
    """A runtime TF ownership contract was not satisfied."""

    def __init__(
        self, message: str, evidence: dict[str, Any] | None = None
    ) -> None:
        super().__init__(message)
        self.evidence = evidence


def normalize_node_name(name: str) -> str:
    parts = [part for part in name.strip().split("/") if part]
    if not parts:
        raise ValueError("node name must not be empty")
    return "/" + "/".join(parts)


def endpoint_node_name(namespace: str, name: str) -> str:
    if namespace in ("", "/"):
        return normalize_node_name(name)
    return normalize_node_name(f"{namespace}/{name}")


@dataclass(frozen=True)
class OwnerExpectation:
    node: str
    parent_parameter: str
    parent_frame: str
    child_parameter: str
    child_frame: str

    @property
    def edge(self) -> tuple[str, str]:
        return self.parent_frame, self.child_frame

    @classmethod
    def parse(cls, text: str) -> "OwnerExpectation":
        fields = [field.strip() for field in text.split(",")]
        if len(fields) != 5 or any(not field for field in fields):
            raise ValueError(
                "owner must be NODE,PARENT_PARAMETER,PARENT_FRAME,"
                "CHILD_PARAMETER,CHILD_FRAME"
            )
        return cls(normalize_node_name(fields[0]), *fields[1:])


def publisher_record(info: Any) -> dict[str, str]:
    return {
        "node": endpoint_node_name(info.node_namespace, info.node_name),
        "endpoint_gid": bytes(info.endpoint_gid).hex(),
        "topic_type": str(info.topic_type),
    }


def write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


class TfOwnershipProbe(Node):
    def __init__(self, initialization_odom_topic: str | None) -> None:
        super().__init__("tf_ownership_probe")
        self.observed_edges: Counter[tuple[str, str]] = Counter()
        self.latest_odometry: Odometry | None = None
        self.last_publishers: list[dict[str, str]] = []
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)

        self.initialpose_publisher = None
        if initialization_odom_topic:
            self.create_subscription(
                Odometry,
                initialization_odom_topic,
                self.on_odometry,
                20,
            )
            self.initialpose_publisher = self.create_publisher(
                PoseWithCovarianceStamped, "/initialpose", 10
            )

    def on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.observed_edges[
                (str(transform.header.frame_id), str(transform.child_frame_id))
            ] += 1

    def on_odometry(self, message: Odometry) -> None:
        self.latest_odometry = message

    def graph_nodes(self) -> set[str]:
        return {
            endpoint_node_name(namespace, name)
            for name, namespace in self.get_node_names_and_namespaces()
        }

    def publisher_snapshot(self) -> list[dict[str, str]]:
        records = [
            publisher_record(info)
            for info in self.get_publishers_info_by_topic("/tf")
        ]
        records.sort(key=lambda record: (record["node"], record["endpoint_gid"]))
        self.last_publishers = records
        return records

    @staticmethod
    def publisher_counts(records: list[dict[str, str]]) -> Counter[str]:
        return Counter(record["node"] for record in records)

    def wait_for_exact_graph(
        self,
        expected_counts: Counter[str],
        required_nodes: set[str],
        timeout_sec: float,
        settle_sec: float,
    ) -> list[dict[str, str]]:
        deadline = time.monotonic() + timeout_sec
        stable_since: float | None = None
        last_nodes: set[str] = set()
        last_counts: Counter[str] = Counter()
        last_records: list[dict[str, str]] = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            last_nodes = self.graph_nodes()
            last_records = self.publisher_snapshot()
            last_counts = self.publisher_counts(last_records)
            exact = last_counts == expected_counts and required_nodes <= last_nodes
            if exact:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= settle_sec:
                    return last_records
            else:
                stable_since = None

        missing = sorted(required_nodes - last_nodes)
        failure_evidence = {
            "expected_publisher_counts": dict(sorted(expected_counts.items())),
            "observed_publisher_counts": dict(sorted(last_counts.items())),
            "observed_publishers": last_records,
            "missing_nodes": missing,
        }
        raise OwnershipError(
            "runtime /tf publisher graph did not settle to the exact contract: "
            f"expected={dict(sorted(expected_counts.items()))}, "
            f"observed={dict(sorted(last_counts.items()))}, "
            f"missing_nodes={missing}",
            failure_evidence,
        )

    def get_remote_parameters(
        self, target_node: str, names: list[str], timeout_sec: float
    ) -> dict[str, Any]:
        client = AsyncParameterClient(self, target_node)
        if not client.wait_for_services(timeout_sec=timeout_sec):
            raise OwnershipError(
                f"parameter services for owner {target_node!r} were unavailable"
            )
        future = client.get_parameters(names)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise OwnershipError(
                f"parameter query for owner {target_node!r} timed out"
            )
        if future.exception() is not None:
            raise OwnershipError(
                f"parameter query for owner {target_node!r} failed: "
                f"{future.exception()}"
            )
        response = future.result()
        if response is None or len(response.values) != len(names):
            raise OwnershipError(
                f"parameter query for owner {target_node!r} returned an invalid response"
            )
        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, response.values)
        }

    def publish_initialpose(self) -> bool:
        if self.initialpose_publisher is None or self.latest_odometry is None:
            return False
        source = self.latest_odometry
        message = PoseWithCovarianceStamped()
        message.header.stamp = source.header.stamp
        message.header.frame_id = "map"
        message.pose.pose = source.pose.pose
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[14] = 1.0e6
        message.pose.covariance[21] = 1.0e6
        message.pose.covariance[28] = 1.0e6
        message.pose.covariance[35] = 0.04
        self.initialpose_publisher.publish(message)
        return True

    def wait_for_exact_edges(
        self,
        expected_edges: set[tuple[str, str]],
        min_samples: int,
        timeout_sec: float,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        next_initialpose = time.monotonic()
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_initialpose:
                self.publish_initialpose()
                next_initialpose = now + 0.2

            observed = set(self.observed_edges)
            unexpected = observed - expected_edges
            if unexpected:
                raise OwnershipError(
                    "unexpected dynamic TF edge(s) were emitted: "
                    + ", ".join(
                        f"{parent}->{child}"
                        for parent, child in sorted(unexpected)
                    )
                )
            if all(self.observed_edges[edge] >= min_samples for edge in expected_edges):
                return

        counts = {
            f"{parent}->{child}": self.observed_edges[(parent, child)]
            for parent, child in sorted(expected_edges)
        }
        raise OwnershipError(
            "expected dynamic TF edges were not all observed at the required rate: "
            f"minimum_samples={min_samples}, counts={counts}"
        )


def validate_expectations(
    owners: list[OwnerExpectation], disabled_nodes: list[str]
) -> None:
    owner_nodes = [owner.node for owner in owners]
    if len(owner_nodes) != len(set(owner_nodes)):
        raise ValueError("an owner node may be specified only once")
    edges = [owner.edge for owner in owners]
    duplicate_edges = [edge for edge, count in Counter(edges).items() if count > 1]
    if duplicate_edges:
        formatted = ", ".join(
            f"{parent}->{child}" for parent, child in sorted(duplicate_edges)
        )
        raise ValueError(f"each dynamic TF edge must have one owner; duplicates: {formatted}")
    overlap = set(owner_nodes) & set(disabled_nodes)
    if overlap:
        raise ValueError(f"nodes cannot be both owner and disabled: {sorted(overlap)}")


def run_probe(arguments: argparse.Namespace) -> dict[str, Any]:
    owners = [OwnerExpectation.parse(text) for text in arguments.owner]
    disabled_nodes = [normalize_node_name(node) for node in arguments.disabled_owner]
    required_nodes = {
        normalize_node_name(node) for node in arguments.required_node
    }
    validate_expectations(owners, disabled_nodes)

    expected_counts = Counter(owner.node for owner in owners)
    required_nodes.update(owner.node for owner in owners)
    required_nodes.update(disabled_nodes)
    expected_edges = {owner.edge for owner in owners}

    probe = TfOwnershipProbe(arguments.initialize_map_odom_from)
    try:
        publishers = probe.wait_for_exact_graph(
            expected_counts,
            required_nodes,
            arguments.timeout,
            arguments.settle_sec,
        )

        owner_evidence: list[dict[str, Any]] = []
        for owner in owners:
            values = probe.get_remote_parameters(
                owner.node,
                ["publish_tf", owner.parent_parameter, owner.child_parameter],
                arguments.timeout,
            )
            if values["publish_tf"] is not True:
                raise OwnershipError(
                    f"configured TF owner {owner.node!r} has publish_tf="
                    f"{values['publish_tf']!r}, expected True"
                )
            if values[owner.parent_parameter] != owner.parent_frame:
                raise OwnershipError(
                    f"{owner.node!r} {owner.parent_parameter}="
                    f"{values[owner.parent_parameter]!r}, expected {owner.parent_frame!r}"
                )
            if values[owner.child_parameter] != owner.child_frame:
                raise OwnershipError(
                    f"{owner.node!r} {owner.child_parameter}="
                    f"{values[owner.child_parameter]!r}, expected {owner.child_frame!r}"
                )
            endpoint = next(record for record in publishers if record["node"] == owner.node)
            owner_evidence.append(
                {
                    "node": owner.node,
                    "edge": f"{owner.parent_frame}->{owner.child_frame}",
                    "publish_tf": True,
                    "endpoint_gid": endpoint["endpoint_gid"],
                    "parent_parameter": owner.parent_parameter,
                    "child_parameter": owner.child_parameter,
                }
            )

        disabled_evidence: list[dict[str, Any]] = []
        for node_name in disabled_nodes:
            values = probe.get_remote_parameters(
                node_name, ["publish_tf"], arguments.timeout
            )
            if values["publish_tf"] is not False:
                raise OwnershipError(
                    f"non-owner {node_name!r} has publish_tf="
                    f"{values['publish_tf']!r}, expected False"
                )
            disabled_evidence.append({"node": node_name, "publish_tf": False})

        if not arguments.skip_edge_samples:
            probe.wait_for_exact_edges(
                expected_edges,
                arguments.min_edge_samples,
                arguments.timeout,
            )

        final_publishers = probe.publisher_snapshot()
        if probe.publisher_counts(final_publishers) != expected_counts:
            raise OwnershipError("/tf publisher endpoints changed during the probe")

        observed_edge_counts = {
            f"{parent}->{child}": count
            for (parent, child), count in sorted(probe.observed_edges.items())
        }
        return {
            "schema_version": 1,
            "result": "PASS",
            "failure": None,
            "tf_topic": "/tf",
            "publisher_endpoints": final_publishers,
            "owners": owner_evidence,
            "disabled_owners": disabled_evidence,
            "required_nodes": sorted(required_nodes),
            "observed_edge_counts": observed_edge_counts,
            "edge_samples_checked": not arguments.skip_edge_samples,
        }
    finally:
        probe.destroy_node()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Require exactly one runtime /tf publisher endpoint for each declared "
            "dynamic TF edge."
        )
    )
    parser.add_argument(
        "--owner",
        action="append",
        default=[],
        metavar="NODE,PARENT_PARAMETER,PARENT_FRAME,CHILD_PARAMETER,CHILD_FRAME",
        help="Expected TF owner; repeat for each dynamic edge.",
    )
    parser.add_argument(
        "--disabled-owner",
        action="append",
        default=[],
        help="Node that must exist, expose publish_tf=false, and own no /tf endpoint.",
    )
    parser.add_argument(
        "--required-node",
        action="append",
        default=[],
        help="Node that must exist but is not expected to publish /tf.",
    )
    parser.add_argument(
        "--initialize-map-odom-from",
        metavar="ODOMETRY_TOPIC",
        help="Publish /initialpose from this local odometry until map->odom appears.",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--settle-sec", type=float, default=0.5)
    parser.add_argument("--min-edge-samples", type=int, default=2)
    parser.add_argument(
        "--skip-edge-samples",
        action="store_true",
        help="Check endpoint identity and effective parameters without waiting for TF data.",
    )
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    if not math.isfinite(arguments.timeout) or arguments.timeout <= 0.0:
        parser.error("--timeout must be a positive finite value")
    if not math.isfinite(arguments.settle_sec) or arguments.settle_sec < 0.0:
        parser.error("--settle-sec must be a non-negative finite value")
    if arguments.min_edge_samples < 1:
        parser.error("--min-edge-samples must be at least one")
    return arguments


def main() -> int:
    arguments = parse_arguments()
    rclpy.init(args=[])
    exit_code = 0
    try:
        document = run_probe(arguments)
    except Exception as exception:  # noqa: BLE001 - test boundary records every failure
        exit_code = 1
        document = {
            "schema_version": 1,
            "result": "FAIL",
            "failure": f"{type(exception).__name__}: {exception}",
        }
        if isinstance(exception, OwnershipError) and exception.evidence is not None:
            document["failure_evidence"] = exception.evidence
        print(f"FAIL: {exception}", file=sys.stderr)
    finally:
        rclpy.shutdown()

    if arguments.output is not None:
        try:
            write_json(arguments.output, document)
        except Exception as exception:  # noqa: BLE001 - artifact write is contractual
            print(f"FAIL: could not write ownership evidence: {exception}", file=sys.stderr)
            exit_code = 1
    print(json.dumps(document, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
