#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Release a paused rosbag2 player only after its ROS graph is ready."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Sequence


@dataclass(frozen=True)
class InputRoute:
    source_topic: str
    destination_topic: str
    subscriber_node: str


def normalize_node_name(name: str, namespace: str = "/") -> str:
    parts = [part for part in (namespace + "/" + name).split("/") if part]
    return "/" + "/".join(parts)


def parse_input_route(value: str) -> InputRoute:
    fields = value.split("=")
    if len(fields) != 3 or any(not field.startswith("/") for field in fields):
        raise argparse.ArgumentTypeError(
            "input routes must be SOURCE=DESTINATION=SUBSCRIBER_NODE"
        )
    return InputRoute(*fields)


def stamp_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        stamp = getattr(message, "clock", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_prefix(
    expected: dict[str, Sequence[int | None]], observed: dict[str, int | None]
) -> list[str]:
    errors = []
    for topic, expected_stamps in expected.items():
        if topic not in observed:
            errors.append(f"no first message observed on {topic}")
        elif observed[topic] not in expected_stamps:
            errors.append(
                f"{topic} first header stamp is outside the accepted prefix: "
                f"expected={list(expected_stamps)} observed={observed[topic]}"
            )
    return errors


def run_self_test() -> int:
    assert normalize_node_name("node", "/") == "/node"
    assert normalize_node_name("node", "/ns/") == "/ns/node"
    route = parse_input_route("/source=/destination=/subscriber")
    assert route == InputRoute("/source", "/destination", "/subscriber")
    assert not validate_prefix({"/source": [10]}, {"/source": 10})
    assert not validate_prefix({"/source": [10, 11]}, {"/source": 11})
    assert validate_prefix({"/source": [10]}, {"/source": 11}) == [
        "/source first header stamp is outside the accepted prefix: "
        "expected=[10] observed=11"
    ]
    assert validate_prefix({"/source": [10]}, {}) == [
        "no first message observed on /source"
    ]
    clock = type("Clock", (), {"sec": 1, "nanosec": 2})()
    assert stamp_ns(type("Message", (), {"clock": clock})()) == 1_000_000_002
    print("rosbag paused-start handshake self-test PASS")
    return 0


def read_expected_prefix(
    bag: Path, routes: Sequence[InputRoute]
) -> tuple[dict[str, dict[str, Any]], dict[str, type]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=os.fspath(bag), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = sorted(
        route.source_topic
        for route in routes
        if route.source_topic not in topic_types
    )
    if missing:
        raise RuntimeError(f"input bag lacks selected topics: {missing}")

    message_classes = {
        route.source_topic: get_message(topic_types[route.source_topic])
        for route in routes
    }
    wanted = {route.source_topic for route in routes}
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))
    # Reliable IMU/NMEA probes must receive the exact first record. A remote
    # best-effort PointCloud2 publisher has no acknowledgement that can prove
    # delivery of record zero, even after both graph endpoints are stable, so
    # retain a bounded one-second (20-sample for this Hesai bag) audit prefix.
    prefix_lengths = {
        topic: 20 if topic_types[topic] == "sensor_msgs/msg/PointCloud2" else 1
        for topic in wanted
    }
    prefix: dict[str, dict[str, Any]] = {}
    while reader.has_next() and any(
        len(prefix.get(topic, {}).get("accepted_header_stamps_ns", []))
        < prefix_lengths[topic]
        for topic in wanted
    ):
        topic, serialized, record_ns = reader.read_next()
        accepted = prefix.get(topic, {}).get("accepted_header_stamps_ns", [])
        if len(accepted) >= prefix_lengths[topic]:
            continue
        message = deserialize_message(serialized, message_classes[topic])
        if topic not in prefix:
            prefix[topic] = {
                "type": topic_types[topic],
                "record_ns": int(record_ns),
                "header_stamp_ns": stamp_ns(message),
                "accepted_header_stamps_ns": [],
            }
        prefix[topic]["accepted_header_stamps_ns"].append(stamp_ns(message))
    incomplete = sorted(
        topic
        for topic in wanted
        if len(prefix.get(topic, {}).get("accepted_header_stamps_ns", []))
        < prefix_lengths[topic]
    )
    if incomplete:
        raise RuntimeError(f"selected input topics have incomplete prefixes: {incomplete}")
    return prefix, message_classes


def endpoint_name(endpoint: Any) -> str:
    return normalize_node_name(endpoint.node_name, endpoint.node_namespace)


def qos_compatible(publisher: Any, subscription: Any) -> bool:
    from rclpy.qos import QoSCompatibility, qos_check_compatible

    compatibility, _ = qos_check_compatible(
        publisher.qos_profile, subscription.qos_profile
    )
    return compatibility != QoSCompatibility.ERROR


def compatible_pair(publishers: Sequence[Any], subscriptions: Sequence[Any]) -> bool:
    return any(
        publisher.topic_type == subscription.topic_type
        and qos_compatible(publisher, subscription)
        for publisher in publishers
        for subscription in subscriptions
    )


def call_service(node: Any, client: Any, request: Any, timeout_sec: float) -> Any:
    import rclpy

    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError(f"service call timed out: {client.srv_name}")
    exception = future.exception()
    if exception is not None:
        raise RuntimeError(f"service call failed: {client.srv_name}: {exception}")
    return future.result()


def graph_errors(
    node: Any,
    routes: Sequence[InputRoute],
    player_node: str,
    probe_node: str,
    recorder_node: str | None,
    record_topics: Sequence[str],
) -> tuple[list[str], dict[str, Any]]:
    errors = []
    detail: dict[str, Any] = {"inputs": {}, "recorder_topics": {}}
    visible_nodes = {
        normalize_node_name(name, namespace)
        for name, namespace in node.get_node_names_and_namespaces()
    }
    if player_node not in visible_nodes:
        errors.append(f"player node is not visible: {player_node}")
    if recorder_node and recorder_node not in visible_nodes:
        errors.append(f"recorder node is not visible: {recorder_node}")

    clock_publishers = [
        item
        for item in node.get_publishers_info_by_topic("/clock")
        if endpoint_name(item) == player_node
    ]
    detail["player_clock_publishers"] = len(clock_publishers)
    if not clock_publishers:
        errors.append(f"player has no /clock publisher: {player_node}")

    for route in routes:
        publishers = list(node.get_publishers_info_by_topic(route.destination_topic))
        subscriptions = list(
            node.get_subscriptions_info_by_topic(route.destination_topic)
        )
        player_publishers = [
            item for item in publishers if endpoint_name(item) == player_node
        ]
        input_subscriptions = [
            item
            for item in subscriptions
            if endpoint_name(item) == route.subscriber_node
        ]
        probe_subscriptions = [
            item for item in subscriptions if endpoint_name(item) == probe_node
        ]
        matched = compatible_pair(player_publishers, input_subscriptions)
        probe_matched = compatible_pair(player_publishers, probe_subscriptions)
        detail["inputs"][route.destination_topic] = {
            "source_topic": route.source_topic,
            "player_publishers": len(player_publishers),
            "expected_subscribers": len(input_subscriptions),
            "qos_compatible": matched,
            "probe_subscribers": len(probe_subscriptions),
            "probe_qos_compatible": probe_matched,
        }
        if not matched:
            errors.append(
                f"no QoS-compatible graph match for {route.destination_topic}: "
                f"{player_node} -> {route.subscriber_node}"
            )
        if not probe_matched:
            errors.append(
                f"no QoS-compatible prefix probe match for "
                f"{route.destination_topic}: {player_node} -> {probe_node}"
            )

    if recorder_node:
        for topic in record_topics:
            publishers = [
                item
                for item in node.get_publishers_info_by_topic(topic)
                if endpoint_name(item) != recorder_node
            ]
            recorder_subscriptions = [
                item
                for item in node.get_subscriptions_info_by_topic(topic)
                if endpoint_name(item) == recorder_node
            ]
            matched = compatible_pair(publishers, recorder_subscriptions)
            detail["recorder_topics"][topic] = {
                "publishers": len(publishers),
                "recorder_subscriptions": len(recorder_subscriptions),
                "qos_compatible": matched,
            }
            if not matched:
                errors.append(
                    f"recorder has no QoS-compatible publisher match on {topic}"
                )
    return errors, detail


def wait_for_graph(
    node: Any,
    routes: Sequence[InputRoute],
    player_node: str,
    probe_node: str,
    recorder_node: str | None,
    record_topics: Sequence[str],
    timeout_sec: float,
    stable_cycles: int,
) -> dict[str, Any]:
    import rclpy

    deadline = time.monotonic() + timeout_sec
    stable = 0
    last_errors: list[str] = []
    last_detail: dict[str, Any] = {}
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        last_errors, last_detail = graph_errors(
            node, routes, player_node, probe_node, recorder_node, record_topics
        )
        if last_errors:
            stable = 0
        else:
            stable += 1
            if stable >= stable_cycles:
                last_detail["stable_cycles"] = stable
                return last_detail
        time.sleep(0.1)
    raise RuntimeError("ROS graph handshake timed out: " + "; ".join(last_errors))


def run_handshake(args: argparse.Namespace) -> dict[str, Any]:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosbag2_interfaces.srv import IsPaused, Resume
    from rosgraph_msgs.msg import Clock

    expected_prefix, message_classes = read_expected_prefix(args.bag, args.input_route)
    rclpy.init()
    node = rclpy.create_node(
        f"hesai_bag_start_handshake_{os.getpid()}",
        enable_rosout=False,
        start_parameter_services=False,
    )
    subscriptions = []
    try:
        observed: dict[str, int | None] = {}
        observed_record_wall_ns: dict[str, int] = {}

        def observe(topic: str, message: Any) -> None:
            if topic not in observed:
                observed[topic] = stamp_ns(message)
                observed_record_wall_ns[topic] = time.time_ns()

        # Join the graph before waiting for the remote player services. This
        # gives the paused player's publishers time to discover the prefix
        # probes before the resume edge, including best-effort point clouds.
        for route in args.input_route:
            subscriptions.append(
                node.create_subscription(
                    message_classes[route.source_topic],
                    route.destination_topic,
                    lambda message, topic=route.source_topic: observe(topic, message),
                    qos_profile_sensor_data,
                )
            )
        subscriptions.append(
            node.create_subscription(
                Clock,
                "/clock",
                lambda message: observe("/clock", message),
                qos_profile_sensor_data,
            )
        )

        player_is_paused = node.create_client(
            IsPaused, f"{args.player_node}/is_paused"
        )
        player_resume = node.create_client(Resume, f"{args.player_node}/resume")
        recorder_is_paused = None
        if args.recorder_node:
            recorder_is_paused = node.create_client(
                IsPaused, f"{args.recorder_node}/is_paused"
            )
        deadline = time.monotonic() + args.timeout
        required_clients = [player_is_paused, player_resume]
        if recorder_is_paused is not None:
            required_clients.append(recorder_is_paused)
        while time.monotonic() < deadline and not all(
            client.service_is_ready() for client in required_clients
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        unavailable = [
            client.srv_name
            for client in required_clients
            if not client.service_is_ready()
        ]
        if unavailable:
            raise RuntimeError(f"required rosbag services unavailable: {unavailable}")

        pause_response = call_service(
            node, player_is_paused, IsPaused.Request(), args.service_timeout
        )
        if pause_response is None or not pause_response.paused:
            raise RuntimeError("rosbag player did not start paused")
        if recorder_is_paused is not None:
            recorder_pause_response = call_service(
                node, recorder_is_paused, IsPaused.Request(), args.service_timeout
            )
            if recorder_pause_response is None or recorder_pause_response.paused:
                raise RuntimeError("rosbag recorder is paused")

        probe_node = normalize_node_name(node.get_name(), node.get_namespace())
        graph = wait_for_graph(
            node,
            args.input_route,
            args.player_node,
            probe_node,
            args.recorder_node,
            args.record_topic,
            args.timeout,
            args.stable_cycles,
        )

        # Let the probe subscriptions become visible while playback is still
        # paused. The explicit localizer matches above prevent this probe from
        # satisfying the input readiness contract by itself.
        probe_deadline = time.monotonic() + args.timeout
        while time.monotonic() < probe_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(
                any(
                    endpoint_name(item) == probe_node
                    for item in node.get_subscriptions_info_by_topic(
                        route.destination_topic
                    )
                )
                for route in args.input_route
            ):
                break
        else:
            raise RuntimeError("prefix probe subscriptions did not join the graph")

        pre_resume_deadline = time.monotonic() + args.probe_settle_sec
        while time.monotonic() < pre_resume_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        input_topics = {route.source_topic for route in args.input_route}
        premature = sorted(input_topics.intersection(observed))
        if premature:
            raise RuntimeError(
                f"input messages arrived while player reported paused: {premature}"
            )

        resume_wall_ns = time.time_ns()
        call_service(node, player_resume, Resume.Request(), args.service_timeout)
        resume_deadline = time.monotonic() + args.service_timeout
        paused_after_resume = True
        while time.monotonic() < resume_deadline:
            response = call_service(
                node, player_is_paused, IsPaused.Request(), args.service_timeout
            )
            paused_after_resume = bool(response.paused)
            if not paused_after_resume:
                break
            time.sleep(0.05)
        if paused_after_resume:
            raise RuntimeError("rosbag player remained paused after resume")

        prefix_deadline = time.monotonic() + args.prefix_timeout
        expected_observations = input_topics | {"/clock"}
        while (
            not expected_observations.issubset(observed)
            and time.monotonic() < prefix_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        missing = sorted(expected_observations - set(observed))
        if missing:
            raise RuntimeError(f"playback prefix observation timed out: {missing}")
        expected_stamps = {
            topic: item["accepted_header_stamps_ns"]
            for topic, item in expected_prefix.items()
        }
        prefix_errors = validate_prefix(expected_stamps, observed)
        if prefix_errors:
            raise RuntimeError("; ".join(prefix_errors))

        return {
            "valid": True,
            "player_node": args.player_node,
            "recorder_node": args.recorder_node,
            "paused_before_resume": True,
            "paused_after_resume": False,
            "resume_wall_ns": resume_wall_ns,
            "graph": graph,
            "input_prefix": {
                topic: {
                    **expected_prefix[topic],
                    "exact_first_required": expected_prefix[topic]["type"]
                    != "sensor_msgs/msg/PointCloud2",
                    "expected_first_header_stamp_ns": expected_prefix[topic][
                        "header_stamp_ns"
                    ],
                    "observed_header_stamp_ns": observed[topic],
                    "observed_first_header_stamp_ns": observed[topic],
                    "observed_prefix_index": expected_prefix[topic][
                        "accepted_header_stamps_ns"
                    ].index(observed[topic]),
                    "skipped_count": expected_prefix[topic][
                        "accepted_header_stamps_ns"
                    ].index(observed[topic]),
                    "maximum_skipped_count": len(
                        expected_prefix[topic]["accepted_header_stamps_ns"]
                    ) - 1,
                    "observed_wall_ns": observed_record_wall_ns[topic],
                }
                for topic in sorted(expected_prefix)
            },
            "first_clock_ns": observed["/clock"],
            "first_clock_observed_wall_ns": observed_record_wall_ns["/clock"],
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--bag", type=Path, required=True)
    result.add_argument(
        "--input-route", type=parse_input_route, action="append", required=True
    )
    result.add_argument("--player-node", default="/rosbag2_player")
    result.add_argument("--recorder-node")
    result.add_argument("--record-topic", action="append", default=[])
    result.add_argument("--timeout", type=float, default=30.0)
    result.add_argument("--prefix-timeout", type=float, default=15.0)
    result.add_argument("--service-timeout", type=float, default=5.0)
    result.add_argument("--stable-cycles", type=int, default=3)
    result.add_argument("--probe-settle-sec", type=float, default=1.0)
    result.add_argument("--status-file", type=Path, required=True)
    result.add_argument("--self-test", action="store_true")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    if argv == ["--self-test"]:
        return run_self_test()
    args = parser().parse_args(argv)
    status: dict[str, Any]
    try:
        status = run_handshake(args)
    except Exception as error:  # noqa: BLE001 - fail closed with an audit file
        status = {"valid": False, "error": str(error)}
        args.status_file.parent.mkdir(parents=True, exist_ok=True)
        args.status_file.write_text(
            json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(f"rosbag paused-start handshake failed: {error}", file=sys.stderr)
        return 1
    args.status_file.parent.mkdir(parents=True, exist_ok=True)
    args.status_file.write_text(
        json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print("rosbag paused-start handshake PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
