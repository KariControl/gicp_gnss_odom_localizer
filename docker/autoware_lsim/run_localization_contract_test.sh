#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -Eeuo pipefail

set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /opt/autoware/setup.bash
source "${GICP_GNSS_ODOM_INSTALL:-/opt/gicp_gnss_odom_localizer}/setup.bash"
set -u

contract_output_dir="${CONTRACT_OUTPUT_DIR:-/output/autoware_localization_contract}"
result_json="$contract_output_dir/result.json"
tf_ownership_json="$contract_output_dir/tf_ownership.json"
launch_log="$contract_output_dir/launch.log"
probe_log="$contract_output_dir/probe.log"
tf_ownership_log="$contract_output_dir/tf_ownership.log"
runner_status="$contract_output_dir/runner_status.txt"
launch_pid=""
failure_reason="contract runner exited before completion"

mkdir -p "$contract_output_dir"
: > "$launch_log"
: > "$probe_log"
: > "$tf_ownership_log"
rm -f "$result_json" "$tf_ownership_json"

annotate_result() {
  local status="$1"
  local reason="$2"
  python3 - "$result_json" "$status" "$reason" "$tf_ownership_json" <<'PY'
import json
import os
from pathlib import Path
import sys

path = Path(sys.argv[1])
status = int(sys.argv[2])
reason = sys.argv[3]
tf_ownership_path = Path(sys.argv[4])
document = {}
if path.is_file():
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        document = {}

if not document:
    document = {
        "schema_version": 1,
        "result": "FAIL" if status else "PASS",
        "failure": reason if status else None,
    }

document.setdefault(
    "environment",
    {
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "autoware_image": os.environ.get("AUTOWARE_IMAGE", "unknown"),
        "localizer_image_id": os.environ.get("LOCALIZER_IMAGE_ID", "unknown"),
        "autoware_pose_instability_detector_version": os.environ.get(
            "POSE_INSTABILITY_DETECTOR_VERSION", "unknown"
        ),
        "autoware_localization_error_monitor_version": os.environ.get(
            "LOCALIZATION_ERROR_MONITOR_VERSION", "unknown"
        ),
        "use_sim_time": False,
    },
)

document["runner"] = {
    "result": "FAIL" if status else "PASS",
    "exit_code": status,
    "launch_alive_after_probe": status == 0,
    "failure": reason if status else None,
}
if tf_ownership_path.is_file():
    try:
        document["tf_ownership"] = json.loads(
            tf_ownership_path.read_text(encoding="utf-8")
        )
    except (json.JSONDecodeError, OSError) as exception:
        document["tf_ownership"] = {
            "result": "FAIL",
            "failure": f"could not read TF ownership evidence: {exception}",
        }
        document["result"] = "FAIL"
        if not document.get("failure"):
            document["failure"] = document["tf_ownership"]["failure"]
if status:
    document["result"] = "FAIL"
    if not document.get("failure"):
        document["failure"] = reason

temporary = path.with_name(f".{path.name}.runner.tmp")
temporary.write_text(
    json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
temporary.replace(path)
PY
}

restore_artifact_owner() {
  if [[ "${HOST_UID:-}" =~ ^[0-9]+$ && "${HOST_GID:-}" =~ ^[0-9]+$ ]]; then
    chown -R "${HOST_UID}:${HOST_GID}" "$contract_output_dir" 2>/dev/null || true
  fi
}

stop_process() {
  local pid="$1"
  local signal="${2:-INT}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill "-$signal" "$pid" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
      for _ in {1..20}; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
      done
    fi
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  fi
  [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_process "$launch_pid" INT
  annotate_result "$status" "$failure_reason" || true
  {
    printf 'result=%s\n' "$([[ "$status" -eq 0 ]] && printf PASS || printf FAIL)"
    printf 'exit_code=%s\n' "$status"
    printf 'failure=%s\n' \
      "$([[ "$status" -eq 0 ]] && printf none || printf '%s' "$failure_reason")"
  } > "$runner_status" || true
  if ((status != 0)); then
    printf '%s\n' '--- Autoware localization contract launch log ---' >&2
    sed -n '1,320p' "$launch_log" >&2 || true
    printf '%s\n' '--- Autoware localization contract probe log ---' >&2
    sed -n '1,320p' "$probe_log" >&2 || true
    printf '%s\n' '--- Dynamic TF ownership probe log ---' >&2
    sed -n '1,320p' "$tf_ownership_log" >&2 || true
  fi
  restore_artifact_owner
  exit "$status"
}
trap cleanup EXIT
trap 'failure_reason="contract runner interrupted"; exit 130' INT TERM

pose_param="$(ros2 pkg prefix --share autoware_pose_instability_detector)"
pose_param+="/config/pose_instability_detector.param.yaml"
error_param="$(ros2 pkg prefix --share autoware_localization_error_monitor)"
error_param+="/config/localization_error_monitor.param.yaml"
export POSE_INSTABILITY_DETECTOR_VERSION="$(
  ros2 pkg xml --tag version autoware_pose_instability_detector
)"
export LOCALIZATION_ERROR_MONITOR_VERSION="$(
  ros2 pkg xml --tag version autoware_localization_error_monitor
)"

ros2 launch pure_odometry_bringup autoware_lsim_localization.launch.py \
  launch_autoware:=false \
  launch_localizer:=false \
  launch_localization_monitors:=true \
  pose_instability_detector_param:="$pose_param" \
  localization_error_monitor_param:="$error_param" \
  launch_vehicle:=false \
  launch_rviz:=false \
  use_sim_time:=false \
  fused_odom_topic:=/test/localization/ekf_odom \
  > "$launch_log" 2>&1 &
launch_pid=$!

required_nodes=(
  /localization_interface_adapter
  /pose_instability_detector
  /localization_error_monitor
)
deadline=$((SECONDS + 30))
while ((SECONDS < deadline)); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    failure_reason="Autoware localization launch exited during startup"
    exit 1
  fi
  nodes="$(ros2 node list --no-daemon --spin-time 1 2>/dev/null || true)"
  ready=true
  for node in "${required_nodes[@]}"; do
    if ! grep -Fxq "$node" <<< "$nodes"; then
      ready=false
      break
    fi
  done
  [[ "$ready" == true ]] && break
  sleep 0.2
done

for node in "${required_nodes[@]}"; do
  grep -Fxq "$node" <<< "${nodes:-}" || {
    printf 'ERROR: required node did not start: %s\n' "$node" >&2
    failure_reason="required node did not start: $node"
    exit 1
  }
done

set +e
timeout 20 ros2 run pure_odometry_bringup tf_ownership_probe.py \
  --owner \
  /localization_interface_adapter,map_frame,map,base_frame,base_link \
  --skip-edge-samples \
  --timeout 10 \
  --output "$tf_ownership_json" \
  > "$tf_ownership_log" 2>&1
tf_ownership_status=$?
set -e
sed -n '1,320p' "$tf_ownership_log"
if ((tf_ownership_status != 0)); then
  failure_reason="dynamic TF ownership probe exited with status $tf_ownership_status"
  exit "$tf_ownership_status"
fi

set +e
timeout 45 ros2 run pure_odometry_bringup autoware_localization_contract_test.py \
  --output "$result_json" \
  > "$probe_log" 2>&1
probe_status=$?
set -e
sed -n '1,400p' "$probe_log"

if ((probe_status != 0)); then
  failure_reason="contract probe exited with status $probe_status"
  exit "$probe_status"
fi
if ! kill -0 "$launch_pid" 2>/dev/null; then
  failure_reason="Autoware localization launch exited after the contract probe"
  exit 1
fi
if ! python3 - "$result_json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
if not path.is_file():
    raise SystemExit(1)
document = json.loads(path.read_text(encoding="utf-8"))
raise SystemExit(0 if document.get("result") == "PASS" else 1)
PY
then
  failure_reason="contract probe did not produce a PASS result document"
  exit 1
fi

failure_reason=""
printf 'PASS: pinned Autoware localization monitor contract\n'
printf 'Evidence: %s\n' "$result_json"
