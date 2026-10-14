#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
#
# Container entrypoint for the ha-ros2-bridge ROS 2 image (tools/Dockerfile.ros2).
#
# Sources ROS 2 and the colcon workspace, renders a ROS 2 parameters file from the
# environment, then runs one of these sub-commands (the first argument):
#
#   launch [launch args]  ros2 launch ros2_ha_bridge bridge.launch.py params_file:=<rendered> ...
#   run [ros args]        ros2 run ros2_ha_bridge bridge_node --ros-args --params-file <rendered> ...
#   test [colcon args]    colcon test in the workspace, then colcon test-result --verbose
#   clock-check [args]    python3 tools/clock_check.py ...
#   <anything else>       exec'd as given, with ROS 2 and the workspace sourced
#
# With no argument at all, "launch" is assumed (the image's CMD).
#
# Environment-driven overrides (each is optional; unset or empty means the node's own default
# or the value in the base file wins). They are written into the rendered parameters file:
#
#   ROS2_HA_BRIDGE_PARAMS_FILE   a base parameters file to start from (mount it into the container)
#   ROS2_HA_BRIDGE_ROBOT_ID      robot.id
#   ROS2_HA_BRIDGE_ROBOT_NAME    robot.name
#   MQTT_HOST                    mqtt.host
#   MQTT_PORT                    mqtt.port (an integer)
#   MQTT_USERNAME                mqtt.username
#   TOPIC_PREFIX                 topic_prefix
#
# ROS2_HA_BRIDGE_MQTT_PASSWORD and ROS2_HA_BRIDGE_COMMAND_SECRET are read by the node itself
# and are never written to the rendered file. The rendered file lands at
# $ROS2_HA_BRIDGE_RENDERED_PARAMS (default /run/ros2_ha_bridge/params.yaml).
# -u comes later: the ROS 2 and colcon setup scripts sourced next read unset variables.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS="${ROS2_HA_BRIDGE_WS:-/ws}"
RENDERED="${ROS2_HA_BRIDGE_RENDERED_PARAMS:-/run/ros2_ha_bridge/params.yaml}"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WS}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS}/install/setup.bash"
fi
# nounset only from here. The setup scripts above test bare $COLCON_TRACE, $COLCON_CURRENT_PREFIX
# and $AMENT_TRACE_SETUP_FILES, which are unset in a fresh container, so sourcing them under
# set -u aborts the entrypoint before any sub-command runs; the official ros_entrypoint.sh uses
# set -e alone for the same reason.
set -u

# Render the parameters file: the base file (if any) with the environment overrides applied
# as nested maps under ros__parameters, which rcl flattens to the dotted parameter names.
# The overrides go to the one node entry a base file names, or to "/**" (every node).
render_params() {
    local out="$1"
    mkdir -p "$(dirname "$out")"
    python3 - "$out" <<'PY'
import os
import sys

import yaml

out = sys.argv[1]
base_path = os.environ.get("ROS2_HA_BRIDGE_PARAMS_FILE", "")
document = {}
if base_path:
    try:
        with open(base_path, encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}
    except OSError as error:
        sys.exit(f"entrypoint: cannot read ROS2_HA_BRIDGE_PARAMS_FILE {base_path!r}: {error}")
    if not isinstance(loaded, dict):
        sys.exit(f"entrypoint: {base_path!r} must be a mapping (a ROS 2 parameters file)")
    document = loaded

overrides = []


def take(name, dotted, convert=str):
    raw = os.environ.get(name, "")
    if raw == "":
        return
    try:
        value = convert(raw)
    except ValueError:
        sys.exit(f"entrypoint: {name}={raw!r} is not a valid value for {dotted}")
    overrides.append((dotted, value))


take("ROS2_HA_BRIDGE_ROBOT_ID", "robot.id")
take("ROS2_HA_BRIDGE_ROBOT_NAME", "robot.name")
take("MQTT_HOST", "mqtt.host")
take("MQTT_PORT", "mqtt.port", int)
take("MQTT_USERNAME", "mqtt.username")
take("TOPIC_PREFIX", "topic_prefix")


def params_of(doc):
    entries = [k for k, v in doc.items() if isinstance(v, dict) and "ros__parameters" in v]
    key = entries[0] if len(entries) == 1 else "/**"
    node = doc.setdefault(key, {})
    if not isinstance(node, dict):
        sys.exit(f"entrypoint: node entry {key!r} in the base file is not a mapping")
    params = node.setdefault("ros__parameters", {})
    if not isinstance(params, dict):
        sys.exit(f"entrypoint: ros__parameters of {key!r} in the base file is not a mapping")
    return params


def set_nested(params, dotted, value):
    # A flat "a.b: x" key in the base file would name the same parameter; drop it first.
    params.pop(dotted, None)
    keys = dotted.split(".")
    cursor = params
    for key in keys[:-1]:
        nxt = cursor.get(key)
        if not isinstance(nxt, dict):
            nxt = {}
            cursor[key] = nxt
        cursor = nxt
    cursor[keys[-1]] = value


if overrides:
    params = params_of(document)
    for dotted, value in overrides:
        set_nested(params, dotted, value)
elif not document:
    document = {"/**": {"ros__parameters": {}}}

with open(out, "w", encoding="utf-8") as handle:
    yaml.safe_dump(document, handle, default_flow_style=False, sort_keys=False)

if overrides:
    shown = ", ".join(f"{dotted}={value!r}" for dotted, value in overrides)
    print(f"entrypoint: rendered {out} with overrides {shown}", file=sys.stderr)
else:
    print(f"entrypoint: rendered {out} with no environment overrides", file=sys.stderr)
PY
}

command="${1:-launch}"
if [ "$#" -gt 0 ]; then
    shift
fi

case "$command" in
    launch)
        render_params "$RENDERED"
        exec ros2 launch ros2_ha_bridge bridge.launch.py "params_file:=${RENDERED}" "$@"
        ;;
    run)
        render_params "$RENDERED"
        exec ros2 run ros2_ha_bridge bridge_node --ros-args --params-file "${RENDERED}" "$@"
        ;;
    test)
        cd "$WS"
        colcon test --event-handlers console_direct+ "$@"
        exec colcon test-result --verbose
        ;;
    clock-check)
        exec python3 "${WS}/tools/clock_check.py" "$@"
        ;;
    *)
        exec "$command" "$@"
        ;;
esac
