#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
#
# Container entrypoint for the ha-ros2-bridge ROS 2 image (tools/Dockerfile.ros2).
#
# Sources ROS 2 and the colcon workspace, renders a ROS 2 parameters file from the
# environment (tools/render_params.py), then runs one of these sub-commands (the first argument):
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
#   ROS2_HA_BRIDGE_PARAM_<NAME>  any other node parameter: <NAME> is the dotted name in upper
#                                case with dots as double underscores, for example
#                                ROS2_HA_BRIDGE_PARAM_WATCHDOG__MODE=inhibit_new or
#                                ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED=true; the value is
#                                converted to the parameter's declared type (tools/README.md)
#
# ROS2_HA_BRIDGE_MQTT_PASSWORD and ROS2_HA_BRIDGE_COMMAND_SECRET are read by the node itself
# and are never written to the rendered file (a ROS2_HA_BRIDGE_PARAM_ form of either is
# refused). An unknown parameter name or a value of the wrong type stops the container before
# the node starts. The rendered file lands at $ROS2_HA_BRIDGE_RENDERED_PARAMS (default
# /run/ros2_ha_bridge/params.yaml); ROS2_HA_BRIDGE_WS (default /ws) is the workspace and
# ROS_DISTRO (default jazzy) the ROS 2 distribution sourced.
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

# Render the parameters file: the base file (if any) with the environment overrides applied.
# The renderer is a separate, importable module so the ROS test suite can drive it; it runs
# after the workspace is sourced because the generic overrides are typed from the node's own
# parameter table (ros2_ha_bridge.parameters).
render_params() {
    python3 "${WS}/tools/render_params.py" "$1"
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
