# Sensors: getting Home Assistant's sensors into ROS 2

Interview decision Q4 asked for two ways of getting sensor events to the robots, because most of the kitchen's sensors are on Zigbee2MQTT and already on the broker, while some may arrive by other protocols. This page describes both paths, the one payload shape the robots can rely on, and how a ROS 2 node reads them. The reverse direction, a robot's joint positions into Home Assistant (added 2026-10-14 towards 0.2.0), is the last section.

| | Fast path | Normalised path |
|---|---|---|
| Source | A Zigbee2MQTT topic (or any MQTT topic) | Any Home Assistant entity, whatever its radio |
| Who publishes | Zigbee2MQTT itself; Home Assistant is not in the loop | The integration, from the `mirror_entities` option |
| Topic | `zigbee2mqtt/{name}` as published | `{prefix}/ha/sensors/{domain}/{object_id}` |
| Payload | Unchanged, whatever the device publishes | The contract's canonical sensor shape |
| Configured on | The robot node (`mirror.topics`) | Home Assistant (options), plus `mirror.topics` on the node |
| Use it for | Raw events where every millisecond matters (Q3) | One schema for everything, and sensors that are not on MQTT |

Derived and timed triggers ("no motion for five minutes") belong to neither path: they stay in Home Assistant automations, which then use the command path (the example at the end).

## Path 1: the fast path (straight from the broker into the node)

The robot node subscribes to the MQTT topics named in its `mirror.topics` parameter and republishes every message it receives on those topics, unchanged, as a ROS 2 `std_msgs/String` on `~/mqtt_in`. Home Assistant receives the same message from the broker at the same moment, so neither side waits for the other; the provisional targets are under 50 ms from broker to node callback and under 200 ms end to end, with the sensor radio expected to dominate.

On the node (`docs/configuration.md`):

| Parameter | Default | Meaning |
|---|---|---|
| `mirror.topics` | `[]` | Topics or filters to mirror: `zigbee2mqtt/kitchen_motion`, `zigbee2mqtt/+`, `ros2_bridge/ha/sensors/#`. A filter MQTT would refuse stops the node at start-up, on purpose: a bad filter is an operator error that should not silence a sensor path. |
| `mirror.availability_suffix` | `/availability` | Each entry's availability sibling (`zigbee2mqtt/kitchen_motion/availability`) is subscribed too and feeds `source_available`. A filter ending in `#` already covers its siblings and gets no extra subscription. Empty disables availability tracking. |
| `mirror.ros_topic` | `~/mqtt_in` | The ROS 2 topic the envelopes go out on; `~` is the node's name, so with the default node name it is `/bridge_node/mqtt_in`. |

```yaml
/**:
  ros__parameters:
    mirror:
      topics:
        - zigbee2mqtt/kitchen_motion
        - zigbee2mqtt/kitchen_door
```

Every message, data or availability, becomes one JSON envelope with these keys, always present and always in this order:

```json
{
  "topic": "zigbee2mqtt/kitchen_motion",
  "payload": "{\"occupancy\":true,\"battery\":97,\"linkquality\":120}",
  "encoding": "utf-8",
  "retain": false,
  "source_available": true,
  "rx_ts_ms": 1791892800123
}
```

- `payload` is the message as received; `encoding` is `utf-8`, or `base64` with the bytes base64-encoded when they are not valid UTF-8.
- `retain` is the MQTT retain flag of the delivery, so a consumer can tell a live event from the retained value replayed on subscribe.
- `source_available` is the last availability seen on the topic's sibling (Zigbee2MQTT's `{"state":"online"}` and the plain word `online` are both understood), or `null` when no sibling message has been seen. An availability message is mirrored as an envelope too, so a consumer learns that a sensor went offline without waiting for a data message that will not come; an empty retained availability resets it to `null`.
- `rx_ts_ms` is the epoch millisecond at which the node received the message.

Nothing on this path reformats the device's own payload: a Zigbee2MQTT motion sensor arrives as Zigbee2MQTT publishes it, so a consumer has to know that device's keys (`occupancy` for one vendor, `motion` for another). That is the price of the latency, and the reason the second path exists.

## Path 2: the normalised path (any entity, one schema)

The integration republishes every entity named in its "Mirrored entities" option (`mirror_entities`) to `{prefix}/ha/sensors/{domain}/{object_id}`, retained at QoS 0 with no expiry: once when the entry starts, and again on every state change of the entity. Retained, so a node started later sees the current value at once.

How it behaves:

- `unavailable` and `unknown` become `"state": null` with `"available": false`, so the robot side never mistakes a dead sensor for a quiet one.
- `attributes` is filtered to the allowlist below, so large or private attributes never leave Home Assistant.
- Nothing is published while Home Assistant's MQTT client is disconnected; the backlog is published when it reconnects, and every mirrored entity is republished then.
- Every topic published is recorded in the integration's store. When an entity leaves the option, or the prefix changes, the topics that are no longer produced are cleared with an empty retained publish at the next start; deleting the entry clears all of them. A stale retained value therefore never outlives the option that produced it.
- An entity whose domain or object id cannot form a topic segment (anything outside `a-z`, `0-9` and `_`) is skipped with a warning in the log.

The canonical payload, from `docs/mqtt-contract.md`:

```json
{
  "schema": 1,
  "entity_id": "binary_sensor.kitchen_motion",
  "domain": "binary_sensor",
  "object_id": "kitchen_motion",
  "state": "on",
  "available": true,
  "device_class": "motion",
  "unit": null,
  "attributes": {"friendly_name": "Kitchen motion"},
  "last_changed": "2026-10-13T12:00:00.123456+00:00",
  "ts": 1791892800123
}
```

| Field | Meaning |
|---|---|
| `schema` | Always 1. |
| `entity_id`, `domain`, `object_id` | The Home Assistant entity, split so the topic can be rebuilt. |
| `state` | The entity's state as a string, or `null` when unavailable or unknown. |
| `available` | `false` exactly when `state` is `null`. |
| `device_class`, `unit` | The entity's device class and unit of measurement, or `null`. |
| `attributes` | The allowlisted attributes only. |
| `last_changed` | Home Assistant's `last_changed` for the state, ISO 8601 with offset, or `null`. |
| `ts` | Epoch milliseconds at which the integration built the message. |

The attribute allowlist (`SENSOR_ATTRIBUTE_ALLOWLIST` in `sensors_out.py`); anything not listed is dropped:

| Group | Attributes |
|---|---|
| Identity and presentation | `friendly_name`, `device_class`, `state_class`, `unit_of_measurement`, `icon` |
| Device health | `battery`, `battery_level`, `linkquality`, `last_seen` |
| Binary sensing | `occupancy`, `motion`, `contact`, `tamper` |
| Environment | `illuminance`, `illuminance_lux`, `temperature`, `humidity`, `pressure`, `co2`, `voc` |
| Electrical | `voltage`, `current`, `power`, `energy` |
| Lights | `brightness`, `color_temp_kelvin`, `rgb_color`, `color_mode`, `effect` |
| Covers, fans and positions | `current_position`, `current_tilt_position`, `position`, `percentage`, `preset_mode`, `speed` |
| Climate | `hvac_action`, `hvac_mode`, `current_temperature`, `target_temperature`, `fan_mode` |
| Selects and numbers | `options`, `min`, `max`, `step`, `mode` |

Attribute values that are not plain JSON types are converted: mappings and lists recursively, anything else to its string form.

## Reading them on the ROS 2 side

The normalised topics reach ROS 2 through the same mirror as the fast path. Add the sensor filter to the node's `mirror.topics`:

```yaml
/**:
  ros__parameters:
    mirror:
      topics:
        - zigbee2mqtt/kitchen_motion
        - ros2_bridge/ha/sensors/#
```

Each canonical message then arrives on `~/mqtt_in` wrapped in the envelope, with the canonical JSON as the `payload` string. Home Assistant publishes no availability sibling for these topics, so `source_available` is `null` for them; the sensor's own availability is the `available` field inside the payload.

To watch it from a shell:

```text
ros2 topic echo /bridge_node/mqtt_in
```

A minimal subscriber that unwraps both layers and branches on the topic:

```python
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SensorListener(Node):
    def __init__(self) -> None:
        super().__init__("sensor_listener")
        self.create_subscription(String, "/bridge_node/mqtt_in", self.on_envelope, 10)

    def on_envelope(self, msg: String) -> None:
        envelope = json.loads(msg.data)
        if envelope["encoding"] != "utf-8":
            return
        topic = envelope["topic"]
        if topic.startswith("ros2_bridge/ha/sensors/"):
            sensor = json.loads(envelope["payload"])
            if not sensor["available"]:
                return
            self.get_logger().info(f"{sensor['entity_id']} is {sensor['state']}")
        elif topic.startswith("zigbee2mqtt/"):
            raw = json.loads(envelope["payload"])
            self.get_logger().info(f"{topic}: {raw}")


def main() -> None:
    rclpy.init()
    rclpy.spin(SensorListener())


if __name__ == "__main__":
    main()
```

The executor plugin can subscribe in the same way; the mirror is just another ROS 2 topic. Nothing on it is a command: the node's authorisation path (`docs/mqtt-contract.md`, command handling) is untouched by anything that arrives through the mirror.

## Derived and timed triggers stay in Home Assistant

The interview's own first scenario (Q1): no motion in the kitchen for five minutes turns the lights off and sends every available robot back to its home pose. The "for five minutes" is a Home Assistant judgement, so this is an automation on the command path, not a mirrored sensor. From `docs/examples/automations.yaml`:

```yaml
- alias: "Kitchen idle: lights off and arms home"
  description: Five minutes without motion turns the kitchen lights off and resets every robot.
  mode: single
  triggers:
    - trigger: state
      entity_id: binary_sensor.kitchen_motion
      to: "off"
      for:
        minutes: 5
  actions:
    - action: light.turn_off
      target:
        area_id: kitchen
    - action: ros2_bridge.get_robots
      response_variable: robots
    - repeat:
        for_each: >-
          {{ robots.robots | selectattr('available') | map(attribute='robot_id') | list }}
        sequence:
          - action: ros2_bridge.send_task
            data:
              robot_id: "{{ repeat.item }}"
              task: reset
              wait: none
            continue_on_error: true
```

The robots are discovered through `get_robots`, so nothing names a robot; each send is `wait: none` because the outcome is reported by the speak-back automation in the same file; and `continue_on_error` keeps the loop going when one robot refuses (for example `rejected_busy`). Replace `binary_sensor.kitchen_motion` and `area_id: kitchen` with your own.

The same motion sensor can also be on the fast path at the same time, so that a ROS 2 node reacts to each raw motion event in tens of milliseconds while Home Assistant keeps the five-minute judgement. The two paths coexist; that was the point of Q3.

## The joints path (robot positions into Home Assistant)

The reverse direction, added 2026-10-14 towards 0.2.0 (`docs/decisions.md`, revising interview decision Q17 for a dashboard that shows each arm's articulation): a robot whose announce carries a `joints` block publishes its joint positions on `{prefix}/{robot_id}/joints`, and Home Assistant shows them as one sensor per joint. `docs/mqtt-contract.md` has both payloads; this section is about what arrives in Home Assistant and how to keep it out of the recorder when it is not wanted. `docs/dashboard.md` is the dashboard built on it.

| | |
|---|---|
| Topic | `{prefix}/{robot_id}/joints`, QoS 0, never retained, MQTT 5 message expiry 5 s. Only from a node whose announce carries `joints`. |
| Rate | At most `telemetry.joints_rate_hz` messages per second (default 2; 0 disables the path and the announce omits `joints`), only when a position moved by more than `telemetry.joints_min_delta` since the last publication (default 0.5 for degrees, 0.01 for radians), plus once on every (re)connect and announce. |
| Entities | One sensor per announced joint: unique id `{robot_id}_joint_{index}` (index from 0), name "Joint {index + 1} {joint name}", unit `°` or `rad` as announced, one decimal in the display, no state class (so nothing enters long-term statistics). Attributes `index`, `joint_name`, `limit_min` and `limit_max` (null when the robot announces no limits). |
| Availability | Unavailable with the robot, by the same rule as every other entity of the device. A stalled stream alone changes nothing: the sensor keeps its last value. |
| Lifetime | Created on an announce with `joints`; removed when a later announce drops the joint or the whole key. |
| Ordering | A payload whose `seq` regresses within a session is dropped, as is one whose number of positions differs from the announced names. |
| Diagnostics | The latest positions and the joints `seq` per robot. |

Nothing on this path is a safety signal (`docs/safety.md`): a position is a display value with a five-second expiry, never a command input.

### Keeping the joint sensors out of the recorder

The sensors are enabled by default so that the dashboard works without setup. An arm in motion produces up to `rate_hz` state changes per joint per second, each of which the recorder writes to the database and keeps for its purge window (ten days by default). To keep them out of history, exclude them in `configuration.yaml` (the recorder has no UI for this) and restart Home Assistant:

```yaml
recorder:
  exclude:
    entity_globs:
      - sensor.*_joint_*
```

The glob matches every joint sensor of every robot; `sensor.left_arm_joint_*` narrows it to one robot. With the exclude in place the sensors still show live values and the gauges of `docs/examples/dashboard.yaml` still move, but its history graph of the joints stays empty and the logbook never lists them. Disabling individual joint sensors on the device page is the other option: a disabled sensor is neither recorded nor shown.
