# Security

This project lets Home Assistant start motion on a robotic arm. Treat anything that can publish to the broker as something that can move the arm, and read this page before deploying.

## Reporting a vulnerability

Please do not open a public issue for a security problem.

- Contact: `security@example.invalid` (placeholder; the maintainer will replace this with a real address or a GitHub private vulnerability report link before the first release).
- Include: the version or commit, the broker and HA versions, what you can make the arm do that you should not, and how.
- You will get an acknowledgement within seven days and a fix or a published mitigation before any public disclosure, agreed with you.

Hardware e-stop and the arm controller's own protective functions are out of scope for this project; report those to the arm vendor.

## The authorisation model in plain words

The bridge does not trust the broker to decide who may command a robot. The ROS 2 node checks every command itself, in a fixed order, and the first failing check wins:

1. **Retained commands are dropped.** Commands are published without the retain flag. If one arrives with the flag set (someone planted it, or it was left over), the node drops it, counts it, and clears the retained copy from the broker. A retained command can never replay a motion after a restart.
2. **Oversize messages are dropped unparsed.** 16 KiB is the limit.
3. **The command id must be a UUID4.** A command whose id contains a slash, a wildcard, or anything else that is not a UUID is dropped without a reply, because the reply topic is built from the id and there is no safe topic to answer on.
4. **Duplicates are answered, not re-run.** A command id the node has already answered gets the stored ack again, byte for byte, from a store that survives a node restart for as long as the command could still be fresh. This check runs the moment the id is known, before the schema, signature and freshness checks, because the contract promises one distinct ack per id: if a later check could still produce a different ack for a known id, an unsigned or deliberately malformed replay of an accepted command could overwrite the retained `accepted` ack, and republishing the stored bytes gives a replayer nothing it could not already read from the retained topic. QoS 1 redelivery and a crash-loop can never start the same task twice.
5. **The payload must match the schema** and name this robot.
6. **If a shared secret is configured, the signature must verify.** The HMAC-SHA256 is computed over the topic plus the canonical JSON, so a valid command for one robot cannot be replayed at another, and the comparison is constant-time. The secret is read from a ROS 2 parameter or the `ROS2_HA_BRIDGE_COMMAND_SECRET` environment variable, never from a launch argument. This layer is **off by default** (interview decision Q15) and switched on with one option on each side.
7. **Stale and future-dated commands are rejected.** A command carries `issued_at` and `max_age_ms`; the node accepts it only inside that window plus a 2 s clock skew tolerance, and caps the window at 30 s whatever the sender asked for. This needs both machines on NTP; the ack carries the measured skew and HA raises a repair issue when it drifts.
8. **Only announced tasks run**, with parameters validated against the announced schema. The operator's `tasks.allow` parameter narrows the catalogue further.
9. **One motion at a time.** A motion task is refused while the robot is busy, in error or inhibited. Control tasks (stop, pause, resume) are accepted in any state.

On the Home Assistant side, commands can only be sent through the integration's own service, buttons and intents, which run with the calling user's HA permissions; the integration never exposes a raw publish. `mqtt.publish` itself is administrator-only in HA since 2026.10.

What this model does **not** do: it does not stop an attacker who already has valid broker credentials from sending well-formed, fresh, allowlisted commands unless the HMAC secret is configured. Switch the secret on wherever the broker is shared with devices or people you do not fully trust. Use TLS on the ROS 2 side (`mqtt.tls.*`) wherever the network is shared Wi-Fi.

## The Mosquitto add-on ACL finding

Research on 2026-10-13 found that the stock Home Assistant Mosquitto add-on 7.1.1 (Mosquitto 2.1.2 with mosquitto-go-auth 3.0.0) does not enforce an ACL file placed in its customize folder: home-assistant/addons issue #4571, open since 2026-04-17, with pull request #4831 unmerged at the time of writing. The consequence is that any client that can authenticate to the add-on at all (Zigbee2MQTT, ESPHome devices, any Home Assistant user account) can publish to `ros2_bridge/+/cmd`.

This finding comes from the open issue and was not reproduced on a live add-on by this project; it is to be verified on the live add-on before anything relies on it. It is the reason authorisation lives in the node rather than in broker ACLs, and the reason the HMAC option exists.

Options for adopters who want broker-level enforcement as well:

- Switch on the HMAC shared secret (recommended for any shared broker).
- Run a standalone Mosquitto (or another MQTT 5 broker) that enforces ACLs, with a dedicated user for the bridge scoped to the topic prefix.
- Pin the add-on to a version that enforced the ACL file, at the cost of missing later fixes.
- Watch the issue and re-enable the customize ACL once the fix ships.

Broker ACLs, where they work, are documented as an extra layer; they are not a substitute for the node's own checks.

## Secrets and configuration

- No file in this repository contains a real host, address, username, password or key. Example configurations use placeholders and environment variables.
- The ROS 2 side reads the MQTT password from `ROS2_HA_BRIDGE_MQTT_PASSWORD` and the command secret from `ROS2_HA_BRIDGE_COMMAND_SECRET` when the matching parameter is empty. Launch files pass these through and never put them on the command line.
- The HA side inherits HA's own MQTT connection and stores the optional shared secret in the config entry; diagnostics redact it.
- `tools/.env` is ignored by git; `tools/.env.example` holds placeholders only.

## Safety boundary

Nothing on MQTT or in Home Assistant is an emergency stop. The bridge can send `stop` and, in `stop_and_hold` watchdog mode, can ask the executor to hold when HA's heartbeat disappears, but the physical e-stop, protective stop and collision handling belong to the arm controller. By default (Q16) the loss of Home Assistant has no effect on the arms at all.
