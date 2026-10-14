# Security

This page is the security model of ha-ros2-bridge: who can make an arm move through the bridge, what the code does about it, what it deliberately does not do, and what to configure at each kind of site. Reporting a vulnerability is covered by the root [SECURITY.md](../SECURITY.md); the safety posture (what the bridge promises about motion) is in [safety.md](safety.md).

Everything below describes the code as it stands in `ros2/ros2_ha_bridge` and `custom_components/ros2_bridge`; each defence names its tests in [failure-modes.md](failure-modes.md).

## Threat model

### What is at stake

- **Starting motion.** A command on `{prefix}/{robot_id}/cmd` that passes the node's checks runs a task on the arm.
- **Lying about outcomes.** Acknowledgements, results, state and availability drive dashboards, speech and automations.
- **Secrets.** The broker password and, when configured, the shared signing secret.

### Who can publish on the broker

Anyone who can authenticate to the MQTT broker can publish on any topic unless the broker enforces an access control list. On a Home Assistant installation that is typically: Home Assistant itself, Zigbee2MQTT, every ESPHome or Tasmota device with the broker credentials, every add-on that uses the Supervisor's MQTT service (which share one `addons` account), and every person who knows the broker password. With the stock Mosquitto add-on, the finding below means all of them can publish commands.

### The Mosquitto add-on ACL finding

Research on 2026-10-13 (`docs/research/2026-10-13-landscape-research.json`) found that the stock Home Assistant Mosquitto add-on 7.1.1 (Mosquitto 2.1.2 with mosquitto-go-auth 3.0.0) does not enforce an `acl_file` placed in its customize folder: home-assistant/addons issue #4571, open since 2026-04-17 (issue #4721 reports the same), with pull request #4831 unmerged at the time of writing. The research's own words for the cause are "go-auth superuser bypass via nginx returning 200". The consequence is that any client that can authenticate to the add-on at all can publish to `ros2_bridge/+/cmd`.

This finding comes from the open issue and was **not reproduced on a live add-on by this project**. It is to be verified on the live add-on before anything relies on it, in either direction. It is the reason authorisation lives in the node rather than in broker ACLs, and the reason the HMAC option exists; the design panel recorded both on 2026-10-13 (`docs/decisions.md`, "Authorisation in the node").

### Attacker capabilities considered

- Publish arbitrary MQTT messages on any topic, including retained ones.
- Capture a genuine command and replay it later, or at another robot.
- Flood the command topic.
- Read every topic, including commands (the shared secret never travels on the wire; a signature does).
- Join the ROS 2 DDS domain on the robot's network and read node parameters.

Not considered: an attacker with a shell on the Home Assistant host or the ROS 2 host; a compromised broker (see "What is not defended").

## Layered defences, as implemented

The node applies these to every delivery on its command topic, on its main thread, in the order below; the first failing check wins and the acknowledgement names it (`docs/mqtt-contract.md`, "Command handling on the node").

1. **Retained commands are dropped.** The node subscribes to `cmd` with MQTT 5 Retain As Published, so a retained command is delivered with its RETAIN flag set whether it was planted while the node was connected or before it connected. It is dropped unparsed, counted `dropped_retained`, and the topic is cleared with an empty retained publish. The node's own clearing echo is ignored. A retained command therefore cannot replay a motion after a restart.
2. **Size limits.** A payload over `mqtt.max_payload_bytes` (16 KiB) is dropped unparsed on the network thread and counted `dropped_oversize`. The inbound queue between the network thread and the node is bounded (`mqtt.inbound_queue_max`, 256); a full queue drops the newest message and counts `dropped_queue_full`. The integration applies 16 KiB to replies and 64 KiB to announces, measured on the bytes received.
3. **The command id must be a lower-case UUID4 before any reply is built.** The reply topics are built from the id. An id containing `/`, `+`, `#`, a wrong case, a wrong length or anything else that fails the pattern, or a payload that is not JSON, is dropped with no acknowledgement because there is no safe topic to answer on; it is counted `dropped_invalid_id` and warned about at most once per `logging.warn_interval_s` (10 s by default; 0 warns on every event). Nothing reaches paho's publish with a wildcard in it: `MqttLink.publish` validates every outbound topic as well.
4. **Duplicates are answered, not re-run (persisted dedupe).** The moment the id is known, and before the schema, signature and freshness checks, the node looks the id up in its dedupe store (`command.dedupe_window_s`, 300 s, at least `reply.ack_expiry_s`; `command.dedupe_size`, 1024). A known id republishes the stored acknowledgement byte for byte, with the retain flag it was first published with, and stops. Accepted acknowledgements are persisted to `{data_dir}/{robot.id}/dedupe.json` (atomic rewrite, at most once per `command.dedupe_write_interval_s`, 1 s by default, flushed at shutdown; an empty `data_dir` keeps the store in memory only) and reloaded at start for ids younger than `command.max_age_cap_ms` plus `command.clock_skew_tolerance_ms`, exactly the window in which a publisher-side QoS 1 resend could still pass the freshness check. Because the lookup runs first, an unsigned, malformed or stale replay of an accepted id can never overwrite the retained `accepted` acknowledgement, and republishing the stored bytes gives a replayer nothing it could not already read from the retained topic.
5. **Schema and robot id.** The payload must be schema 1 with correctly typed required fields, and its `robot_id` must equal this node's id (the topic segment is already the node's own by subscription); otherwise `rejected_malformed`.
6. **Optional topic-bound HMAC.** When a secret is configured, the signature must verify or the acknowledgement is `rejected_unauthorised` (warned at most once per `logging.warn_interval_s`). The definition, identical in both `protocol.py` copies and checked against `contract/hmac_vectors.json` by both suites:

   ```text
   canonical = json.dumps(payload_without_sig, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
   sig       = hex(HMAC-SHA256(secret, topic + "\n" + canonical))
   ```

   The topic is bound into the signature, so a valid command captured for one robot cannot be replayed at another robot or under another prefix. The node compares with `hmac.compare_digest` (constant time). Without a secret the node ignores any `sig` present. Off by default (interview Q15). To switch it on:

   - **ROS 2 side:** export `ROS2_HA_BRIDGE_COMMAND_SECRET` in the environment the launch file is started from (preferred), or set the `command.secret` parameter. With a secret the node announces `capabilities.hmac_required: true` and refuses unsigned commands.
   - **Home Assistant side:** enter the same value in the integration's **Shared secret** field (the user step or the discovery confirm step when adding it, or the options flow afterwards). When set, every command is signed; when empty, commands are unsigned and a robot that announces `hmac_required` opens the `hmac_rejected` repair.
   - The two values must be identical, or both empty. During a rotation the robot refuses commands with `rejected_unauthorised` until both sides agree; nothing is accepted unsigned by mistake. The placeholder `example-shared-secret` in `docs/mqtt-contract.md` and the contract vectors exists so the canonicalisation can be checked by hand; never deploy it.

7. **Freshness window with skew tolerance.** A command carries `issued_at` and `max_age_ms` (the integration's `command_max_age_ms` option, default 5000; a service call may pass its own). The node accepts it only when `-skew <= now - issued_at <= min(max_age_ms, command.max_age_cap_ms) + skew`, with `command.max_age_cap_ms` 30000 and `skew = command.clock_skew_tolerance_ms` 2000 by default; otherwise `rejected_stale` or `rejected_future`, with the measured `clock_skew_ms` in the acknowledgement so Home Assistant can open the `clock_skew` repair. `now` is taken when the command is processed on the main thread, not when the network thread received it, so a command that aged in a stalled node's queue is refused too. The integration also publishes every command with an MQTT 5 message expiry of `ceil(max_age_ms / 1000)` seconds, so a broker does not hold it for longer than that; if Home Assistant's MQTT session is not MQTT 5 the expiry is dropped and the `mqtt_v5_required` repair opens, and the node's freshness rule remains the backstop. This layer needs both machines on NTP.
8. **Allowlist and parameters.** Only tasks in the announced catalogue run, and the catalogue is the executor's task list intersected with the operator's `tasks.allow` parameter; otherwise `rejected_unknown_task`. Parameters are validated against the task's announced schema subset on the node (and on the integration before publishing); otherwise `rejected_invalid_params`. Any requested `timeout_s` is clamped to `command.task_timeout_cap_s` (600), and so is the announced catalogue.
9. **State gating.** A motion task is refused while the robot is busy (`rejected_busy`), in error (`rejected_error`) or inhibited by the watchdog (`rejected_inhibited`); control and query tasks pass in any state. The task manager applies the same gate again, so the single motion slot can never be taken twice.
10. **One `executor.start` per accepted id, and a bounded retained store.** The acknowledgement is committed to the dedupe store only after the executor accepted the task. Rejected acknowledgements beyond `reply.retained_rejections_per_minute` (30) are published non-retained, and stay non-retained when redelivered, so a flood of bad commands cannot fill the broker's retained store. A node restart connects with `clean_start`, so a broker-queued backlog of commands is never drained.

On the Home Assistant side:

- **No raw publish.** Commands can be raised only through the `send_task` and `cancel_task` actions, the task and cancel buttons and the two Assist intents, all of which run with the calling user's Home Assistant permissions and record the user id, context id and (for voice) device id in the command's `source`. The integration never exposes a publish of arbitrary bytes, and `mqtt.publish` itself is administrator-only in Home Assistant since 2026.10. The node records `source` in acknowledgements and logs but never interprets it.
- **Validation before anything leaves.** Every target of a multi-robot call is validated before the first publish; a command the contract refuses (over 16 KiB once encoded, or parameters JSON cannot represent) is reported as `invalid_params` and is never tracked, persisted or published. `max_pending_per_robot` (16) bounds outstanding commands per robot.
- **Inbound hygiene.** Every wildcard handler validates the robot segment (`ha` is reserved and discarded, anything failing `^[a-z][a-z0-9_]{0,31}$` is counted), checks that the payload's `robot_id` and `id` match the topic, drops a state whose `seq` regresses within a session, and never lets a bad payload or a handler bug reach the subscription callback.

### Secrets

- **Environment variables first.** The node reads `ROS2_HA_BRIDGE_MQTT_PASSWORD` when `mqtt.password` is empty and `ROS2_HA_BRIDGE_COMMAND_SECRET` when `command.secret` is empty. The launch files pass them through and never put them on a command line.
- **Undeclared parameters.** The two secret parameters are declared (so a params file can still set them), read once and undeclared immediately, with a WARN that names the environment variable to use instead. The reason: a declared parameter sits in the parameter server, where any participant on the DDS domain could read it with `ros2 param get`. They are the only parameters not declared read-only, because rclpy refuses to undeclare a read-only parameter.
- **`repr=False`.** `MqttSettings.password`, `CommandSettings.secret`, `MqttLinkConfig.password` and the integration's `HubOptions.shared_secret` are dataclass fields excluded from `repr`, so a logged or raised settings object never shows a secret. `describe_settings` masks them in the node's start-up log (naming the variable they came from), and the executor plugin's parameter view (`BridgeContext.parameters`) excludes them altogether.
- **Diagnostics.** The integration's diagnostics download redacts `shared_secret` and `user_id` wherever they appear and reports only `secret_configured: true/false`.
- **Where the Home Assistant secret lives.** In the config entry (Home Assistant's `.storage`), under Home Assistant's own file protection; anyone with access to the configuration directory can read it, as with every other integration's credentials.
- **The repository.** No file names a real host, address, user name, password or key; examples use placeholders and the two environment variables. `tools/.env` is ignored by git and `tools/.env.example` holds placeholders only.

### TLS

The ROS 2 side connects with TLS when `mqtt.tls.enabled` is true (set `mqtt.port` to the broker's TLS port as well). `mqtt.tls.ca_cert` names the CA certificate that signed the broker's certificate (empty uses the system trust store), `mqtt.tls.certfile` and `mqtt.tls.keyfile` enable client-certificate authentication, and `mqtt.tls.insecure` skips host name verification for a self-signed test certificate only; it does not disable encryption. A TLS configuration paho refuses (a missing file, for instance) is a fatal start-up error. The Home Assistant side inherits the MQTT integration's own TLS settings and asks for none of its own.

## What is not defended

- **A trusted client with the secret.** Anyone who holds the shared secret (or, without one, anyone who can authenticate to the broker) and publishes a well-formed, fresh, allowlisted command is indistinguishable from Home Assistant. The node's checks establish that a command is recent, intact, for this robot and permitted; they do not establish who sent it beyond the secret.
- **Two Home Assistant instances on one broker.** Both would heartbeat and both could command every robot; `instance_id` in the heartbeat and in each command's `source` makes it visible in the diagnostics, but nothing prevents it (there is no origin allowlist).
- **A compromised broker.** It can drop, delay and reorder; replay within the freshness window (bounded by the dedupe store to one execution per id); forge acknowledgements, results, state and availability, because replies are not signed; and publish tombstones that remove a robot's device from Home Assistant. None of that moves the arm, but it can mislead automations and people. A broker compromise is a host compromise and is out of this model.
- **A compromised Home Assistant.** It has the full authority the allowlist grants, by design.
- **The ROS 2 host and the DDS domain.** This project configures no DDS security. The Docker rig keeps discovery inside each container (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), which is a rig convenience, not a defence for a real deployment.
- **Denial of service.** The size, queue, retained-rejection and pending caps bound the damage a flood does to the broker and the node's memory; they do not stop the node's main thread being kept busy rejecting commands, and a flood on the fast sensor path is only dropped when the queue fills.
- **Unauthenticated sensor data.** The fast-path mirror and the canonical `{prefix}/ha/sensors/...` topics are not signed. Anyone who can publish on the broker can publish a false "room empty". An executor must not take a decision that puts a person at risk from mirrored data alone; see [safety.md](safety.md).
- **Clock trust.** The freshness rule trusts both machines' NTP. A machine with a wrong clock refuses genuine commands (fail closed) rather than accepting stale ones, and the `clock_skew` repair says so.
- **The dedupe write window.** A node that crashes within `command.dedupe_write_interval_s` (1 s by default) of accepting a command may lose that id from the file; a publisher-side resend inside the freshness window would then run the task again. Judged acceptable because the freshness window bounds it (design risk list, 2026-10-13) and recorded in [failure-modes.md](failure-modes.md); a site that wants no window sets the interval to 0, at the cost of one file rewrite per accepted command.

## Recommendations per deployment

| Site | What to do |
|---|---|
| Every site | Both machines on NTP. One `robot.id` per robot with the derived MQTT client id. `tasks.allow` narrowed to what the site needs. Secrets in the two environment variables, never in a params file or a launch argument. Never publish anything retained on a `cmd` topic. Test with the fake executor first. |
| Stock Mosquitto add-on, wired LAN, one household (the set-up the project was developed against; decisions Q14 and Q15) | The node's own checks are the authorisation layer. Reusing the existing MQTT user is acceptable for testing, as decided; for daily use with real arms, switch the shared secret on anyway, because every other MQTT client in the house can otherwise publish commands. Verify the ACL finding on the live add-on and record the result in `docs/decisions.md`. |
| A broker that enforces ACLs (standalone Mosquitto, or the add-on once the fix ships) | Create a dedicated MQTT user for the bridge nodes: publish rights on `{prefix}/{robot_id}/#` only (the node publishes its announce, availability, state, heartbeat, acks, results and the empty retained clear of its own `cmd` topic there) and subscribe rights on `{prefix}/{robot_id}/cmd`, `{prefix}/ha/#`, `homeassistant/status` and the mirrored sensor topics. Home Assistant's own MQTT user keeps whatever it already has for the rest of the house; what the integration needs is subscribe on `{prefix}/#` and publish on `{prefix}/+/cmd`, `{prefix}/ha/#` and, for the tombstones it publishes when a robot's device is deleted, `{prefix}/+/announce`, `{prefix}/+/state` and `{prefix}/+/availability`. Keep the node's checks as they are; ACLs are an extra layer, not a substitute. |
| Shared broker (other people, tenants, or devices you do not fully trust) | Switch the shared secret on both sides. Consider a dedicated user as above. Watch the `hmac_rejected` repair after any secret change. |
| Wi-Fi anywhere on the path between Home Assistant, the broker and the robot | TLS on the ROS 2 side (`mqtt.tls.*`) and on Home Assistant's MQTT integration; a proper CA rather than `mqtt.tls.insecure`. The research's own words: cybersecurity now counts as safety, so use authenticated MQTT and TLS if Wi-Fi is shared. |
| Add-on users who want broker-side enforcement today | Pin the add-on at the last version that enforced the ACL file (the research names 6.5.2), at the cost of missing later fixes; or run a standalone MQTT 5 broker; or watch issue #4571 and re-enable the customize ACL once PR #4831 or an equivalent ships. |
| Assistive-care deployments | All of the above, plus the checklist in [safety.md](safety.md). The outcome events and the `problem` sensor must reach a person. |
