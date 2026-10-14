# Security policy

This project lets Home Assistant start motion on a robotic arm. Treat anything that can publish to the broker as something that can move the arm. The security model (threat model, the layered defences as implemented, what is not defended, and recommendations per deployment) is in [docs/security.md](docs/security.md); the safety posture is in [docs/safety.md](docs/safety.md).

## Supported versions

The project is pre-release and nothing has been tagged yet. Until `v0.1.0` exists, only the current `main` branch is supported. From the first tag onwards, the latest `0.x` release is supported and earlier releases are not; `CHANGELOG.md` and `scripts/check_versions.py` name the current version.

| Version | Supported |
|---|---|
| `main` (unreleased) | Yes |
| Any tagged release older than the latest | No |

## Reporting a vulnerability

Please do not open a public issue for a security problem.

- Contact: `jonathanmullen1@gmail.com` (the maintainer). Please do not open a public issue for a vulnerability.
- Include: the version or commit, the broker and Home Assistant versions, what you can make the arm do that you should not, and how.
- You will get an acknowledgement within seven days, and a fix or a published mitigation before any public disclosure, agreed with you.

## Scope

In scope: the ROS 2 package (`ros2/ros2_ha_bridge`), the Home Assistant integration (`custom_components/ros2_bridge`) and the contract (`contract/`).

Out of scope: the arm's hardware e-stop, protective stop and collision handling (report those to the arm vendor); the MQTT broker and the Mosquitto add-on themselves (report to their projects; the known add-on ACL finding is described in `docs/security.md`); Home Assistant core.
