# Changelog

All notable changes to ha-ros2-bridge are recorded here. The format follows Keep a Changelog, and the project will follow Semantic Versioning from the first tag. Dates are ISO 8601. Each release section is also the body of the GitHub release (`release.yml`).

## [Unreleased]

Phase 0, scaffold and contract. Started 2026-10-13. Nothing in this tree moves an arm.

### Added

- Repository scaffold: Apache-2.0 `LICENSE`, SPDX headers on every Python file, `.editorconfig`, `.gitignore`, CI workflow skeletons for the HA half, the ROS 2 half, the contract and releases, running on empty suites.
- `contract/`: JSON schemas for every payload (schema 1), golden examples and must-reject examples including the malicious-id set, and HMAC test vectors.
- `scripts/check_spdx.py` as the header gate and `scripts/check_versions.py` as the version-agreement check.
- `docs/decisions.md`: the dated decision log seeded with the 20 interview rows, the recorded assumptions, the design panel decisions and the four open questions for phase 0.
- `docs/mqtt-contract.md`: readable contract with every topic, payload example, the command handling order, reply routing and the failure behaviour table.
- `docs/architecture.md`: the two halves, the threading model, the four data flows and the executor plugin interface; measured latency left for phase 5.
- `docs/research/`: the three workflow outputs (landscape research, Isaac Sim and MoveIt 2 research, design panel) and the requirements and plan, with an index.
- `README.md`, `CONTRIBUTING.md`, `SECURITY.md`.

### Not yet

- No ROS 2 node code and no HA integration code: those are phases 1 to 3.
- No measured latency figures (phase 5), no voice sentences (phase 6), no simulator rehearsal (phase 7).
