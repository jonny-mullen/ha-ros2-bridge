# Research record

This folder holds the research and design material that ha-ros2-bridge was built from, kept in the repository so that every design choice can be traced to a source. The three JSON files are verbatim outputs of read-only research workflows (web and GitHub sweeps plus synthesis); they are not hand-edited apart from two mechanical changes (the development account's local repository path is replaced by `<repo root>`, and a final newline is appended so the files meet `.editorconfig`), and every factual claim in them carries its source URL inside the file. The Markdown file is the human-written requirements and plan that the interview produced.

All four files are dated 2026-10-13.

| File | Date | What it is |
|---|---|---|
| `2026-10-13-requirements-and-plan.md` | 2026-10-13 | The interview record (20 questions, asked one at a time, with the decision each produced), the assumptions recorded instead of asked, verified facts about the target Home Assistant, condensed research findings, the recommendation, the condensed design, the delivery phases with done criteria, the verification plan and the top risks. The decision log in `docs/decisions.md` is seeded from this file. |
| `2026-10-13-landscape-research.json` | 2026-10-13 | Workflow output: an eleven-agent, 924-tool-call read-only web sweep of Home Assistant to ROS 2 integration options. Covers ROS 2 side MQTT bridges, HA-side architecture choices (add-on, custom integration, plain MQTT discovery, AppDaemon), prior art, safety and reliability patterns, and version facts, followed by a synthesis and a critique listing gaps and contradictions. 209 source URLs inside (`result.synthesis.all_sources`). |
| `2026-10-13-isaac-sim-moveit.json` | 2026-10-13 | Workflow output: a four-agent, 233-tool-call read-only research pass on NVIDIA Isaac Sim with ROS 2 Jazzy, MoveIt 2 as the executor path, Windows and WSL2 support, the local test topology, requirements for the Isaac host, and a list of unverified items. 100 source URLs inside (`result.sources`). |
| `2026-10-13-design-panel.json` | 2026-10-13 | Workflow output: a ten-agent design panel. Three independent designs (simplest, safety-first, adopter-first), three judges, one synthesis, two sceptics (11 and 8 objections, both "needs repair") and one repair pass. The `result.final` object is the authoritative design: `design_summary`, `repo_layout`, `mqtt_contract`, `ros2_node_design`, `ha_integration_design`, `testing_strategy`, `delivery_phases`, `risks` and `verified_facts`, the last with a URL for every verified Home Assistant, ROS 2, paho and MQTT API fact. `docs/mqtt-contract.md` and `docs/architecture.md` are readable renderings of it. |

## How to read the JSON files

Each has the same top-level shape: `summary` (one paragraph), `agentCount`, `logs`, `result` (the content), `workflowProgress`, `totalTokens` and `totalToolCalls`. The content is under `result`; long fields are Markdown inside JSON strings. To pull one field out:

```text
python -c "import json; d=json.load(open('docs/research/2026-10-13-design-panel.json', encoding='utf-8')); print(d['result']['final']['mqtt_contract'])"
```

## Caveats

- These are research outputs, not tested claims. Items each file marks as UNVERIFIED or carries forward are exactly that; the plan and the design say which ones must be verified before anything depends on them (for example the Mosquitto add-on ACL behaviour, verified only from an open issue).
- The files reflect the state of the world on 2026-10-13: Home Assistant Core 2026.10.1, ROS 2 Jazzy, paho-mqtt 2.1.0, Mosquitto add-on 7.1.1, Isaac Sim 6.1.0. Later versions may change the facts.
- The JSON files mention the development PC's hardware and the research tooling's own temporary paths as they were observed; nothing in them is configuration, and no file in this folder holds a real broker address, username or secret.
