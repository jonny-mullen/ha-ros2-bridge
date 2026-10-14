# Voice control with Assist

"Reset the arm" spoken to a Voice PE starts a task on a robot and gets a spoken acknowledgement at once; the outcome is spoken back later by an automation. This page follows that sentence end to end, installs the sentences, and lists what the bridge says and where it stops.

The design behind it is interview decision Q10: every task is asynchronous, the spoken reply acknowledges and never waits for the result, and the result reaches the household as a bus event that any automation (speech, notification, lights) can act on.

## How a spoken command travels

```text
Voice PE  ->  Assist pipeline (speech to text)
          ->  conversation agent matches custom_sentences/en/ros2_bridge.yaml
          ->  intent Ros2BridgeSendTask {task, name, area}
          ->  ros2_bridge intent handler: resolve robot and task, publish cmd
          ->  MQTT {prefix}/{robot_id}/cmd  (source.kind = intent)
          ->  robot node authorises, acks on .../task/{id}/ack
          ->  handler speaks one of three replies (waits at most 1.5 s)
              ... the robot runs the task ...
          ->  robot publishes .../task/{id}/result
          ->  integration fires ros2_bridge_task_result on the bus
          ->  automation announces the outcome on the satellite
```

1. The satellite (a Voice PE, any Assist satellite, or the text Assist dialog) hands the words to the Assist pipeline, which turns speech into text and passes it to the pipeline's conversation agent.
2. Home Assistant's built-in conversation agent matches the text against the sentence templates in `custom_sentences/en/ros2_bridge.yaml` and produces the intent `Ros2BridgeSendTask` (or `Ros2BridgeCancelTask`) with up to three slots: `task`, `name` and `area`.
3. The integration's intent handler, registered once when Home Assistant starts, resolves which robot is meant and which announced task the words name (both rules are below), then publishes a command through the same hub the actions and buttons use. The command's `source` records `kind: intent`, the user id, the context id and the satellite's device id, so the acknowledgement and the robot's log show where it came from.
4. The robot node authorises the command (allowlist, freshness, optional signature, one motion at a time) and publishes an acknowledgement. The handler waits for it for at most `min(ack_timeout_s, 1.5)` seconds, then speaks.
5. The robot runs the task and publishes its result. The integration matches it by id and fires `ros2_bridge_task_result` exactly once, with `status`, `code` and `reason`.
6. An automation on that event speaks the outcome on the satellite. Nothing in the voice path blocks while the arm moves.

## Installing the sentences

1. Copy `docs/examples/custom_sentences/en/ros2_bridge.yaml` to `<config>/custom_sentences/en/ros2_bridge.yaml` on the Home Assistant machine (create the folders if they do not exist).
2. Reload the conversation integration (Developer tools, YAML, "Conversation") or restart Home Assistant.
3. Say or type a sentence. Developer tools, Assist, is the quickest place to try one.

The file defines these sentence patterns (words in brackets are optional, words separated by `|` are alternatives):

| Intent | Sentences | Example |
|---|---|---|
| `Ros2BridgeSendTask` | `(tell|ask) {name} to {task}`, `{task} (with|on|using) {name}`, `[please] make {name} {task}` | "Tell left arm to reset" |
| | `{task} the (arm|robot) in the {area}`, `{task} the {area} (arm|robot)` | "Reset the kitchen arm" |
| | `{task} the (arm|robot)`, `(arm|robot) {task}` | "Reset the arm" |
| `Ros2BridgeCancelTask` | `(stop|cancel) [what] {name} [is doing]`, `(stop|cancel) the [current] task on {name}` | "Stop left arm" |
| | `(stop|cancel) the (arm|robot) in the {area}`, `(stop|cancel) the {area} (arm|robot)` | "Stop the kitchen robot" |
| | `(stop|cancel) the (arm|robot)`, `(arm|robot) (stop|cancel)` | "Stop the arm" |

All three slots are wildcards, so any robot or task name works without editing the file; the integration does the matching. The sentences are English; another language is a copy of the file under `custom_sentences/<language>/` with the templates translated (the slot names must stay `task`, `name` and `area`).

If the pipeline uses a large-language-model conversation agent, keep its "Prefer handling commands locally" setting on so these sentences are tried by the built-in agent first.

## The three spoken replies

For a task:

| What happened within the wait | Reply |
|---|---|
| The robot accepted the command | "Starting {label} on {robot}" |
| The robot refused it | "{robot} refused: {reason}" (the acknowledgement's reason, or its code when there is no reason) |
| No acknowledgement arrived within 1.5 s | "Sent {label} to {robot}, I will report back" |

For a cancel the same three shapes are "Cancelling the current task on {robot}", "{robot} refused: {reason}" and "Sent cancel to {robot}, I will report back".

`{label}` is the task's announced label ("Reset to home"), `{robot}` the robot's announced name ("Left arm"). The third reply is not a failure: the command is still tracked, its acknowledgement and result still fire the bus events, and the speak-back automation reports the outcome as usual.

When the command cannot be sent at all, the handler answers with an error response instead:

| Situation | Reply |
|---|---|
| The integration's entry is not loaded | "The robot bridge is not loaded" |
| Its MQTT subscriptions are not confirmed yet | "The robot bridge is not ready yet" |
| No robot has announced | "No robot has announced itself yet" |
| More than one robot matches | "Which arm do you mean?" |
| The name matches no robot | "I do not know a robot called {name}" |
| The area is unknown, or has no robot | "I do not know an area called {area}", "There is no robot in {area}" |
| The task is not in the robot's catalogue | "{robot} has no task called {task}" |
| The task has required parameters | "{label} needs details I cannot take by voice" |
| The robot is unavailable | "{robot} is not available right now" |
| The robot is at its pending cap | "{robot} has too many commands waiting" |
| The publish failed | "I could not reach {robot}" |

## How the robot is resolved

The handler tries these in order and stops at the first that gives exactly one robot. Only robots that have announced count.

1. **The `name` slot as a robot id.** The spoken words are case-folded, hyphens and spaces become underscores, and the result is compared with the announced ids: "arm left" and "arm-left" both mean `arm_left`.
2. **The `name` slot as a name.** The folded words are compared with the robot's announced name ("Left arm"), with its id spoken as words ("arm left"), and with the device's name in Home Assistant, including a name you gave it yourself on the device page. Renaming the device is therefore the way to teach the bridge a new spoken name.
3. **The `area` slot.** The area is looked up by name, by alias or by id, and the robots whose device sits in that area are the candidates. Put the robot's device in an area (the announce can suggest one) for "reset the kitchen arm" to work.
4. **The satellite's area.** With neither slot, the area of the device that heard the command (the Voice PE's own device in Home Assistant) is used in the same way, so "reset the arm" in the kitchen means the kitchen's robot.
5. **The only robot.** With one robot announced and nothing else to go on, that robot is meant.

Several candidates at any step give "Which arm do you mean?"; none gives the matching "I do not know" reply.

## How the task is resolved

1. The `task` slot is folded the same way as a robot id and compared with the announced task names: "move to" means `move_to`.
2. Failing that, it is compared case-insensitively with the announced labels: "reset to home" means the task whose label is "Reset to home".

A task whose announced schema lists required parameters (for example `move_to` with its `pose`) cannot be started by voice, because the sentences carry no parameter values; the reply says so, and the `send_task` action is the way to start such a task with its parameters.

## The sentence-trigger alternative

`docs/examples/sentence_trigger_automation.yaml` does the same job with no file under `custom_sentences/`: two automations use `trigger: conversation` with their own `command` templates, turn the spoken words into a robot id and a task name themselves (spaces to underscores), call `ros2_bridge.send_task` or `ros2_bridge.cancel_task` with `wait: ack` and a response variable, and set the spoken reply with `set_conversation_response` from the response.

The trade-off: a sentence trigger owns its words, which is handy for a fixed vocabulary or a bespoke phrase, but it has to do the name and task resolution itself, so it knows nothing about areas, satellites, labels or device names. The intents are the better default; the sentence trigger is there for a household that wants one or two exact phrases.

## Speaking the outcome back

Automation 2 in `docs/examples/automations.yaml` listens for `ros2_bridge_task_result`, keeps only `kind == motion` so a pause or a stop is not announced as a failure, and calls `assist_satellite.announce` on the kitchen satellite with a sentence built from `status`, `task`, `robot_id`, `reason` and `code`: "Left arm finished reset to home", "Left arm refused move to: Task 'reset' is running", "I lost track of reset on left arm (no_ack)", and so on. Swap `assist_satellite.announce` for `tts.speak` or a `notify` action to suit the household; automation 3 in the same file is the notification form.

Because the event fires exactly once per command id, the announcement happens once however the task was started: by voice, by a button, by an automation or by the action.

## Limits

- **Acknowledgement only.** The voice reply never waits for the result (Q10). Even with `wait: result` available in the action, the intent handler does not use it; a task that takes a minute would otherwise hold the pipeline for a minute.
- **A 1.5 s cap on the wait.** The handler waits `min(ack_timeout_s, 1.5)` seconds. On a slow link the reply is "I will report back" even though the robot accepts a moment later; the acknowledgement event still fires with the real outcome.
- **No parameters by voice.** Tasks with required parameters need the action.
- **One robot per sentence.** "Reset everything" is an automation calling `get_robots` and `send_task` in a loop (automation 1 in the examples), not a sentence.
- **Entry not loaded.** The intents exist from Home Assistant start, so a sentence spoken while the entry is not loaded (for instance while MQTT is down) is understood and answered with "The robot bridge is not loaded" rather than falling through to "Sorry, I couldn't understand that".
- **Options reloads do not silence voice.** The intent handlers are registered once and never removed, so saving the options page does not lose them.
