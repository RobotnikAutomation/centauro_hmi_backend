# HMI ↔ robot backend WebSocket contract

The WebSocket endpoint, secure transport, and authentication are defined for
each deployment. The HMI does not expose or consume ROS 2 interfaces directly.

## HMI → backend message

All commands are JSON objects. The required field is `payload.name`.
`request_id` allows the response to be correlated with the request.

```json
{
  "type": "command",
  "request_id": "c85c351a-84c0-4d30-a318-77ddf7c6307e",
  "timestamp": 1730000000.0,
  "payload": {
    "name": "teleoperation.enable"
  }
}
```

If the connection is still open, the backend responds with an `ack`. The
`accepted` field indicates whether the command was accepted; `result` may include
the result of a query, and `error` may include the reason for rejection:

```json
{
  "type": "ack",
  "request_id": "c85c351a-84c0-4d30-a318-77ddf7c6307e",
  "timestamp": 1730000000.1,
  "payload": {"name": "teleoperation.enable", "accepted": true}
}
```

If the JSON is not an object, `payload` is not an object, or `payload.name` is missing,
the backend returns `type: "error"` with `payload.code: "invalid_command"`.

## HMI → backend commands

### Non-periodic commands

| Excel ID | Command | Additional payload | Immediate response | Effect |
| --- | --- | --- | --- | --- |
| — | `teleoperation.enable` | — | `ack`, with `accepted: true/false`. | Enables the teleoperation session if the robot's state and safety conditions allow it; it can then accept deadman and jog commands. It has no equivalent H2B ID in the Excel spreadsheet. |
| H2B-001 | `teleoperation.disable` | — | `ack`, with `accepted: true/false`. | Disables teleoperation, revokes deadman authorization, and stops any active jog. |
| H2B-005 | `teleoperation.set_mode` | `mode: string` | `ack`, with `accepted: true/false`. | Changes the teleoperation mode used to interpret subsequent commands. |
| H2B-006 | `teleoperation.set_speed` | `percentage: number` | `ack`, with `accepted: true/false`. | Updates the maximum speed limit applicable to teleoperation movements. |
| H2B-008 | `arm.plan_to_pose` | `operation_id: string`, `pose: object`, `frame_id: string`, optional `speed_percentage: number` | `ack`, with `accepted: true/false` and `operation_id`. | Starts planning a trajectory to the target pose without moving the robot. |
| H2B-015 | `arm.plan_to_joint_configuration` | `operation_id: string`, `positions: [q0, q1, q2, q3, q4, q5]` | `ack`, with `accepted: true/false` and `operation_id`. | Starts planning a trajectory to a joint configuration without moving the robot. |
| H2B-009 | `arm.execute_trajectory` | `operation_id: string`, `trajectory_id: string` | `ack`, with `accepted: true/false` and `operation_id`. | Executes a previously planned trajectory confirmed by the operator. |
| — | `arm.execute_pending_trajectory` | — | `ack`, with `accepted: true/false` and the IDs in `result`. | Executes the single trajectory awaiting confirmation, if one exists. |
| — | `robot.model.get` | — | `ack` with the manifest in `result`. | Describes the available URDF model and its assets without transferring their bytes. |
| — | `robot.model.download` | — | `ack` with the manifest in `result`, followed by `robot_model_chunk`. | Downloads the model ZIP in Base64 chunks. |
| H2B-010 | `poses.list`, `poses.save`, `poses.execute` | Depends on the operation; `poses.save` uses optional `pose_id: string`, `pose_name: string`, and `pose: object` | `ack`, with `accepted: true/false`; `poses.list` includes the poses in `result`. | Queries, validates/saves, or executes predefined poses. Corresponds to the `pose_management` actions. |
| H2B-011 | `home.set` | `pose: object`, `frame_id: string` | `ack`, with `accepted: true/false`. | Validates and persistently saves the robot's Home pose. |
| H2B-014 | `operation.cancel` | `operation_id: string` | `ack`, with `accepted: true/false`. | Safely cancels the specified active planning or execution operation. |

### Periodic commands

Commands in this section do not generate an `ack` when accepted. The
backend only responds with `error` if it cannot process them.

| Excel ID | Command | Additional payload | Send frequency | Effect | When messages stop |
| --- | --- | --- | --- | --- | --- |
| H2B-002 | `teleoperation.deadman` | `active: boolean` | **10 Hz** recommended; the interval must always be less than `0.5 s`. | Maintains the operator's motion authorization while `active` is `true`. | After `0.5 s` by default, revokes authorization, stops motion, and emits `teleoperation_status` with `active: false` and `reason: "deadman_timeout"`. |
| H2B-003 | `arm.joint_jog` | `velocities: [v0, v1, v2, v3, v4, v5]` | **10 Hz** while moving. | Requests velocities for each joint, subject to limits and safety constraints. | After `0.5 s` by default, sets joint velocities to zero. The session and deadman remain active if deadman messages continue to arrive. |
| H2B-004 | `arm.cartesian_jog` | `frame_id: string`, `twist: {"linear": [x, y, z], "angular": [rx, ry, rz]}` | **10 Hz** while moving. | Requests a Cartesian velocity relative to the specified frame, subject to limits and safety constraints. | After `0.5 s` by default, sets velocities to zero. The session and deadman remain active if deadman messages continue to arrive. |
| H2B-007 | `teleoperation.freedrive` | `active: boolean` | **10 Hz** while `active` is `true`; the interval must always be less than `0.5 s`. | Keeps freedrive active while the periodic command continues to arrive and `active` is `true`. | After `0.5 s` by default, disables freedrive and emits `teleoperation_status` with `freedrive: false` and `reason: "freedrive_timeout"`. |

Joint jog example:

```json
{
  "type": "command",
  "request_id": "jog-001",
  "payload": {
    "name": "arm.joint_jog",
    "velocities": [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}
```

## Backend → HMI messages

Messages emitted by the backend use the same JSON envelope
(`type`, `timestamp`, optional `request_id`, and `payload`).

| Excel ID | Message | Pattern | Minimum content |
| --- | --- | --- | --- |
| — | `ack` | Immediate response to non-periodic commands. | `name`, `accepted: boolean`; `operation_id` when applicable; optional `result` to return query data; optional `error` when `accepted` is `false`. Confirms receipt and the outcome of the request, not completion of an operation. It is part of the WebSocket envelope and has no separate row in the Excel spreadsheet. |
| — | `error` | Envelope error or a periodic command that cannot be processed. | `code`, `message`, and `name` when known. It is part of the WebSocket envelope and has no separate row in the Excel spreadsheet. |
| — | `teleoperation_status` | When teleoperation starts, ends, or its effective state changes. | `enabled`, `active`, `deadman`, `freedrive`, `mode`, `speed_percentage`, safety state, and `reason`. The Excel spreadsheet does not define a specific equivalent B2H message. |
| B2H-001, B2H-002 | `telemetry` | Periodic. | Joint state, available pose/frame, and effective teleoperation state: `enabled`, `active`, `deadman`, `freedrive`, `mode`, `speed_percentage`, and safety state. Combines the contents of `/joint_states` and `/tf + /tf_static`. |
| B2H-006 | `constraints` | Periodically or on change. | Blocked/limited directions or axes, maximum speed, and reason. |
| B2H-011 | `robot_status` | Periodically or on change. | Base, arm, tool, battery, sensor, and alarm states. The Excel spreadsheet identifies `/robot/arm/io_and_status_controller/robot_mode` as the reference. |
| B2H-009 | `operation_status` | During an operation and on state changes. | `operation_id`, `status`, normalized `progress`, and `result` or `error`. |
| B2H-007 | `planned_trajectory` | After planning a movement that requires confirmation. | `operation_id`, `trajectory_id`, trajectory, validation result, and final pose. |
| — | `robot_model_chunk` | After `robot.model.download`. | `model_id`, chunk number, total number of chunks, and Base64 ZIP data. |
| B2H-004 | `tool_camera` | Independent stream or periodic messages, depending on the agreed transport. | Image/video, encoding, timestamp, and available camera metadata. |

### Robot model

`robot.model.get` returns only the model manifest. The HMI can
compare `model_id` and `sha256` against its cache before requesting a download:

```json
{"type":"command","request_id":"model-001","payload":{"name":"robot.model.get"}}
```

The result contains `model_id`, `format: "urdf-zip"`, `root_file`, `size`,
`sha256`, `chunk_size`, and the `assets` list, with the size and SHA-256 of each
included file.

To download it:

```json
{"type":"command","request_id":"model-002","payload":{"name":"robot.model.download"}}
```

The backend first responds with an `ack` whose `result` contains the manifest,
then sends a sequence of `robot_model_chunk` messages over the same
connection. Each chunk has a zero-based `sequence`, `total`, and Base64-encoded
`data`. The HMI must concatenate the chunks in order, validate
the overall SHA-256, and extract the ZIP. The ZIP contains `robot.urdf` and
`meshes/` with relative paths.

The maximum ZIP size and chunk size are configured using
`robot_model_max_size_bytes` and `robot_model_chunk_size`, respectively.

## Message examples

The following examples show the complete envelope and representative
values. The `timestamp` and `request_id` values are illustrative.

### HMI → backend

The HMI sends messages with `type: "command"`; the specific command is given in
`payload.name`. The following commands generate an immediate response from the backend.

#### `teleoperation.enable`

Request:

```json
{"type":"command","request_id":"req-001","timestamp":1730000000.0,"payload":{"name":"teleoperation.enable"}}
```

Response:

```json
{"type":"ack","request_id":"req-001","timestamp":1730000000.01,"payload":{"name":"teleoperation.enable","accepted":true}}
```

#### `teleoperation.disable`

Request:

```json
{"type":"command","request_id":"req-002","timestamp":1730000000.1,"payload":{"name":"teleoperation.disable"}}
```

Response:

```json
{"type":"ack","request_id":"req-002","timestamp":1730000000.11,"payload":{"name":"teleoperation.disable","accepted":true}}
```

#### `teleoperation.set_mode`

Request:

```json
{"type":"command","request_id":"req-003","timestamp":1730000000.2,"payload":{"name":"teleoperation.set_mode","mode":"joint"}}
```

Response:

```json
{"type":"ack","request_id":"req-003","timestamp":1730000000.21,"payload":{"name":"teleoperation.set_mode","accepted":true}}
```

#### `teleoperation.set_speed`

Request:

```json
{"type":"command","request_id":"req-004","timestamp":1730000000.3,"payload":{"name":"teleoperation.set_speed","percentage":25.0}}
```

Response:

```json
{"type":"ack","request_id":"req-004","timestamp":1730000000.31,"payload":{"name":"teleoperation.set_speed","accepted":true}}
```

#### `arm.plan_to_pose`

Request:

```json
{"type":"command","request_id":"req-005","timestamp":1730000000.4,"payload":{"name":"arm.plan_to_pose","operation_id":"op-001","pose":{"position":[0.4,0.0,0.5],"orientation":[0.0,1.0,0.0,0.0]},"frame_id":"base_link","speed_percentage":25.0}}
```

Response:

```json
{"type":"ack","request_id":"req-005","timestamp":1730000000.41,"payload":{"name":"arm.plan_to_pose","accepted":true,"operation_id":"op-001"}}
```

When planning is complete, the backend sends `planned_trajectory` with the
trajectory that the HMI must display before execution.

#### `arm.plan_to_joint_configuration`

Request:

```json
{"type":"command","request_id":"req-012","timestamp":1730000000.45,"payload":{"name":"arm.plan_to_joint_configuration","operation_id":"op-002","positions":[0.2,-1.2,0.4,-1.4,0.0,0.2]}}
```

Response:

```json
{"type":"ack","request_id":"req-012","timestamp":1730000000.46,"payload":{"name":"arm.plan_to_joint_configuration","accepted":true,"operation_id":"op-002"}}
```

With `robot.type: mock`, the robot linearly interpolates between the current configuration and `positions`; the
HMI receives the resulting points through `planned_trajectory`.

#### `arm.execute_trajectory`

Request:

```json
{"type":"command","request_id":"req-006","timestamp":1730000000.5,"payload":{"name":"arm.execute_trajectory","operation_id":"op-001","trajectory_id":"traj-op-001"}}
```

Response:

```json
{"type":"ack","request_id":"req-006","timestamp":1730000000.51,"payload":{"name":"arm.execute_trajectory","accepted":true,"operation_id":"op-001"}}
```

Subsequent progress is published through `operation_status`.

#### `operation.cancel`

Request:

```json
{"type":"command","request_id":"req-011","timestamp":1730000000.55,"payload":{"name":"operation.cancel","operation_id":"op-001"}}
```

Response:

```json
{"type":"ack","request_id":"req-011","timestamp":1730000000.56,"payload":{"name":"operation.cancel","accepted":true,"operation_id":"op-001"}}
```

#### `poses.list`

Request:

```json
{"type":"command","request_id":"req-007","timestamp":1730000000.6,"payload":{"name":"poses.list"}}
```

Response:

```json
{"type":"ack","request_id":"req-007","timestamp":1730000000.61,"payload":{"name":"poses.list","accepted":true,"result":{"poses":[{"pose_id":"home","name":"home","pose":{"position":[0.0,-0.4,0.5],"orientation":[0.0,1.0,0.0,0.0]},"frame_id":"base_link","is_home":true}]}}}
```

#### `poses.save`

Request:

```json
{"type":"command","request_id":"req-008","timestamp":1730000000.7,"payload":{"name":"poses.save","pose_id":"inspection","pose_name":"inspection","pose":{"position":[0.4,0.0,0.5],"orientation":[0.0,1.0,0.0,0.0]}}}
```

Response:

```json
{"type":"ack","request_id":"req-008","timestamp":1730000000.71,"payload":{"name":"poses.save","accepted":true}}
```

#### `poses.execute`

Request:

```json
{"type":"command","request_id":"req-009","timestamp":1730000000.8,"payload":{"name":"poses.execute","operation_id":"op-002","pose_id":"inspection"}}
```

Response:

```json
{"type":"ack","request_id":"req-009","timestamp":1730000000.81,"payload":{"name":"poses.execute","accepted":true,"operation_id":"op-002"}}
```

Subsequent progress is published through `operation_status`.

#### `home.set`

Request:

```json
{"type":"command","request_id":"req-010","timestamp":1730000000.9,"payload":{"name":"home.set","pose":{"position":[0.0,-0.4,0.5],"orientation":[0.0,1.0,0.0,0.0]},"frame_id":"base_link"}}
```

Response:

```json
{"type":"ack","request_id":"req-010","timestamp":1730000000.91,"payload":{"name":"home.set","accepted":true}}
```

### Periodic HMI → backend messages

The HMI must send these messages repeatedly for as long as it needs to keep the
state or motion active.

#### `teleoperation.deadman`

Request:

```json
{"type":"command","request_id":"hb-001","timestamp":1730000001.0,"payload":{"name":"teleoperation.deadman","active":true}}
```

Immediate response: none. Deadman messages must be sent periodically; if the
timeout expires, the backend publishes `teleoperation_status` with
`reason: "deadman_timeout"`.

#### `arm.joint_jog`

Request:

```json
{"type":"command","request_id":"jog-001","timestamp":1730000001.0,"payload":{"name":"arm.joint_jog","velocities":[0.3,0.0,0.0,0.0,0.0,0.0]}}
```

Immediate response: none. Jog messages must be sent periodically; if the
timeout expires, the backend sets velocities to zero and reflects this in `telemetry`.

#### `arm.cartesian_jog`

Request:

```json
{"type":"command","request_id":"jog-002","timestamp":1730000001.0,"payload":{"name":"arm.cartesian_jog","frame_id":"base_link","twist":{"linear":[0.05,0.0,0.0],"angular":[0.0,0.0,0.0]}}}
```

Immediate response: none. Jog messages must be sent periodically; if the
timeout expires, the backend sets velocities to zero and reflects this in `telemetry`.

#### `teleoperation.freedrive`

Request to enable:

```json
{"type":"command","request_id":"fd-001","timestamp":1730000001.0,"payload":{"name":"teleoperation.freedrive","active":true}}
```

Immediate response: none. The command must be sent periodically; if the
timeout expires, the backend disables freedrive and publishes `teleoperation_status` with
`reason: "freedrive_timeout"`.

Request to disable:

```json
{"type":"command","request_id":"fd-002","timestamp":1730000001.1,"payload":{"name":"teleoperation.freedrive","active":false}}
```

Immediate response: none; freedrive is disabled.

### Messages received by the HMI ← backend

Acknowledgment of an accepted command:

```json
{"type":"ack","request_id":"req-001","timestamp":1730000000.01,"payload":{"name":"teleoperation.enable","accepted":true}}
```

Rejection of a command for an unavailable operation:

```json
{"type":"ack","request_id":"req-006","timestamp":1730000000.51,"payload":{"name":"arm.execute_trajectory","accepted":false,"error":{"code":"trajectory_not_found","message":"the trajectory does not exist or is no longer available"}}}
```

Effective teleoperation state, emitted on change or when a watchdog expires:

```json
{"type":"teleoperation_status","timestamp":1730000001.05,"payload":{"enabled":true,"active":true,"deadman":true,"freedrive":true,"mode":"joint","speed_percentage":25.0,"safety":"ok","reason":"freedrive_active"}}
```

Joint and session telemetry:

```json
{"type":"telemetry","timestamp":1730000001.05,"payload":{"joint_names":["shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"],"positions":[0.0,-1.57,0.0,-1.57,0.0,0.0],"velocities":[0.0,0.0,0.0,0.0,0.0,0.0],"frame_id":"base_link","tool_frame":"tool0","teleoperation":{"enabled":true,"active":true,"deadman":true,"freedrive":true,"mode":"joint","speed_percentage":25.0,"safety":"ok","reason":"freedrive_active"}}}
```

Motion constraints:

```json
{"type":"constraints","timestamp":1730000001.05,"payload":{"directions":{"x+":true,"x-":true,"y+":true,"y-":true,"z+":true,"z-":true},"max_velocity_percentage":25.0,"reason":"none"}}
```

Overall robot status:

```json
{"type":"robot_status","timestamp":1730000001.05,"payload":{"base":"available","arm":"available","tool":"available","battery_percentage":87.0,"alarms":[]}}
```

Operation status:

```json
{"type":"operation_status","timestamp":1730000001.1,"payload":{"operation_id":"op-001","status":"executing","progress":0.35}}
```

Planned trajectory awaiting confirmation:

```json
{"type":"planned_trajectory","timestamp":1730000001.1,"payload":{"operation_id":"op-001","trajectory_id":"traj-op-001","trajectory":{"joint_names":["shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"],"points":[{"positions":[0.0,-1.57,0.0,-1.57,0.0,0.0]},{"positions":[0.0,-1.57,0.0,-1.57,0.0,0.0]}]},"validation":{"valid":true},"final_pose":{"frame_id":"base_link","position":[0.4,0.0,0.5],"orientation":[0.0,1.0,0.0,0.0]}}}
```

Base64-encoded tool camera image:

```json
{"type":"tool_camera","timestamp":1730000001.1,"payload":{"encoding":"base64","format":"png","data":"iVBORw0KGgoAAAANSUhEUgAAAAEAAAAB..."}}
```

## Pending Excel mappings

The `Arquitectura HMI - Backend` sheet in the Excel spreadsheet contains messages
that are not yet described as WebSocket messages in this contract:

| Excel ID | Reference interface / message | Status in this contract |
| --- | --- | --- |
| H2B-012 | `/robot/arm/sequence_control` | A command to start, pause, resume, and cancel sequences is missing. |
| B2H-003 | Sensor data for 3D visualization | A message/stream for scene data, depth, and additional sensors is missing. |
| B2H-005 | Video metadata / overlays | A message/stream for detections, references, distances, and zones for overlays is missing. |

`teleoperation.enable` corresponds to H2B-013 and `operation.cancel` to H2B-014
in the Excel spreadsheet.

Periodic commands (`teleoperation.deadman`, `arm.joint_jog`,
`arm.cartesian_jog`, and `teleoperation.freedrive`) do not generate an `ack` when
accepted. The HMI must use
`telemetry`, `constraints`, and `teleoperation_status` to determine the
effective state. The backend only responds with `error` if it cannot accept one of
these commands.

## Frequencies and watchdogs

Default values:

| Item | Frequency or duration |
| --- | --- |
| Backend → HMI telemetry and streams | `telemetry_hz: 20 Hz` (every 50 ms) |
| Telemetry and operation status | `20 Hz` while an operation is active |
| Deadman timeout | `0.5 s` |
| Jog timeout | `0.5 s` |
| Freedrive timeout | `0.5 s` |
| Recommended deadman and jog frequency | `10 Hz` (every 100 ms) |
| Incoming-message log summary | `log_stats_period_sec: 5 s` |

On each `telemetry_hz` cycle, the WebSocket emits four messages:

- `telemetry`: joints, velocities, and teleoperation state.
- `constraints`: active constraints and maximum speed percentage.
- `robot_status`: overall robot status.
- `tool_camera`: tool camera image or stream, using the encoding negotiated for the deployment.

### Deadman rule

For manual jogging, the HMI must continuously send `teleoperation.deadman`
with `active: true` **and** the jog command. To use freedrive, it must
also continuously send `teleoperation.freedrive` with `active: true`. Each
command has an independent watchdog. The example script uses 10 Hz, giving an interval
shorter than the configured timeout of 0.5 s.

If no `teleoperation.deadman` with `active: true` arrives within the configured
timeout, the backend:

1. Deactivates the deadman.
2. Sets joint velocities to zero.
3. Publishes `telemetry.teleoperation.active: false` on the next cycle.

WebSocket disconnection produces the same result when that timeout expires,
because deadman messages stop arriving. The actual stop must also be
enforced by the robot's control and safety layer.

If deadman messages continue but `arm.joint_jog` or
`arm.cartesian_jog` messages stop arriving, velocities are set to zero when the same
timeout expires. The session remains enabled and the `deadman` field remains active;
the HMI must send periodic jog messages again to resume moving the arm.

### Freedrive rule

Freedrive remains active only while the backend periodically receives
`teleoperation.freedrive` with `active: true`. If the command stops arriving
for the configured timeout, the backend disables freedrive and publishes the
change in `teleoperation_status`. Sending `active: false` disables it
immediately.

### Missing backend → HMI publications

The backend publishes while it remains active. If the client receives no messages or
the connection closes, the backend neither retries nor retains a history. The HMI
should mark telemetry as stale if it receives no messages for
at least three publication periods (150 ms at the default frequency)
and disable its motion controls.
