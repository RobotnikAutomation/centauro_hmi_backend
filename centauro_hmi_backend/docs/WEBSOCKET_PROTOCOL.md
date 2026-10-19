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

## Mock point cloud stream

With `robot.type=mock` and `mock.point_cloud_source=scans` or `pointcloud`,
the backend broadcasts `point_cloud` at recorded bag speed, looping indefinitely.
The sources are mutually exclusive. `none` disables both (the default).
See [configuration and launch examples](../README.md#mock-point-cloud-playback).

```json
{
  "type": "point_cloud",
  "timestamp": 1792414278.0,
  "payload": {
    "stream_id": "scans",
    "source": "scans",
    "topic": "/robot/laser/filtered_scan",
    "frame_id": "robot_rear_laser_link",
    "encoding": "base64",
    "format": "ros_pointcloud2",
    "data": "AACAPwAAAAAAAAAA",
    "height": 1,
    "width": 1,
    "fields": [
      {"name": "x", "offset": 0, "datatype": 7, "count": 1},
      {"name": "y", "offset": 4, "datatype": 7, "count": 1},
      {"name": "z", "offset": 8, "datatype": 7, "count": 1}
    ],
    "is_bigendian": false,
    "point_step": 12,
    "row_step": 12,
    "is_dense": true,
    "timestamp": {"sec": 1792409862, "nanosec": 123460298},
    "sequence": 1,
    "loop_index": 0
  }
}
```

`data` is the Base64-encoded **PointCloud2 data buffer**, not serialized CDR
or an array of JSON points. Decode it with the ROS PointField datatype codes,
field offsets, `point_step`, `row_step`, and `is_bigendian`. Coordinates are
meters in `frame_id`; `topic` names the original recorded input. For `scans`,
the original input is LaserScan and only finite, in-range returns are projected
to XYZ (`z=0`, little-endian FLOAT32). For `pointcloud`, all original fields,
bytes, row padding, organization, and header values are preserved, including
invalid points when `is_dense` is false. An empty projected scan has width zero.

`stream_id` and `source` are the selected source (`scans` or `pointcloud`).
`sequence` increases for every emitted frame across loops; `loop_index` is
zero-based. Payload `timestamp` is the original header stamp and repeats each
loop; the envelope timestamp is current send time. Recorded inter-message
intervals are retained, with a final frame interval before restarting.
All matching topics are replayed in bag order, without TF transformation,
ROS topic publication, or a dependency on `telemetry_hz`. Camera delivery
is unaffected. New clients join at the current playback position.

The supplied native clouds exceed 1 MiB per JSON message. Configure client
receive limits accordingly (for example `websockets.connect(url, max_size=None)`).
Startup fails for invalid selected bags. A runtime read/decode failure stops
this stream, logs the error, and broadcasts `type: "error"` with
`payload.code: "point_cloud_replay_failed"` and `payload.message`.

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
| — | `tf.lookup` | `reference_frame: string`, `observed_frame: string`, optional `continuous: boolean` (false), `timeout_sec: number` (5.0) | `ack` with `result.observation_id`. | Read-only TF action; emits `tf_transform` feedback/result. |
| — | `tf.cancel` | `observation_id: string` | `ack`, with `accepted: true/false`. | Requests cancellation of this connection's observation, including pending goals. |
| H2B-010 | `poses.list`, `poses.save`, `poses.execute` | Depends on the operation; `poses.save` uses optional `pose_id: string`, `pose_name: string`, and `pose: object` | `ack`, with `accepted: true/false`; `poses.list` includes the poses in `result`. | Queries, validates/saves, or executes predefined poses. Corresponds to the `pose_management` actions. |
| H2B-011 | `home.set` | `pose: object`, `frame_id: string` | `ack`, with `accepted: true/false`. | Validates and persistently saves the robot's Home pose. |
| H2B-014 | `operation.cancel` | `operation_id: string` | `ack`, with `accepted: true/false`. | Safely cancels the specified active planning or execution operation. |

See [Transform lookup and continuous observation](../README.md#transform-lookup-and-continuous-observation)
for the `tf_transform` schema, correlation, broadcasts, disconnect cancellation,
error codes and timeout semantics. Positions are observed-frame coordinates in
the reference frame; TF lookup timeout does not enforce transform freshness.

### Periodic commands

Commands in this section do not generate an `ack` when accepted. The
backend only responds with `error` if it cannot process them.

| Excel ID | Command | Additional payload | Send frequency | Effect | When messages stop |
| --- | --- | --- | --- | --- | --- |
| H2B-002 | `teleoperation.deadman` | `active: boolean` | **10 Hz** recommended; the interval must always be less than `0.5 s`. | Maintains the operator's motion authorization while `active` is `true`. | After `0.5 s` by default, revokes authorization, stops motion, and emits `teleoperation_status` with `active: false` and `reason: "deadman_timeout"`. |
| H2B-003 | `arm.joint_jog` | `velocities: [v0, v1, v2, v3, v4, v5]` | **10 Hz** while moving. | Requests velocities for each joint, subject to limits and safety constraints. | After `0.5 s` by default, sets joint velocities to zero. The session and deadman remain active if deadman messages continue to arrive. |
| H2B-004 | `arm.cartesian_jog` | `frame_id: string`, `twist: {"linear": [x, y, z], "angular": [rx, ry, rz]}` | **10 Hz** while moving. | Requests a Cartesian velocity relative to the specified frame, subject to limits and safety constraints. | After `0.5 s` by default, sets velocities to zero. The session and deadman remain active if deadman messages continue to arrive. |
| — | `base.joystick` | `twist: {"linear": [x, y, 0], "angular": [0, 0, yaw]}` | Periodically while controlling the base; interval must remain below `command_timeout_sec`. | Scales normalized planar axes. Real mode publishes `geometry_msgs/Twist` to `/robot/pad_teleop/cmd_vel`; mock mode integrates planar motion and broadcasts `base_link -> sim_base`. Both run at `base_cmd_vel_hz` (100 Hz by default). | Real mode publishes zero velocity; mock mode holds the last pose until fresh joystick input arrives. |
| H2B-007 | `teleoperation.freedrive` | `active: boolean` | **10 Hz** while `active` is `true`; the interval must always be less than `0.5 s`. | Keeps freedrive active while the periodic command continues to arrive and `active` is `true`. | After `0.5 s` by default, disables freedrive and emits `teleoperation_status` with `freedrive: false` and `reason: "freedrive_timeout"`. |
| — | `arm.cartesian_target` | `pose: {"position": [x, y, z], "orientation": [qx, qy, qz, qw]}`, optional `frame_id: string` (default `ik.base_frame`) | Periodically while following; interval must remain below `command_timeout_sec`. | Broadcasts the pose as TF `ik.target_frame` and keeps a continuous goal on the `cartesian_kinematics_for_hmi_teleoperation` IK action server. IK solutions are streamed back as `ik_solution`. Does not move the robot. | After `0.5 s` by default, cancels the IK goal; a final `ik_solution` is emitted. |

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

Base joystick example (axis values are normalized to `[-1, 1]`):

```json
{
  "type": "command",
  "payload": {
    "name": "base.joystick",
    "twist": {
      "linear": [0.5, -0.25, 0.0],
      "angular": [0.0, 0.0, 0.4]
    }
  }
}
```

With the default PS4/PS5 pad configuration, this converts to
`linear.x = 0.5 m/s`, `linear.y = -0.25 m/s`, and `angular.z = 0.6 rad/s`.
Limits are loaded from the selected `robot_bringup/config/base/pad/<pad_model>.yaml`;
linear z and angular x/y must be zero. In real mode, the backend republishes the
latest command at 100 Hz and publishes zero after `command_timeout_sec` without
a fresh joystick message. In mock mode, it broadcasts `base_link -> sim_base`
at the same rate, integrating local-frame x/y translation and yaw using elapsed
ROS time. It holds that pose when input times out and never publishes to the
real base velocity topic. In both modes, base control is independent of arm
enable, deadman, freedrive, and speed percentage.

## Backend → HMI messages

Messages emitted by the backend use the same JSON envelope
(`type`, `timestamp`, optional `request_id`, and `payload`).

| Excel ID | Message | Pattern | Minimum content |
| --- | --- | --- | --- |
| — | `ack` | Immediate response to non-periodic commands. | `name`, `accepted: boolean`; `operation_id` when applicable; optional `result` to return query data; optional `error` when `accepted` is `false`. Confirms receipt and the outcome of the request, not completion of an operation. It is part of the WebSocket envelope and has no separate row in the Excel spreadsheet. |
| — | `error` | Envelope error or a periodic command that cannot be processed. | `code`, `message`, and `name` when known. It is part of the WebSocket envelope and has no separate row in the Excel spreadsheet. |
| — | `teleoperation_status` | When teleoperation starts, ends, or its effective state changes. | `enabled`, `active`, `deadman`, `freedrive`, `mode`, `speed_percentage`, safety state, and `reason`. The Excel spreadsheet does not define a specific equivalent B2H message. |
| B2H-001, B2H-002 | `telemetry` | Periodic. | Joint state, available pose/frame, and effective teleoperation state: `enabled`, `active`, `deadman`, `freedrive`, `mode`, `speed_percentage`, and safety state. Combines the contents of `/joint_states` and `/tf + /tf_static`. |
| — | `tf_frames` | On every connection and whenever the discovered frame list changes, in both real and mock modes. | `frames`: sorted, unique TF frame names, including roots. |
| B2H-006 | `constraints` | Periodically or on change. | Blocked/limited directions or axes, maximum speed, and reason. |
| B2H-011 | `robot_status` | Periodically or on change. | Mock mode emits demo values. Real mode includes the latest available `base_status`, `robot_mode`, `robot_program_running`, `safety_mode`, `tool_data`, `safety_module_status`, and `battery` fields; `battery_percentage` is copied from `battery.level`. |
| B2H-009 | `operation_status` | During an operation and on state changes. | `operation_id`, `status`, normalized `progress`, and `result` or `error`. |
| B2H-007 | `planned_trajectory` | After planning a movement that requires confirmation. | `operation_id`, `trajectory_id`, trajectory, validation result, and final pose. |
| — | `robot_model_chunk` | After `robot.model.download`. | `model_id`, chunk number, total number of chunks, and Base64 ZIP data. |
| — | `ik_solution` | For each IK solution while `arm.cartesian_target` is active, plus one final message when the goal ends. | `final: boolean`, `joint_names` (ROS joint names), `positions`; non-final messages add `position_error` [m], `orientation_error` [rad], `joint_delta_norm`; final messages add `success` and `message`. |
| B2H-004 | `tool_camera` | Mock WebSocket and real modes send the newest received frame per stream, at most once per telemetry tick. Mock WebRTC mode emits no camera messages on this socket. | `stream_id`, per-stream `sequence`, `topic`, `encoding: "base64"`, `format: "ros_image"`, raw pixel `data`, `image_encoding`, `width`, `height`, `step`, `is_bigendian`, and source `timestamp`. |

In real mode, `telemetry.payload.teleoperation.mode` and
`teleoperation_status.payload.mode` are updated from
`/robot/arm/servo/teleoperation_mode/state`. The ROS integer values map to
`cartesian_tcp` (`0`), `cartesian_base` (`1`), and `joint` (`2`); `-1` maps to
`unknown` until the first mode is selected. Telemetry carries the latest value
periodically, and `teleoperation_status` is emitted when the value changes.

### Robot model
In real mode, status fields preserve their ROS message structure rather than
being flattened. Read arm robot mode from `payload.robot_mode.mode`, whether the
external-control program is running from
`payload.robot_program_running.data`, and the UR arm safety mode from
`payload.safety_mode.mode`. Safety-module values are under
`payload.safety_module_status`, including `operation_mode`, `safety_mode`,
`emergency_stop`, `safety_stop`, `current_speed`, `laser_mode`, and
`laser_status`. The two `safety_mode` values are distinct: the top-level one is
the UR arm's numeric mode; the nested one is the safety module's string mode.
Values are the latest received samples and can have different update times;
fields are absent until their topics have delivered a sample.

### Available TF frames

Both real and mock modes subscribe to `/tf` and `/tf_static`, including latched
static transforms from publishers that started before the backend. Every
connection (including reconnects) receives the current frame list:

```json
{"type":"tf_frames","timestamp":1730000001.0,"payload":{"frames":["base_link","sim_base","tool0"]}}
```

The list is sorted and deduplicated and includes both parent/root and child
names across all discovered TF trees. Changes are broadcast to all clients at
most once per `telemetry_hz` tick; pose updates alone do not resend the list.
Before any TF is received, `frames` is `[]`. This is discovery of the ROS TF
buffer, not a list synthesized from the robot model, and contains no transform
poses. Mock arm frames require a TF publisher such as the mock visualization
launch; the backend's own `sim_base` TF is discovered in the same way.
Frame names remain known to the buffer even after a publisher stops; inclusion
does not guarantee a fresh transform or connectivity to another listed frame.

### Robot model

Mock mode sends a `robot_description` message immediately on every connection
(including reconnects):

```json
{"type":"robot_description","timestamp":1730000001.0,"payload":{"topic":"/robot/robot_description","data":"<?xml version=\"1.0\"?><robot name=\"rbkairos_plus\">...</robot>"}}
```

`data` is the expanded RB-KAIROS+ URDF from `robotnik_description`, with
`namespace:=robot`, `prefix:=robot_`, and the default UR10e arm. It is identical
to the `std_msgs/msg/String.data` published on `/robot/robot_description`
(reliable, transient-local, depth 1). This is a static, once-per-connection
message, not a telemetry stream. Real mode does not publish this topic or
WebSocket message. The XML uses ROS `package://` asset URIs; use the download
below for a portable copy with meshes and textures.

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
`meshes/` with relative paths. In mock mode this is the same RB-KAIROS+ model,
including referenced meshes and COLLADA textures from its dependency packages.
This model describes the full robot; the existing mock telemetry and motion
commands remain the simplified six-joint simulation, not full-robot dynamics.

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

#### `arm.cartesian_target`

Request:

```json
{"type":"command","request_id":"ik-001","timestamp":1730000001.0,"payload":{"name":"arm.cartesian_target","frame_id":"robot_arm_base_link","pose":{"position":[0.4,0.0,0.5],"orientation":[0.0,1.0,0.0,0.0]}}}
```

Immediate response: none. The backend streams `ik_solution` messages:

```json
{"type":"ik_solution","timestamp":1730000001.02,"request_id":null,"payload":{"final":false,"joint_names":["robot_arm_shoulder_pan_joint","robot_arm_shoulder_lift_joint","robot_arm_elbow_joint","robot_arm_wrist_1_joint","robot_arm_wrist_2_joint","robot_arm_wrist_3_joint"],"positions":[0.0,-1.2,1.4,-1.8,-1.57,0.0],"position_error":0.0001,"orientation_error":0.0002,"joint_delta_norm":0.003}}
```

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

Base64-encoded raw ROS camera image (one RGB8 pixel for illustration):

```json
{"type":"tool_camera","timestamp":1730000001.1,"payload":{"stream_id":"front_camera","sequence":42,"topic":"/robot/front_rgbd_camera/color/image_raw","encoding":"base64","format":"ros_image","data":"AAH/","image_encoding":"rgb8","width":1,"height":1,"step":3,"is_bigendian":false,"timestamp":{"sec":1730000001,"nanosec":100000000}}}
```

`data` contains raw pixels, not a PNG/JPEG file. Use `image_encoding`, `step`,
and byte order to render them. Frames received between telemetry ticks are
replaced by the latest frame; no message is sent without a new image.
Mock WebSocket mode replaces the old test PNG with this real-image payload.
Its bag loops at end-of-file: source timestamps reset each loop, while sequence
numbers continue increasing. In mock WebRTC mode only commands and telemetry
remain on this socket; receive `front_camera` through the separate WebRTC
server on HTTPS port 8081. See [Mock camera playback](../README.md#mock-camera-playback).

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
