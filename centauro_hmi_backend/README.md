# centauro_hmi_backend

ROS 2 (Jazzy) node that exposes the CENTAURO HMI teleoperation contract
over WebSocket. It keeps the transport, message contract, and robot
implementation separate.

Currently, only `robot.type: mock` is implemented: a lightweight simulated
six-joint arm, without Gazebo or MoveIt. The `real` value is reserved
for future integration with the physical robot and **currently raises an error
on startup**.

If this is your first time exploring this repository, start with the
[root README](../README.md).

## Quick start

With the workspace already built and sourced (see the [root README](../README.md)):

```bash
ros2 run centauro_hmi_backend hmi_backend
```

The WebSocket server listens at `ws://127.0.0.1:8765`.

It is usually best to start it with its configuration file:

```bash
ros2 launch centauro_hmi_backend backend.launch.py
```

## WebSocket contract

The full contract (payloads, responses, frequencies, and watchdogs) is documented in
[docs/WEBSOCKET_PROTOCOL.md](docs/WEBSOCKET_PROTOCOL.md). Summary:

Each message is JSON with this envelope:

```json
{"type":"command", "request_id":"optional-id", "timestamp":0.0,
 "payload":{"name":"teleoperation.enable"}}
```

### Accepted commands (client → backend)

| Category | Commands |
| --- | --- |
| Teleoperation | `teleoperation.enable`, `teleoperation.disable`, `teleoperation.set_mode`, `teleoperation.set_speed` |
| Teleoperation (periodic) | `teleoperation.deadman`, `teleoperation.freedrive` |
| Continuous motion (periodic) | `arm.joint_jog`, `arm.cartesian_jog` |
| Planning and execution | `arm.plan_to_pose`, `arm.plan_to_joint_configuration`, `arm.execute_trajectory`, `arm.execute_pending_trajectory`, `operation.cancel` |
| Poses | `poses.list`, `poses.save`, `poses.execute`, `home.set` |
| Robot model | `robot.model.get`, `robot.model.download` |

Commands marked as **periodic** must be resent continuously
(10 Hz is recommended). A single message does not keep the deadman active or
maintain motion: once `command_timeout_sec` is exceeded, the backend stops the arm. To
move the mock for a set duration, use `ws_jog_for_seconds_demo` or
`ws_teleoperation_demo`.

### Outgoing messages (backend → client)

| Message | When |
| --- | --- |
| `ack` / `error` | In response to a non-periodic command, or on a protocol error. |
| `telemetry`, `constraints`, `robot_status`, `tool_camera` | Periodically, at `telemetry_hz`. |
| `teleoperation_status` | When the teleoperation state changes (or a watchdog is triggered). |
| `operation_status` | While an operation is active. |
| `planned_trajectory` | After planning a pose or a joint configuration. |
| `robot_model_chunk` | After `robot.model.download`, one chunk per message. |

The `tool_camera` image is a base64-encoded test PNG, allowing the
client to validate the video stream without hardware.

### Robot model

`robot.model.get` returns the manifest, and `robot.model.download` sends the
model as a chunked ZIP archive. The ZIP contains `robot.urdf` and the
`meshes/` directory with relative paths; the manifest includes the size, SHA-256, and
`model_id`, so the HMI can cache it.

## Configuration

Parameters are defined in [config/backend.yaml](config/backend.yaml) and
loaded by the launch files.

| Parameter | Default | Description |
| --- | --- | --- |
| `robot.type` | `mock` | Robot implementation to use. `real` is not yet implemented. |
| `websocket_host` | `127.0.0.1` | Network interface on which the WebSocket server listens. |
| `websocket_port` | `8765` | WebSocket server port. |
| `telemetry_hz` | `20.0` | Telemetry and status publication frequency. |
| `command_timeout_sec` | `0.5` | Watchdog timeout for periodic commands (deadman, jog, freedrive). |
| `initial_speed_percentage` | `25.0` | Initial speed limit, as a percentage. |
| `log_stats_period_sec` | `5.0` | Interval between incoming-message statistics summaries. `<= 0` disables it. |
| `log_payloads` | `true` | Include payload samples in the summary. |
| `log_payload_max_chars` | `180` | Maximum length of each payload sample. |
| `robot_model_max_size_bytes` | `52428800` | Maximum model ZIP size (50 MiB). |
| `robot_model_chunk_size` | `65536` | Size of each download chunk (64 KiB). |

The periodic summary prints messages per second, bytes per second, sources,
commands, errors, and connected clients.

To accept connections from another machine, change `websocket_host` to `0.0.0.0`.
The protocol has no authentication or encryption, so it should only be exposed on
a trusted network.

## ROS 2 interfaces

| Topic | Type | Direction |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState` | Publishes |
| `/centauro/hmi/state` | `std_msgs/String` (JSON telemetry) | Publishes |
| `/centauro/hmi/events` | `std_msgs/String` (JSON events) | Publishes |
| `/centauro/hmi/planned_tool_path` | `visualization_msgs/Marker` | Publishes |
| `/centauro/hmi/command` | `std_msgs/String` (JSON command) | Subscribes |

`/centauro/hmi/command` accepts the same payload as the WebSocket, allowing you to
test the backend from the command line without a WebSocket client.

## Launch files

| Launch | What it starts | Arguments |
| --- | --- | --- |
| `backend.launch.py` | Only the `hmi_backend` node with `config/backend.yaml`. | None |
| `mock_visualization.launch.py` | `robot_state_publisher` with the URDF and RViz2. | `rviz` (`true`) |
| `complete.launch.py` | Both of the above. | `rviz` (`true`) |

```bash
ros2 launch centauro_hmi_backend complete.launch.py
ros2 launch centauro_hmi_backend complete.launch.py rviz:=false
```

When planning a joint configuration, RViz2 shows the planned tool trajectory
in green under the `Planned tool path` display.

## Mock robot

An abstract six-joint arm defined in
[urdf/arm.urdf](urdf/arm.urdf). Each link's visual geometry uses its own STL in
`meshes/`, so the meshes can be replaced or refined independently. Once a
specific robot is integrated, the HMI will be able to load its actual model through
`robot.model.download`.

The mock simulates kinematics, applies the speed limit, respects the watchdogs,
and generates interpolated trajectories with the states `planning` →
`awaiting_confirmation` → `executing`.

## Code map

| Module | Responsibility |
| --- | --- |
| `node.py` | ROS 2 node: parameters, timers, telemetry publication, and command dispatch. |
| `transport.py` | WebSocket server: connections, broadcast, and unicast delivery. |
| `protocol.py` | Message envelope construction and encoding. |
| `robot.py` | `create_robot()` factory that selects the implementation based on `robot.type`. |
| `robots/mock_robot.py` | Simulated robot: state, commands, operations, and events. |
| `robot_model.py` | Packaging the URDF and meshes into a ZIP with a manifest and chunks. |
| `stats.py` | Aggregation of incoming-message statistics. |

## Example clients

The [centauro_hmi_ws_examples](../centauro_hmi_ws_examples/README.md) package
contains executable clients that work with both this backend in
`mock` mode and any backend that implements the same contract.

```bash
ros2 run centauro_hmi_ws_examples ws_protocol_smoke_test
ros2 run centauro_hmi_ws_examples ws_teleoperation_demo
ros2 run centauro_hmi_ws_examples ws_jog_for_seconds_demo 5 --velocities 0.3,0,0,0,0,0 --speed 25
ros2 run centauro_hmi_ws_examples ws_send_command_demo teleoperation.enable
ros2 run centauro_hmi_ws_examples ws_send_command_demo arm.joint_jog --payload '{"velocities":[0.3,0,0,0,0,0]}'
```

They require the backend to be running and the workspace to be sourced in that terminal.

## Tests

With `ROS_WS` pointing to your colcon workspace:

```bash
cd "$ROS_WS"
source /opt/ros/humble/setup.bash  # jazzy on Ubuntu 24.04
source "$ROS_WS/.venv/bin/activate"
python -m colcon build --symlink-install
source install/setup.bash
python -m colcon test --packages-select centauro_hmi_backend --event-handlers console_direct+
colcon test-result --verbose
```

Tests can also be run directly from the repository:

```bash
cd "$ROS_WS/src/centauro_hmi_backend"
source /opt/ros/humble/setup.bash  # jazzy on Ubuntu 24.04
source "$ROS_WS/.venv/bin/activate"
PYTHONPATH="$PWD/centauro_hmi_backend:$PYTHONPATH" python -m pytest -q centauro_hmi_backend/test
```
