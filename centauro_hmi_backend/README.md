# centauro_hmi_backend

ROS 2 (Jazzy) node that exposes the CENTAURO HMI teleoperation contract
over WebSocket. It keeps the transport, message contract, and robot
implementation separate.

`robot.type: mock` selects a lightweight simulated six-joint arm, without Gazebo
or MoveIt. `robot.type: real` connects to the `robotnik_servo` teleoperation
node for deadman, freedrive, mode, jog commands, and joint-state feedback. The
real adapter does not provide planning, trajectory execution, pose management,
camera streaming, or full robot status; see [Real robot mode](#real-robot-mode)
for details.

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

When `robot.type: real`, use `ros2 launch` so the launch file waits for the
configured teleoperation node's parameter services and
`default_velocity_percentage` before starting the backend. Running the backend
directly with `ros2 run` bypasses that launch-time wait.

## Required dependencies

Install system tools and Python packages:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
    python3-websockets python3-yaml python3-pytest
```

From the root of the colcon workspace, resolve the ROS dependencies declared in
this package's `package.xml`:

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # Run once per machine; skip if rosdep is already initialized.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

This package uses `rclpy`, `rcl_interfaces`, standard/control/image messages,
`robotnik_battery_msgs`, `robotnik_hardware_msgs`, `robotnik_safety_msgs`,
`ur_dashboard_msgs`, `ur_msgs`, `robot_bringup`, launch, `robot_state_publisher`, and RViz2. The
real adapter additionally requires the custom `robotnik_servo` package and its
robot driver/controller dependencies to be built in this workspace or installed in a sourced overlay.
That package is not installed by `apt` through this README; include its source
workspace before building/running this backend. The example-client package also
uses `python3-websockets`, already included above.

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
| Continuous motion (periodic) | `arm.joint_jog`, `arm.cartesian_jog`, `base.joystick` |
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
| `robot.type` | `mock` | Robot adapter to use: `mock` or `real`. Real mode requires `robotnik_servo` and its robot interfaces to be running. |
| `websocket_host` | `127.0.0.1` | Network interface on which the WebSocket server listens. |
| `websocket_port` | `8765` | WebSocket server port. |
| `websocket_ping_timeout_sec` | `1800.0` | Time allowed for a WebSocket client to reply to a keepalive ping before the connection is closed (30 minutes). |
| `telemetry_hz` | `20.0` | Telemetry and status publication frequency. |
| `base_cmd_vel_hz` | `100.0` | Real-mode base velocity publication rate; mock-mode base integration and TF publication rate. |
| `command_timeout_sec` | `0.5` | Watchdog timeout for periodic commands (deadman, jog, freedrive, base joystick). |
| `initial_speed_percentage` | `25.0` | Initial mock speed limit, as a percentage. Real mode reads the teleoperation node's `default_velocity_percentage` parameter. |
| `robot.joint_states_topic` | `/robot/joint_states` | Joint-state feedback topic used by the real adapter. |
| `robot.base_cmd_vel_topic` | `/robot/pad_teleop/cmd_vel` | Real-mode planar base velocity output (`geometry_msgs/Twist`). |
| `robot.pad_model` | `JOY_CONFIG` environment variable, or `ps5` | Pad profile whose `Movement` speed limits are loaded from `robot_bringup/config/base/pad/<pad_model>.yaml`. |
| `robot.teleoperation_node_name` | `/robot/arm_teleoperation_node` in the node defaults; configurable in `config/backend.yaml` | Fully qualified teleoperation node name queried for `default_velocity_percentage`. |
| `robot.image_streams` | `front_camera=/robot/front_rgbd_camera/color/image_raw` | List of `stream_name=/image/topic` entries; add entries to forward multiple image streams. |
| `robot.battery_topic` | `/robot/battery_estimator/data` | Battery status source; `level` is sent as `battery_percentage`. |
| `robot.base_status_topic` | `/robot/robotnik_base_hw_monitor/status` | Base motor status source. |
| `robot.robot_mode_topic` | `/robot/arm/io_and_status_controller/robot_mode` | Arm robot-mode source. |
| `robot.robot_program_running_topic` | `/robot/arm/io_and_status_controller/robot_program_running` | Arm program-running source. |
| `robot.safety_mode_topic` | `/robot/arm/io_and_status_controller/safety_mode` | Arm safety-mode source. |
| `robot.tool_data_topic` | `/robot/arm/io_and_status_controller/tool_data` | Tool status source. |
| `robot.safety_module_status_topic` | `/robot/safety_module/status` | Robot safety-module status source. |
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
| `/robot/pad_teleop/cmd_vel` | `geometry_msgs/Twist` | Publishes in real mode at `base_cmd_vel_hz` |
| `/tf` | `tf2_msgs/TFMessage` | Broadcasts `base_link -> sim_base` in mock mode at `base_cmd_vel_hz` |

`/centauro/hmi/command` accepts the same payload as the WebSocket, allowing you to
test the backend from the command line without a WebSocket client.

## Real robot mode

Set `robot.type: real` to connect the HMI command contract to `robotnik_servo`.
The backend publishes deadman, freedrive, mode, speed adjustment, joint-jog, and
Cartesian-jog inputs on the teleoperation node's `/robot/arm/servo/*` interfaces, calls
its enable/disable services, and reads joint feedback from
`robot.joint_states_topic` (default `/robot/joint_states`). The HMI-facing
speed starts from the live `default_velocity_percentage` ROS parameter on the
node configured by `robot.teleoperation_node_name`. The backend requests it
asynchronously from that node's parameter services, retries while they are
unavailable, and converts the returned `0.0`-to-`1.0` fraction to a percentage.
Teleoperation cannot be enabled until the query succeeds. Subsequent HMI speed
commands adjust from that value. The `/joint_states` topic relays feedback after
the first hardware sample.

The real adapter forwards the latest battery, base, arm, tool, and safety
messages in `robot_status`; `battery_percentage` comes from the battery
message's `level` field. Each `robot.image_streams` entry creates a separate
image subscription. Each new frame is sent once as a `tool_camera` message
with its `stream_id`, per-stream sequence, source topic, base64-encoded raw pixel
bytes, image encoding, dimensions, row step, endianness, and source timestamp. Add another
`stream_name=/image/topic` entry in `backend.yaml` to forward another stream.

Real-mode `telemetry` and `teleoperation_status` use the selected arm mode
reported on `/robot/arm/servo/teleoperation_mode/state`; mode integers map to
`cartesian_tcp` (`0`), `cartesian_base` (`1`), and `joint` (`2`).

Joint and Cartesian command values are reduced to direction inputs (`-1`, `0`,
or `1`); the teleoperation node applies its configured velocity limits and speed
percentage. The backend stops jog input and releases deadman/freedrive when
their periodic HMI commands time out.

The real adapter also accepts periodic `base.joystick` commands. The
`twist.linear` and `twist.angular` arrays contain normalized axes in `[-1, 1]`;
only linear x/y and angular z are supported. Their limits are loaded from
`Movement.max_linear_speed` and `Movement.max_angular_speed` in the selected
pad profile under the dynamically resolved `robot_bringup` package share path.
The scaled values are published as `geometry_msgs/Twist` on
`robot.base_cmd_vel_topic` at `base_cmd_vel_hz` (100 Hz by default). If updates
stop for `command_timeout_sec`, the adapter publishes zero velocity until fresh
joystick input arrives.

Planning, trajectory execution/cancellation, pose management, home-pose storage,
robot-model downloads, camera streaming, robot battery/base/tool status, and
actual teleoperation safety/constraint reporting are not provided by this adapter. Such
commands are rejected where applicable; mock camera/status/model outputs are
not sent in real mode. The backend also omits the mock constraints message. The
teleoperation safety field is reported as `unknown` because the teleoperation
node does not expose it through these interfaces.

## Launch files

| Launch | What it starts | Arguments |
| --- | --- | --- |
| `backend.launch.py` | Starts `hmi_backend` with `config/backend.yaml`; in real mode it waits for the configured teleoperation node's parameter service and `default_velocity_percentage` parameter first. | None |
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

### Mock base motion

With `robot.type: mock`, the same periodic `base.joystick` command used by the
real adapter moves a simulated base. The backend broadcasts the dynamic TF
`base_link -> sim_base` at `base_cmd_vel_hz` (100 Hz by default), including when
stationary. It starts at the origin with zero yaw. Linear x/y move in the
simulated base's local frame; angular z rotates it about the vertical axis.
Translation z and rotation about x/y remain zero. Integration uses elapsed ROS
time, independently of the arm's telemetry rate.

Axes must be finite, normalized to `[-1, 1]`, and planar. Both adapters use the
same validation and speed limits from the selected `robot.pad_model` profile.
Base commands are independent of arm enable, deadman, freedrive, and speed
percentage, matching the real adapter. Send fresh commands at about 10 Hz:
after `command_timeout_sec` without input, the base stops and holds its pose
until fresh input arrives. A zero joystick command stops it immediately.

The mock does **not** publish or subscribe to `/robot/pad_teleop/cmd_vel`; it
consumes the HMI command through WebSocket or `/centauro/hmi/command`. The
`sim_base` frame is separate from the arm model, so it does not move the existing
arm or change its TF tree.

For example, start the mock backend, then inspect its TF:

```bash
ros2 run centauro_hmi_backend hmi_backend --ros-args -p robot.type:=mock
ros2 run tf2_ros tf2_echo base_link sim_base
```

From another terminal, publish the same HMI payload at 10 Hz (stop with Ctrl+C):

```bash
ros2 topic pub -r 10 /centauro/hmi/command std_msgs/msg/String \
  '{data: "{\"type\":\"command\",\"payload\":{\"name\":\"base.joystick\",\"twist\":{\"linear\":[0.5,-0.25,0.0],\"angular\":[0.0,0.0,0.4]}}}"}'
```

## Code map

| Module | Responsibility |
| --- | --- |
| `node.py` | ROS 2 node: parameters, timers, telemetry publication, and command dispatch. |
| `transport.py` | WebSocket server: connections, broadcast, and unicast delivery. |
| `protocol.py` | Message envelope construction and encoding. |
| `robot.py` | `create_robot()` factory that selects the implementation based on `robot.type`. |
| `base_motion.py` | Shared planar joystick validation and velocity scaling for mock and real adapters. |
| `robots/mock_robot.py` | Simulated robot: state, commands, operations, and events. |
| `robots/real_robot.py` | Adapter for the `robotnik_servo` teleoperation node. |
| `wait_for_teleoperation.py` | Launch helper that waits for the teleoperation node's parameter service. |
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
