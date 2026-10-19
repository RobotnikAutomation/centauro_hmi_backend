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
| `robot_description` | Once per WebSocket connection, only in mock mode. |

In mock WebSocket mode, `tool_camera` forwards frames from a looping camera
rosbag as Base64-encoded **raw ROS image pixels**, not PNG/JPEG. Mock WebRTC
mode sends video through the separate WebRTC server and emits no `tool_camera`
messages. See [Mock camera playback](#mock-camera-playback).

### Robot model

In mock mode, including when started directly with `ros2 run`, the backend
expands `robotnik_description/robots/rbkairos/rbkairos_plus.urdf.xacro`
with `namespace:=robot` and `prefix:=robot_` (the model's default UR10e arm).
It publishes the resulting URDF as `std_msgs/msg/String` on
`/robot/robot_description` with reliable, transient-local, depth-1 QoS, so
late subscribers receive it. Subscribe with transient-local durability, e.g.:

```bash
ros2 topic echo /robot/robot_description std_msgs/msg/String \
  --qos-durability transient_local --once
```

Each new WebSocket connection receives a `robot_description` message with
`payload.topic: "/robot/robot_description"` and `payload.data` containing the
same URDF XML. The topic and message are **not** published in real mode.
Mock mode requires `robotnik_description`, its model dependencies, and `xacro`
in the sourced host ROS environment; missing models fail startup explicitly.

`robot.model.get` returns the manifest, and `robot.model.download` sends the
same RB-KAIROS+ model as a chunked ZIP archive. The ZIP contains `robot.urdf` and the
`meshes/` directory with relative paths; the manifest includes the size, SHA-256, and
`model_id`, so the HMI can cache it. Referenced assets from `robotnik_description`,
`robotnik_sensors`, and `ur_description`, including COLLADA textures, are bundled.
The unbundled description retains its original `package://` asset URIs.
The existing mock visualization launch remains a separate, simplified arm-only
debug view on `/robot_description`; it does not publish the RB-KAIROS+ topic
or simulate the full robot's kinematics.

## Mock camera playback

Use **`ros2 launch`** to start the mock with camera playback. The launch reads
the backend ROS parameters from [config/backend.yaml](config/backend.yaml).
Its `robot_type`, `mock_camera_transport`, and `mock_rosbag_path` arguments
override `robot.type`, `mock.camera_transport`, and `mock.rosbag_path`,
respectively, and pass the same values to the backend node. The supplied YAML
still selects `robot.type: real`; explicitly select `mock` for these examples.

### WebSocket video

```bash
ros2 launch centauro_hmi_backend backend.launch.py \
  robot_type:=mock mock_camera_transport:=websocket \
  mock_rosbag_path:=/home/robot/teleop_ws/src/front_camera_10s
```

This starts `ros2 bag play --loop`, restricted to
`/robot/front_rgbd_camera/color/image_raw` (`sensor_msgs/msg/Image`).
The backend subscribes to that topic and publishes `tool_camera` JSON messages
on the existing `ws://127.0.0.1:8765` connection. **No WebRTC node is started.**
Mock telemetry, constraints, robot status, and commands continue as before.

Each camera payload contains `stream_id: "front_camera"`, `topic`,
`encoding: "base64"`, `format: "ros_image"`, `data`, `image_encoding`,
`width`, `height`, `step`, `is_bigendian`, the recorded header `timestamp`,
and a monotonically increasing `sequence`. Decode `data` as raw pixels using
the supplied ROS encoding and row stride; do not feed it to a PNG decoder.
The [WebSocket contract](docs/WEBSOCKET_PROTOCOL.md) includes an example.

Only the newest received frame is sent per `telemetry_hz` tick (20 Hz by
default); intermediate frames are dropped rather than queued. A frame is
never resent on subsequent ticks without a new ROS image. If the topic stops,
camera messages stop; there is no test-PNG fallback. The supplied recording
contains 594 frames over about 9.94 seconds (roughly 60 Hz).

### WebRTC video

Build/source `webrtc_video_stream` and `webrtc_video_stream_msgs` from the
sibling [webrtc_video_stream repository](../../webrtc_video_stream/README.md),
including its host Python/media dependencies:

```bash
cd /home/robot/teleop_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  webrtc_video_stream_msgs webrtc_video_stream centauro_hmi_backend
source install/setup.bash
ros2 launch centauro_hmi_backend backend.launch.py \
  robot_type:=mock mock_camera_transport:=webrtc \
  mock_rosbag_path:=/home/robot/teleop_ws/src/front_camera_10s
```

The same looped bag player starts, plus
`webrtc_video_stream/webrtc_video_stream_node.py`. The backend does **not**
subscribe to mock images or emit `tool_camera` messages in this mode. Its
WebSocket connection remains available for commands and telemetry.

[config/mock_camera_webrtc.yaml](config/mock_camera_webrtc.yaml) registers
the single topic source `front_camera`. The WebRTC server listens on
`https://<backend-host>:8081`, with VP8 video and audio disabled:

- `GET /streams` lists `front_camera`.
- `POST /offer` handles WebRTC SDP negotiation; select the `front_camera`
  stream using the WebRTC package's client protocol.
- This is WebRTC video, not an MJPEG/HTTP video URL. Follow the sibling
  package's client documentation for receiving the media track.
- The existing WebRTC package uses its installed development TLS certificate
  and key. Trust that certificate for local testing; use deployment-appropriate
  certificates and network controls before exposing this server.
- **A running STUN/TURN service is also required by the existing WebRTC
  package.** Its [ICE configuration](../../webrtc_video_stream/webrtc_video_stream/src/webrtc_video_stream/server/config.py)
  points to `127.0.0.1:3478`. Start the sibling package's documented
  [Coturn setup](../../webrtc_video_stream/README.md#testing-with-docker-example-setup),
  or an equivalent host Coturn service with matching authentication, before
  connecting a video client. The backend launch starts neither Coturn nor its
  Docker stack. Without this service, `/streams` can work while `POST /offer`
  stalls or fails and no video reaches the client. For remote deployments,
  configure reachable ICE addresses in that package rather than loopback
  addresses. No changes are made to the sibling package by this integration.
- The existing topic-stream implementation converts images to BGR8, resizes
  them to 800x600, and sends a WebRTC video track (normally 30 FPS), independently
  of the backend's `telemetry_hz`.

### Configuration and lifecycle

Both launch files support the same overrides; for visualization, replace
`backend.launch.py` with `complete.launch.py` (optionally `rviz:=false`).
To make the selection persistent without command-line overrides, edit
[config/backend.yaml](config/backend.yaml):

```yaml
centauro_hmi_backend:
  ros__parameters:
    robot.type: mock
    mock.camera_transport: webrtc  # or websocket
    mock.rosbag_path: /home/robot/teleop_ws/src/front_camera_10s
```

These are startup settings, not a live transport switch: restart the launch
after changing them. `ros2 param set` cannot change the two read-only mock
camera parameters. With a non-symlink install, rebuild after editing the source
configuration.

The launch rejects a missing bag/metadata file or metadata without a nonempty
Image topic at the required camera topic. Playback errors appear in the launch
output. Exiting either camera process or the mock backend shuts down the mock
launch instead of leaving a backend running without its selected video source.
Ctrl+C stops the player, optional WebRTC node, and backend together.

Playback uses the recorded timing at 1x speed and loops at end-of-file.
Recorded image header timestamps jump back on every loop; WebSocket sequence
numbers keep increasing until the backend restarts. No `/clock` is published
and `use_sim_time` is not enabled. Only the camera topic is replayed, even if
another bag contains control/joint topics. A small read-ahead queue (10 messages)
limits the memory used for these uncompressed camera frames.

**Do not run mock playback in a ROS domain connected to a real robot camera**:
it publishes on the real camera's topic name. Use a separate `ROS_DOMAIN_ID`
for development, set consistently for the launch and any ROS inspection tools.

Running `ros2 run centauro_hmi_backend hmi_backend` alone does not start sibling
processes: in mock WebSocket mode it can forward an externally published camera
topic, but it neither plays the bag nor starts WebRTC. Use the launch files for
automatic playback/transport startup. In `robot.type: real`, the launch starts
neither the mock player nor the WebRTC node; existing real-image subscriptions
and robot status forwarding are unchanged.

## Configuration

Parameters are defined in [config/backend.yaml](config/backend.yaml) and
loaded by the launch files.

| Parameter | Default | Description |
| --- | --- | --- |
| `robot.type` | `mock` | Robot adapter to use: `mock` or `real`. Real mode requires `robotnik_servo` and its robot interfaces to be running. |
| `mock.camera_transport` | `websocket` | Mock camera delivery: `websocket` or `webrtc`. Startup-only; restart the launch to change it. Does not alter real-mode camera delivery. |
| `mock.rosbag_path` | `/home/robot/teleop_ws/src/front_camera_10s` | Mock camera rosbag directory containing `metadata.yaml` and its data files. Startup-only; the bag is not copied into the package. |
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
