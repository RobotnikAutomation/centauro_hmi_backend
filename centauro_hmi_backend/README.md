# centauro_hmi_backend

Nodo ROS 2 (Jazzy) que expone el contrato de teleoperación de la HMI CENTAURO
sobre WebSocket. Mantiene separados el transporte, el contrato de mensajes y la
implementación del robot.

Actualmente solo está implementado `robot.type: mock`: un brazo simulado de seis
articulaciones, ligero, sin Gazebo ni MoveIt. El valor `real` está reservado
para la futura integración con el robot físico y **hoy lanza un error al
arrancar**.

Si es la primera vez que ves este repositorio, empieza por el
[README raíz](../README.md).

## Arranque rápido

Con el workspace ya compilado y cargado (ver [README raíz](../README.md)):

```bash
ros2 run centauro_hmi_backend hmi_backend
```

El servidor WebSocket escucha en `ws://127.0.0.1:8765`.

Normalmente conviene arrancarlo con su fichero de configuración:

```bash
ros2 launch centauro_hmi_backend backend.launch.py
```

## Contrato WebSocket

El contrato completo (payloads, respuestas, frecuencias y watchdogs) está en
[docs/WEBSOCKET_PROTOCOL.md](docs/WEBSOCKET_PROTOCOL.md). Resumen:

Cada mensaje es JSON con esta envolvente:

```json
{"type":"command", "request_id":"optional-id", "timestamp":0.0,
 "payload":{"name":"teleoperation.enable"}}
```

### Comandos aceptados (cliente → backend)

| Categoría | Comandos |
| --- | --- |
| Teleoperación | `teleoperation.enable`, `teleoperation.disable`, `teleoperation.set_mode`, `teleoperation.set_speed` |
| Teleoperación (periódicos) | `teleoperation.deadman`, `teleoperation.freedrive` |
| Movimiento continuo (periódicos) | `arm.joint_jog`, `arm.cartesian_jog` |
| Planificación y ejecución | `arm.plan_to_pose`, `arm.plan_to_joint_configuration`, `arm.execute_trajectory`, `arm.execute_pending_trajectory`, `operation.cancel` |
| Poses | `poses.list`, `poses.save`, `poses.execute`, `home.set` |
| Modelo del robot | `robot.model.get`, `robot.model.download` |

Los comandos marcados como **periódicos** hay que reenviarlos de forma continua
(se recomiendan 10 Hz). Un envío aislado no mantiene el deadman ni el
movimiento: al superarse `command_timeout_sec` el backend detiene el brazo. Para
mover el mock durante un intervalo usa `ws_jog_for_seconds_demo` o
`ws_teleoperation_demo`.

### Mensajes emitidos (backend → cliente)

| Mensaje | Cuándo |
| --- | --- |
| `ack` / `error` | Respuesta a un comando no periódico, o error de protocolo. |
| `telemetry`, `constraints`, `robot_status`, `tool_camera` | Periódicos, a `telemetry_hz`. |
| `teleoperation_status` | Al cambiar el estado de teleoperación (o al saltar un watchdog). |
| `operation_status` | Mientras hay una operación activa. |
| `planned_trajectory` | Tras planificar una pose o una configuración articular. |
| `robot_model_chunk` | Tras `robot.model.download`, un fragmento por mensaje. |

La imagen de `tool_camera` es un PNG de prueba codificado en base64, para que el
cliente valide el flujo de vídeo sin hardware.

### Modelo del robot

`robot.model.get` devuelve el manifiesto y `robot.model.download` envía el
modelo como ZIP fragmentado. El ZIP contiene `robot.urdf` y la carpeta
`meshes/` con rutas relativas; el manifiesto incluye tamaño, SHA-256 y
`model_id`, de modo que la HMI puede cachearlo.

## Configuración

Los parámetros se definen en [config/backend.yaml](config/backend.yaml) y los
cargan los launch files.

| Parámetro | Defecto | Descripción |
| --- | --- | --- |
| `robot.type` | `mock` | Implementación de robot a usar. `real` aún no está implementado. |
| `websocket_host` | `127.0.0.1` | Interfaz de escucha del servidor WebSocket. |
| `websocket_port` | `8765` | Puerto del servidor WebSocket. |
| `telemetry_hz` | `20.0` | Frecuencia de publicación de telemetría y estados. |
| `command_timeout_sec` | `0.5` | Watchdog de los comandos periódicos (deadman, jog, freedrive). |
| `initial_speed_percentage` | `25.0` | Límite de velocidad inicial, en porcentaje. |
| `log_stats_period_sec` | `5.0` | Período del resumen de estadísticas de entrada. `<= 0` lo desactiva. |
| `log_payloads` | `true` | Incluir muestras de payload en el resumen. |
| `log_payload_max_chars` | `180` | Longitud máxima de cada muestra de payload. |
| `robot_model_max_size_bytes` | `52428800` | Tamaño máximo del ZIP del modelo (50 MiB). |
| `robot_model_chunk_size` | `65536` | Tamaño de cada fragmento de descarga (64 KiB). |

El resumen periódico imprime mensajes por segundo, bytes por segundo, fuentes,
comandos, errores y clientes conectados.

Para escuchar desde otra máquina hay que cambiar `websocket_host` a `0.0.0.0`.
El protocolo no tiene autenticación ni cifrado, así que solo debe exponerse en
una red de confianza.

## Interfaces ROS 2

| Topic | Tipo | Dirección |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState` | Publica |
| `/centauro/hmi/state` | `std_msgs/String` (telemetría JSON) | Publica |
| `/centauro/hmi/events` | `std_msgs/String` (eventos JSON) | Publica |
| `/centauro/hmi/planned_tool_path` | `visualization_msgs/Marker` | Publica |
| `/centauro/hmi/command` | `std_msgs/String` (comando JSON) | Suscribe |

`/centauro/hmi/command` acepta el mismo payload que el WebSocket, lo que permite
probar el backend desde la línea de comandos sin cliente WebSocket.

## Launch files

| Launch | Qué arranca | Argumentos |
| --- | --- | --- |
| `backend.launch.py` | Solo el nodo `hmi_backend` con `config/backend.yaml`. | — |
| `mock_visualization.launch.py` | `robot_state_publisher` con el URDF y RViz2. | `rviz` (`true`) |
| `complete.launch.py` | Los dos anteriores. | `rviz` (`true`) |

```bash
ros2 launch centauro_hmi_backend complete.launch.py
ros2 launch centauro_hmi_backend complete.launch.py rviz:=false
```

Al planificar una configuración articular, RViz2 muestra en verde la trayectoria
prevista de la herramienta bajo el display `Planned tool path`.

## Robot mock

Brazo abstracto de seis articulaciones definido en
[urdf/arm.urdf](urdf/arm.urdf). Cada enlace visual usa su propio STL en
`meshes/`, para poder sustituirlos o refinarlos por separado. Cuando se integre
un robot concreto, la HMI podrá cargar su modelo real a través de
`robot.model.download`.

El mock simula cinemática, aplica el límite de velocidad, respeta los watchdogs
y genera trayectorias interpoladas con estados `planning` →
`awaiting_confirmation` → `executing`.

## Mapa del código

| Módulo | Responsabilidad |
| --- | --- |
| `node.py` | Nodo ROS 2: parámetros, temporizadores, publicación de telemetría y despacho de comandos. |
| `transport.py` | Servidor WebSocket: conexiones, difusión y envío unicast. |
| `protocol.py` | Construcción y codificación de la envolvente de mensajes. |
| `robot.py` | Fábrica `create_robot()` que selecciona la implementación según `robot.type`. |
| `robots/mock_robot.py` | Robot simulado: estado, comandos, operaciones y eventos. |
| `robot_model.py` | Empaquetado del URDF y las mallas en un ZIP con manifiesto y fragmentos. |
| `stats.py` | Agregación de estadísticas de los mensajes de entrada. |

## Clientes de ejemplo

El paquete [centauro_hmi_ws_examples](../centauro_hmi_ws_examples/README.md)
contiene clientes ejecutables que funcionan tanto contra este backend en modo
`mock` como contra cualquier backend que implemente el mismo contrato.

```bash
ros2 run centauro_hmi_ws_examples ws_protocol_smoke_test
ros2 run centauro_hmi_ws_examples ws_teleoperation_demo
ros2 run centauro_hmi_ws_examples ws_jog_for_seconds_demo 5 --velocities 0.3,0,0,0,0,0 --speed 25
ros2 run centauro_hmi_ws_examples ws_send_command_demo teleoperation.enable
ros2 run centauro_hmi_ws_examples ws_send_command_demo arm.joint_jog --payload '{"velocities":[0.3,0,0,0,0,0]}'
```

Requieren el backend arrancado y el workspace cargado en esa terminal.

## Tests

Con `ROS_WS` apuntando a tu workspace colcon:

```bash
cd "$ROS_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select centauro_hmi_backend --event-handlers console_direct+
colcon test-result --verbose
```

También se pueden ejecutar directamente desde el repositorio:

```bash
cd "$ROS_WS/src/centauro_hmi_backend"
source /opt/ros/jazzy/setup.bash
PYTHONPATH="$PWD/centauro_hmi_backend:$PYTHONPATH" pytest -q centauro_hmi_backend/test
```
