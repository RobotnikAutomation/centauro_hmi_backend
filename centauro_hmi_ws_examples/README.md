# centauro_hmi_ws_examples

Clientes WebSocket ejecutables que sirven para probar y entender el contrato de
la HMI sin escribir código. Funcionan contra el backend en modo
`robot.type: mock` y contra cualquier backend que implemente el mismo contrato,
incluido uno conectado a un robot real.

Si es la primera vez que ves este repositorio, empieza por el
[README raíz](../README.md).

## Antes de empezar

1. El backend tiene que estar arrancado:
   `ros2 launch centauro_hmi_backend complete.launch.py`
2. En la terminal donde lances los ejemplos:
   `source /opt/ros/jazzy/setup.bash && source "$ROS_WS/install/setup.bash"`
   (`ROS_WS` es la ruta de tu workspace colcon).

## Orden recomendado

Para alguien que ve esto por primera vez:

```bash
# 1. ¿Hay conexión? No mueve el robot.
ros2 run centauro_hmi_ws_examples ws_protocol_smoke_test

# 2. Ciclo completo de teleoperación: habilitar, velocidad, jog, deshabilitar.
ros2 run centauro_hmi_ws_examples ws_teleoperation_demo

# 3. Planificar una configuración articular (no la ejecuta).
ros2 run centauro_hmi_ws_examples ws_plan_joint_trajectory_demo --positions 0.2,-1.2,0.4,-1.4,0,0.2

# 4. Confirmar y ejecutar la trayectoria que quedó pendiente en el paso 3.
ros2 run centauro_hmi_ws_examples ws_execute_pending_trajectory_demo

# 5. Planificar y cancelar una operación.
ros2 run centauro_hmi_ws_examples ws_cancel_operation_demo
```

Con RViz2 abierto (`complete.launch.py`) se ve el brazo moverse en los pasos 2
y 4, y la trayectoria prevista en verde en el paso 3.

## Clientes disponibles

| Ejecutable | Qué hace | Argumentos propios |
| --- | --- | --- |
| `ws_protocol_smoke_test` | Espera un mensaje de telemetría y pide el catálogo de poses. No mueve el robot. | — |
| `ws_send_command_demo` | Envía un comando suelto y muestra la respuesta. | `name` (posicional), `--payload` (JSON, `{}`) |
| `ws_teleoperation_demo` | Secuencia completa: habilita teleoperación, fija velocidad, hace jog y deshabilita. | — |
| `ws_jog_for_seconds_demo` | Mantiene el jog articular durante N segundos, reenviando deadman y jog. | `seconds` (posicional), `--velocities` (`0.3,0,0,0,0,0`), `--speed` (`25.0`), `--open-timeout` (`5.0`) |
| `ws_plan_joint_trajectory_demo` | Planifica una trayectoria a una configuración articular y muestra la validación. | `--positions` (6 valores), `--operation-id`, `--execute` |
| `ws_execute_pending_trajectory_demo` | Ejecuta la trayectoria pendiente de confirmación y espera a que termine. | — |
| `ws_cancel_operation_demo` | Planifica una trayectoria y luego la cancela. | `--operation-id` |
| `ws_robot_model_demo` | Descarga el modelo del robot por fragmentos, verifica el SHA-256 y lo guarda. | `--output` (`robot_model.zip`) |

Todos aceptan además:

| Opción | Defecto | Descripción |
| --- | --- | --- |
| `--url` | `ws://127.0.0.1:8765` | URL del backend. |
| `--timeout` | `5.0` | Segundos de espera por conexión y respuesta. En `ws_jog_for_seconds_demo` se llama `--open-timeout`. |

`--operation-id` es opcional: si no se indica, el cliente genera uno único.

## Notas de uso

**Comandos periódicos.** `teleoperation.deadman`, `teleoperation.freedrive`,
`arm.joint_jog` y `arm.cartesian_jog` hay que reenviarlos de forma continua. Con
`ws_send_command_demo` un envío aislado no produce movimiento visible: el
watchdog del backend detiene el brazo a los 0,5 s. Para eso están
`ws_jog_for_seconds_demo` y `ws_teleoperation_demo`.

**Planificar frente a ejecutar.** `ws_plan_joint_trajectory_demo` solo planifica
y verifica; la trayectoria queda pendiente de confirmación. Se ejecuta con
`ws_execute_pending_trajectory_demo`, o directamente añadiendo `--execute` a la
planificación. Si se envía otra planificación, la anterior se cancela
automáticamente. Úsalo con precaución contra un robot real.

## Ejemplos sueltos

```bash
ros2 run centauro_hmi_ws_examples ws_send_command_demo teleoperation.enable
ros2 run centauro_hmi_ws_examples ws_send_command_demo teleoperation.set_speed --payload '{"percentage":40}'
ros2 run centauro_hmi_ws_examples ws_jog_for_seconds_demo 5 --velocities 0.3,0,0,0,0,0 --speed 25
ros2 run centauro_hmi_ws_examples ws_robot_model_demo --output robot_model.zip
ros2 run centauro_hmi_ws_examples ws_protocol_smoke_test --url ws://192.168.1.50:8765
```

## Escribir tu propio cliente

`centauro_hmi_ws_examples/common/client.py` contiene las utilidades que usan
todos los ejemplos y es el punto de partida más corto para un cliente propio:

- `connect(url, timeout)` — abre la conexión.
- `send_command(socket, name, **payload)` — envía un comando y devuelve su `request_id`.
- `receive_until(socket, predicate, timeout)` — consume mensajes hasta cumplir una condición.
- `request_ack(socket, name, timeout, **payload)` — envía y espera el `ack` correspondiente.
- `require_accepted(ack)` — valida el `ack` y lanza excepción si fue rechazado.

El contrato completo de mensajes está en
[docs/WEBSOCKET_PROTOCOL.md](../centauro_hmi_backend/docs/WEBSOCKET_PROTOCOL.md).
