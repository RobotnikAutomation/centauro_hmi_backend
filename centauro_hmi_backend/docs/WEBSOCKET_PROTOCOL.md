# Contrato WebSocket HMI ↔ backend del robot

El endpoint WebSocket, el transporte seguro y la autenticación se definen en
cada despliegue. La HMI no expone ni consume interfaces ROS 2 directamente.

## Mensaje HMI → backend

Todos los comandos son objetos JSON. El campo obligatorio es `payload.name`.
`request_id` permite correlacionar la respuesta con la solicitud.

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

El backend responde, cuando la conexión sigue abierta, con un `ack`. El campo
`accepted` indica si el comando ha sido aceptado; `result` puede incluir el
resultado de una consulta y `error` el motivo de un rechazo:

```json
{
  "type": "ack",
  "request_id": "c85c351a-84c0-4d30-a318-77ddf7c6307e",
  "timestamp": 1730000000.1,
  "payload": {"name": "teleoperation.enable", "accepted": true}
}
```

Si el JSON no es un objeto, `payload` no es un objeto o falta `payload.name`,
se devuelve `type: "error"` con `payload.code: "invalid_command"`.

## Comandos HMI → backend

### Comandos no periódicos

| ID Excel | Comando | Payload adicional | Respuesta inmediata | Efecto producido |
| --- | --- | --- | --- | --- |
| — | `teleoperation.enable` | — | `ack`, con `accepted: true/false`. | Habilita la sesión de teleoperación si el estado y la seguridad del robot lo permiten; a partir de entonces puede aceptar deadman y jog. No tiene un ID H2B equivalente en el Excel. |
| H2B-001 | `teleoperation.disable` | — | `ack`, con `accepted: true/false`. | Deshabilita la teleoperación, revoca el deadman y detiene cualquier jog activo. |
| H2B-005 | `teleoperation.set_mode` | `mode: string` | `ack`, con `accepted: true/false`. | Cambia el modo de teleoperación que interpreta los comandos posteriores. |
| H2B-006 | `teleoperation.set_speed` | `percentage: number` | `ack`, con `accepted: true/false`. | Actualiza el límite máximo de velocidad aplicable a los movimientos de teleoperación. |
| H2B-008 | `arm.plan_to_pose` | `operation_id: string`, `pose: object`, `frame_id: string`, `speed_percentage: number` opcional | `ack`, con `accepted: true/false` y `operation_id`. | Inicia la planificación de una trayectoria a la pose objetivo, sin mover el robot. |
| H2B-015 | `arm.plan_to_joint_configuration` | `operation_id: string`, `positions: [q0, q1, q2, q3, q4, q5]` | `ack`, con `accepted: true/false` y `operation_id`. | Inicia la planificación de una trayectoria a una configuración articular, sin mover el robot. |
| H2B-009 | `arm.execute_trajectory` | `operation_id: string`, `trajectory_id: string` | `ack`, con `accepted: true/false` y `operation_id`. | Ejecuta una trayectoria previamente planificada y confirmada por el operador. |
| — | `arm.execute_pending_trajectory` | — | `ack`, con `accepted: true/false` y los IDs en `result`. | Ejecuta la única trayectoria pendiente de confirmación, si existe. |
| — | `robot.model.get` | — | `ack` con el manifiesto en `result`. | Describe el modelo URDF disponible y sus recursos, sin transferir sus bytes. |
| — | `robot.model.download` | — | `ack` con el manifiesto en `result`, seguido de `robot_model_chunk`. | Descarga el ZIP del modelo en fragmentos Base64. |
| H2B-010 | `poses.list`, `poses.save`, `poses.execute` | Según la operación; `poses.save` usa `pose_id: string` opcional, `pose_name: string` y `pose: object` | `ack`, con `accepted: true/false`; `poses.list` incluye las poses en `result`. | Consulta, valida/guarda o ejecuta poses predefinidas. Corresponde a las acciones de `pose_management`. |
| H2B-011 | `home.set` | `pose: object`, `frame_id: string` | `ack`, con `accepted: true/false`. | Valida y guarda de forma persistente la pose Home del robot. |
| H2B-014 | `operation.cancel` | `operation_id: string` | `ack`, con `accepted: true/false`. | Cancela de forma segura una planificación o ejecución activa identificada. |

### Comandos periódicos

Los comandos de esta sección no generan `ack` cuando son aceptados. El
backend solo responde con `error` si no puede procesarlos.

| ID Excel | Comando | Payload adicional | Cadencia | Efecto producido | Al dejar de enviarse |
| --- | --- | --- | --- | --- | --- |
| H2B-002 | `teleoperation.deadman` | `active: boolean` | **10 Hz** recomendado; siempre menor que `0.5 s`. | Mantiene la autorización de movimiento del operador mientras `active` sea `true`. | Tras `0.5 s` por defecto, revoca la autorización, detiene el movimiento y emite `teleoperation_status` con `active: false` y `reason: "deadman_timeout"`. |
| H2B-003 | `arm.joint_jog` | `velocities: [v0, v1, v2, v3, v4, v5]` | **10 Hz** mientras haya movimiento. | Solicita velocidades para cada articulación, sujetas a límites y restricciones de seguridad. | Tras `0.5 s` por defecto, pone las velocidades articulares a cero. La sesión y el deadman permanecen activos si este último continúa llegando. |
| H2B-004 | `arm.cartesian_jog` | `frame_id: string`, `twist: {"linear": [x, y, z], "angular": [rx, ry, rz]}` | **10 Hz** mientras haya movimiento. | Solicita una velocidad cartesiana respecto al frame indicado, sujeta a límites y restricciones de seguridad. | Tras `0.5 s` por defecto, pone las velocidades a cero. La sesión y el deadman permanecen activos si este último continúa llegando. |
| H2B-007 | `teleoperation.freedrive` | `active: boolean` | **10 Hz** mientras `active` sea `true`; siempre menor que `0.5 s`. | Mantiene el freedrive activo mientras el comando periódico siga llegando y `active` sea `true`. | Tras `0.5 s` por defecto, desactiva el freedrive y emite `teleoperation_status` con `freedrive: false` y `reason: "freedrive_timeout"`. |

Ejemplo de jog articular:

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

## Mensajes backend → HMI

Los mensajes emitidos por el backend emplean la misma envolvente JSON
(`type`, `timestamp`, `request_id` opcional y `payload`).

| ID Excel | Mensaje | Patrón | Contenido mínimo |
| --- | --- | --- | --- |
| — | `ack` | Respuesta inmediata a comandos no periódicos. | `name`, `accepted: boolean`; `operation_id` cuando corresponda; `result` opcional para devolver datos de una consulta; `error` opcional cuando `accepted` es `false`. Confirma recepción y resultado de la solicitud, no finalización de una operación. Es parte de la envolvente WebSocket y no tiene fila propia en el Excel. |
| — | `error` | Error de envolvente o de un comando periódico que no puede procesarse. | `code`, `message`, `name` cuando se conozca. Es parte de la envolvente WebSocket y no tiene fila propia en el Excel. |
| — | `teleoperation_status` | Al iniciar, finalizar o cambiar el estado efectivo de teleoperación. | `enabled`, `active`, `deadman`, `freedrive`, `mode`, `speed_percentage`, estado de seguridad y `reason`. El Excel no define un mensaje B2H específico equivalente. |
| B2H-001, B2H-002 | `telemetry` | Periódico. | Estado articular, pose/frame disponible y estado efectivo de teleoperación: `enabled`, `active`, `deadman`, `freedrive`, `mode`, `speed_percentage` y estado de seguridad. Agrupa el contenido de `/joint_states` y `/tf + /tf_static`. |
| B2H-006 | `constraints` | Periódico o al cambiar. | Direcciones o ejes bloqueados/limitados, velocidad máxima y motivo. |
| B2H-011 | `robot_status` | Periódico o al cambiar. | Estado de base, brazo, herramienta, batería, sensores y alarmas. El Excel identifica como referencia `/robot/arm/io_and_status_controller/robot_mode`. |
| B2H-009 | `operation_status` | Durante una operación y ante cambio de estado. | `operation_id`, `status`, `progress` normalizado y `result` o `error`. |
| B2H-007 | `planned_trajectory` | Tras planificar un movimiento que requiera confirmación. | `operation_id`, `trajectory_id`, trayectoria, resultado de validación y pose final. |
| — | `robot_model_chunk` | Después de `robot.model.download`. | `model_id`, número de fragmento, total de fragmentos y datos Base64 del ZIP. |
| B2H-004 | `tool_camera` | Stream independiente o periódico según el transporte acordado. | Imagen/vídeo, codificación, timestamp y metadatos de cámara disponibles. |

### Modelo del robot

`robot.model.get` devuelve únicamente el manifiesto del modelo. La HMI puede
comparar `model_id` y `sha256` con su caché antes de solicitar la descarga:

```json
{"type":"command","request_id":"model-001","payload":{"name":"robot.model.get"}}
```

El resultado contiene `model_id`, `format: "urdf-zip"`, `root_file`, `size`,
`sha256`, `chunk_size` y la lista `assets`, con el tamaño y SHA-256 de cada
archivo incluido.

Para descargarlo:

```json
{"type":"command","request_id":"model-002","payload":{"name":"robot.model.download"}}
```

El backend responde primero con un `ack` cuyo `result` contiene el manifiesto
y después envía una secuencia de mensajes `robot_model_chunk` por la misma
conexión. Cada fragmento tiene `sequence` empezando en cero, `total` y `data`
codificado en Base64. La HMI debe concatenar los fragmentos en orden, validar
el SHA-256 global y descomprimir el ZIP. El ZIP contiene `robot.urdf` y
`meshes/` con rutas relativas.

El tamaño máximo del ZIP y el tamaño de fragmento se configuran en
`robot_model_max_size_bytes` y `robot_model_chunk_size` respectivamente.

## Ejemplos de mensajes

Los siguientes ejemplos muestran la envolvente completa y valores
representativos. Los `timestamp` y `request_id` son ilustrativos.

### HMI → backend

La HMI envía mensajes con `type: "command"`; el comando concreto se indica en
`payload.name`. Los siguientes generan una respuesta inmediata del backend.

#### `teleoperation.enable`

Petición:

```json
{"type":"command","request_id":"req-001","timestamp":1730000000.0,"payload":{"name":"teleoperation.enable"}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-001","timestamp":1730000000.01,"payload":{"name":"teleoperation.enable","accepted":true}}
```

#### `teleoperation.disable`

Petición:

```json
{"type":"command","request_id":"req-002","timestamp":1730000000.1,"payload":{"name":"teleoperation.disable"}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-002","timestamp":1730000000.11,"payload":{"name":"teleoperation.disable","accepted":true}}
```

#### `teleoperation.set_mode`

Petición:

```json
{"type":"command","request_id":"req-003","timestamp":1730000000.2,"payload":{"name":"teleoperation.set_mode","mode":"joint"}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-003","timestamp":1730000000.21,"payload":{"name":"teleoperation.set_mode","accepted":true}}
```

#### `teleoperation.set_speed`

Petición:

```json
{"type":"command","request_id":"req-004","timestamp":1730000000.3,"payload":{"name":"teleoperation.set_speed","percentage":25.0}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-004","timestamp":1730000000.31,"payload":{"name":"teleoperation.set_speed","accepted":true}}
```

#### `arm.plan_to_pose`

Petición:

```json
{"type":"command","request_id":"req-005","timestamp":1730000000.4,"payload":{"name":"arm.plan_to_pose","operation_id":"op-001","pose":{"position":[0.4,0.0,0.5],"orientation":[0.0,1.0,0.0,0.0]},"frame_id":"base_link","speed_percentage":25.0}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-005","timestamp":1730000000.41,"payload":{"name":"arm.plan_to_pose","accepted":true,"operation_id":"op-001"}}
```

Al terminar la planificación, el backend envía `planned_trajectory` con la
trayectoria que la HMI debe mostrar antes de ejecutar.

#### `arm.plan_to_joint_configuration`

Petición:

```json
{"type":"command","request_id":"req-012","timestamp":1730000000.45,"payload":{"name":"arm.plan_to_joint_configuration","operation_id":"op-002","positions":[0.2,-1.2,0.4,-1.4,0.0,0.2]}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-012","timestamp":1730000000.46,"payload":{"name":"arm.plan_to_joint_configuration","accepted":true,"operation_id":"op-002"}}
```

Con `robot.type: mock`, el robot interpola linealmente entre la configuración actual y `positions`; la
HMI recibe los puntos resultantes mediante `planned_trajectory`.

#### `arm.execute_trajectory`

Petición:

```json
{"type":"command","request_id":"req-006","timestamp":1730000000.5,"payload":{"name":"arm.execute_trajectory","operation_id":"op-001","trajectory_id":"traj-op-001"}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-006","timestamp":1730000000.51,"payload":{"name":"arm.execute_trajectory","accepted":true,"operation_id":"op-001"}}
```

El progreso posterior se publica mediante `operation_status`.

#### `operation.cancel`

Petición:

```json
{"type":"command","request_id":"req-011","timestamp":1730000000.55,"payload":{"name":"operation.cancel","operation_id":"op-001"}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-011","timestamp":1730000000.56,"payload":{"name":"operation.cancel","accepted":true,"operation_id":"op-001"}}
```

#### `poses.list`

Petición:

```json
{"type":"command","request_id":"req-007","timestamp":1730000000.6,"payload":{"name":"poses.list"}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-007","timestamp":1730000000.61,"payload":{"name":"poses.list","accepted":true,"result":{"poses":[{"pose_id":"home","name":"home","pose":{"position":[0.0,-0.4,0.5],"orientation":[0.0,1.0,0.0,0.0]},"frame_id":"base_link","is_home":true}]}}}
```

#### `poses.save`

Petición:

```json
{"type":"command","request_id":"req-008","timestamp":1730000000.7,"payload":{"name":"poses.save","pose_id":"inspection","pose_name":"inspection","pose":{"position":[0.4,0.0,0.5],"orientation":[0.0,1.0,0.0,0.0]}}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-008","timestamp":1730000000.71,"payload":{"name":"poses.save","accepted":true}}
```

#### `poses.execute`

Petición:

```json
{"type":"command","request_id":"req-009","timestamp":1730000000.8,"payload":{"name":"poses.execute","operation_id":"op-002","pose_id":"inspection"}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-009","timestamp":1730000000.81,"payload":{"name":"poses.execute","accepted":true,"operation_id":"op-002"}}
```

El progreso posterior se publica mediante `operation_status`.

#### `home.set`

Petición:

```json
{"type":"command","request_id":"req-010","timestamp":1730000000.9,"payload":{"name":"home.set","pose":{"position":[0.0,-0.4,0.5],"orientation":[0.0,1.0,0.0,0.0]},"frame_id":"base_link"}}
```

Respuesta:

```json
{"type":"ack","request_id":"req-010","timestamp":1730000000.91,"payload":{"name":"home.set","accepted":true}}
```

### Mensajes periódicos HMI → backend

La HMI debe enviar estos mensajes repetidamente mientras necesite mantener el
estado o el movimiento activo.

#### `teleoperation.deadman`

Petición:

```json
{"type":"command","request_id":"hb-001","timestamp":1730000001.0,"payload":{"name":"teleoperation.deadman","active":true}}
```

Respuesta inmediata: ninguna. El deadman debe repetirse periódicamente; si
expira, el backend publica `teleoperation_status` con
`reason: "deadman_timeout"`.

#### `arm.joint_jog`

Petición:

```json
{"type":"command","request_id":"jog-001","timestamp":1730000001.0,"payload":{"name":"arm.joint_jog","velocities":[0.3,0.0,0.0,0.0,0.0,0.0]}}
```

Respuesta inmediata: ninguna. El jog debe repetirse periódicamente; si
expira, el backend pone las velocidades a cero y lo refleja en `telemetry`.

#### `arm.cartesian_jog`

Petición:

```json
{"type":"command","request_id":"jog-002","timestamp":1730000001.0,"payload":{"name":"arm.cartesian_jog","frame_id":"base_link","twist":{"linear":[0.05,0.0,0.0],"angular":[0.0,0.0,0.0]}}}
```

Respuesta inmediata: ninguna. El jog debe repetirse periódicamente; si
expira, el backend pone las velocidades a cero y lo refleja en `telemetry`.

#### `teleoperation.freedrive`

Petición para activar:

```json
{"type":"command","request_id":"fd-001","timestamp":1730000001.0,"payload":{"name":"teleoperation.freedrive","active":true}}
```

Respuesta inmediata: ninguna. El comando debe repetirse periódicamente; si
expira, el backend desactiva freedrive y publica `teleoperation_status` con
`reason: "freedrive_timeout"`.

Petición para desactivar:

```json
{"type":"command","request_id":"fd-002","timestamp":1730000001.1,"payload":{"name":"teleoperation.freedrive","active":false}}
```

Respuesta inmediata: ninguna; freedrive queda desactivado.

### Mensajes que recibe la HMI ← backend

Confirmación de un comando aceptado:

```json
{"type":"ack","request_id":"req-001","timestamp":1730000000.01,"payload":{"name":"teleoperation.enable","accepted":true}}
```

Rechazo de un comando con una operación no disponible:

```json
{"type":"ack","request_id":"req-006","timestamp":1730000000.51,"payload":{"name":"arm.execute_trajectory","accepted":false,"error":{"code":"trajectory_not_found","message":"la trayectoria no existe o ya no está disponible"}}}
```

Estado efectivo de teleoperación, emitido al cambiar o al expirar un watchdog:

```json
{"type":"teleoperation_status","timestamp":1730000001.05,"payload":{"enabled":true,"active":true,"deadman":true,"freedrive":true,"mode":"joint","speed_percentage":25.0,"safety":"ok","reason":"freedrive_active"}}
```

Telemetría articular y de sesión:

```json
{"type":"telemetry","timestamp":1730000001.05,"payload":{"joint_names":["shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"],"positions":[0.0,-1.57,0.0,-1.57,0.0,0.0],"velocities":[0.0,0.0,0.0,0.0,0.0,0.0],"frame_id":"base_link","tool_frame":"tool0","teleoperation":{"enabled":true,"active":true,"deadman":true,"freedrive":true,"mode":"joint","speed_percentage":25.0,"safety":"ok","reason":"freedrive_active"}}}
```

Restricciones de movimiento:

```json
{"type":"constraints","timestamp":1730000001.05,"payload":{"directions":{"x+":true,"x-":true,"y+":true,"y-":true,"z+":true,"z-":true},"max_velocity_percentage":25.0,"reason":"none"}}
```

Estado general del robot:

```json
{"type":"robot_status","timestamp":1730000001.05,"payload":{"base":"available","arm":"available","tool":"available","battery_percentage":87.0,"alarms":[]}}
```

Estado de una operación:

```json
{"type":"operation_status","timestamp":1730000001.1,"payload":{"operation_id":"op-001","status":"executing","progress":0.35}}
```

Trayectoria planificada pendiente de confirmación:

```json
{"type":"planned_trajectory","timestamp":1730000001.1,"payload":{"operation_id":"op-001","trajectory_id":"traj-op-001","trajectory":{"joint_names":["shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"],"points":[{"positions":[0.0,-1.57,0.0,-1.57,0.0,0.0]},{"positions":[0.0,-1.57,0.0,-1.57,0.0,0.0]}]},"validation":{"valid":true},"final_pose":{"frame_id":"base_link","position":[0.4,0.0,0.5],"orientation":[0.0,1.0,0.0,0.0]}}}
```

Imagen de la cámara de herramienta codificada en base64:

```json
{"type":"tool_camera","timestamp":1730000001.1,"payload":{"encoding":"base64","format":"png","data":"iVBORw0KGgoAAAANSUhEUgAAAAEAAAAB..."}}
```

## Correspondencias pendientes con el Excel

La hoja `Arquitectura HMI - Backend` del Excel contiene mensajes que todavía no
están descritos como mensajes WebSocket en este contrato:

| ID Excel | Interfaz / mensaje de referencia | Situación en este contrato |
| --- | --- | --- |
| H2B-012 | `/robot/arm/sequence_control` | Falta un comando para iniciar, pausar, reanudar y cancelar secuencias. |
| B2H-003 | Datos de sensores para visualización 3D | Falta un mensaje/stream para escena, profundidad y sensores adicionales. |
| B2H-005 | Metadatos / overlays de vídeo | Falta un mensaje/stream de detecciones, referencias, distancias y zonas para overlays. |

`teleoperation.enable` corresponde a H2B-013 y `operation.cancel` a H2B-014
en el Excel.

Los comandos periódicos (`teleoperation.deadman`, `arm.joint_jog`,
`arm.cartesian_jog` y `teleoperation.freedrive`) no generan `ack` cuando son
aceptados. La HMI debe usar
`telemetry`, `constraints` y `teleoperation_status` para conocer el estado
efectivo. El backend solo responde con `error` si no puede aceptar uno de
esos comandos.

## Frecuencias y watchdog

Valores por defecto:

| Elemento | Frecuencia o tiempo |
| --- | --- |
| Telemetría y streams backend → HMI | `telemetry_hz: 20 Hz` (cada 50 ms) |
| Telemetría y estado de operación | `20 Hz` mientras exista una operación activa |
| Timeout de deadman | `0.5 s` |
| Timeout de jog | `0.5 s` |
| Timeout de freedrive | `0.5 s` |
| Cadencia recomendada de deadman y jog | `10 Hz` (cada 100 ms) |
| Resumen de logs de entrada | `log_stats_period_sec: 5 s` |

En cada ciclo de `telemetry_hz`, el WebSocket emite cuatro mensajes:

- `telemetry`: articulaciones, velocidades y estado de teleoperación.
- `constraints`: restricciones activas y porcentaje máximo de velocidad.
- `robot_status`: estado general del robot.
- `tool_camera`: imagen o stream de la cámara de herramienta, con la codificación negociada para el despliegue.

### Regla de deadman

Para un jog manual, la HMI debe enviar de forma continua `teleoperation.deadman`
con `active: true` **y** el comando de jog. Para usar freedrive, debe enviar
también `teleoperation.freedrive` con `active: true` de forma continua. Cada
comando tiene un watchdog independiente. El script de ejemplo usa 10 Hz, menor
que el timeout configurado de 0.5 s.

Si no llega un `teleoperation.deadman` con `active: true` durante
el backend:

1. desactiva el deadman;
2. establece las velocidades articulares a cero;
3. publica `telemetry.teleoperation.active: false` en el siguiente ciclo.

La desconexión WebSocket produce el mismo resultado al expirar ese timeout,
porque dejan de recibirse mensajes de deadman. La parada efectiva debe estar
también reforzada por la capa de control y seguridad del robot.

Si el deadman se mantiene pero deja de recibirse `arm.joint_jog` o
`arm.cartesian_jog`, las velocidades se ponen a cero cuando expira el mismo
timeout. La sesión continúa habilitada y el campo `deadman` permanece activo;
la HMI debe enviar un nuevo jog periódico para volver a mover el brazo.

### Regla de freedrive

El freedrive solo permanece activo mientras el backend reciba periódicamente
`teleoperation.freedrive` con `active: true`. Si el comando deja de llegar
durante el timeout configurado, el backend desactiva el freedrive y publica el
cambio en `teleoperation_status`. Enviar `active: false` lo desactiva de forma
inmediata.

### Ausencia de publicaciones backend → HMI

El backend publica mientras siga activo. Si el cliente no recibe mensajes o se
cierra la conexión, el backend no reintenta ni conserva un histórico. La HMI
debería marcar la telemetría como caducada si no recibe ningún mensaje durante
al menos tres períodos de publicación (150 ms con la frecuencia por defecto)
y deshabilitar sus controles de movimiento.
