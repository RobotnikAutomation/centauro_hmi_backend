import asyncio
import json
import uuid

import websockets


async def send_command(socket, name, **payload):
    request_id = str(uuid.uuid4())
    await socket.send(json.dumps({
        'type': 'command',
        'request_id': request_id,
        'payload': {'name': name, **payload},
    }))
    return request_id


async def receive_until(socket, predicate, timeout):
    async def receive():
        while True:
            message = json.loads(await socket.recv())
            if predicate(message):
                return message

    return await asyncio.wait_for(receive(), timeout)


async def request_ack(socket, name, timeout, **payload):
    request_id = await send_command(socket, name, **payload)
    return await receive_until(
        socket,
        lambda message: message.get('type') == 'ack' and message.get('request_id') == request_id,
        timeout,
    )


def require_accepted(ack):
    payload = ack['payload']
    if not payload.get('accepted'):
        raise RuntimeError(payload.get('error', {}).get('message', 'comando rechazado'))
    return payload


def connect(url, timeout):
    return websockets.connect(url, open_timeout=timeout)
