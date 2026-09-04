import asyncio
import errno

import pytest
import websockets

from centauro_hmi_backend.transport import WebSocketTransport


def test_transport_accepts_client_and_forwards_messages():
    received = []

    async def on_message(websocket, raw):
        received.append(raw)

    async def run():
        transport = WebSocketTransport('127.0.0.1', 0, on_message)
        try:
            await transport.start()
        except OSError as exc:
            if exc.errno == errno.EPERM or 'could not bind on any address' in str(exc):
                pytest.skip('el entorno no permite abrir sockets locales')
            raise
        port = transport._server.sockets[0].getsockname()[1]
        try:
            async with websockets.connect(f'ws://127.0.0.1:{port}') as client:
                await client.send('{"hello":"world"}')
                await asyncio.wait_for(_wait_for_value(received), 1.0)
                assert len(transport.clients) == 1
        finally:
            await transport.stop()

    asyncio.run(run())


async def _wait_for_value(values):
    while not values:
        await asyncio.sleep(0.01)
