import asyncio
import importlib


class WebSocketTransport:
    """Optional transport adapter; the backend core does not depend on it."""

    def __init__(self, host, port, on_message, on_connect=None, on_disconnect=None):
        self.host, self.port = host, port
        self.on_message, self.on_connect = on_message, on_connect
        self.on_disconnect = on_disconnect
        self.clients = set()
        self._server = None

    async def start(self):
        try:
            websockets = importlib.import_module("websockets")
        except ImportError as exc:
            raise RuntimeError("Falta la dependencia 'websockets'. Instala: sudo apt install python3-websockets") from exc

        async def websocket_handler(websocket, _path=None):
            self.clients.add(websocket)
            if self.on_connect: await self.on_connect(websocket)
            try:
                async for raw in websocket:
                    await self.on_message(websocket, raw)
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                self.clients.discard(websocket)
                if self.on_disconnect: await self.on_disconnect(websocket)

        self._server = await websockets.serve(websocket_handler, self.host, self.port)

    async def publish(self, raw):
        if self.clients:
            await asyncio.gather(*(client.send(raw) for client in tuple(self.clients)), return_exceptions=True)

    async def send_to_client(self, client, raw):
        """Send a response, tolerating a peer that closed between receive/send."""
        websockets = importlib.import_module("websockets")
        try:
            await client.send(raw)
            return True
        except websockets.exceptions.ConnectionClosed:
            return False

    async def stop(self):
        if self._server:
            self._server.close()
            await self._server.wait_closed()
