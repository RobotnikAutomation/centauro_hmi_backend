import argparse
import asyncio
import json

from .common.client import connect, receive_until, request_ack, require_accepted


async def run(args):
    async with connect(args.url, args.timeout) as socket:
        telemetry = await receive_until(socket, lambda message: message.get('type') == 'telemetry', args.timeout)
        print('Telemetría recibida:')
        print(json.dumps(telemetry, indent=2))
        ack = await request_ack(socket, 'poses.list', args.timeout)
        require_accepted(ack)
        poses = ack['payload']['result']['poses']
        print(f'Catálogo recibido con {len(poses)} pose(s).')


def main():
    parser = argparse.ArgumentParser(description='Comprueba conectividad y mensajes WebSocket sin mover el robot.')
    parser.add_argument('--url', default='ws://127.0.0.1:8765')
    parser.add_argument('--timeout', type=float, default=5.0)
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
