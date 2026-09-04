import argparse
import asyncio
import json

from .common.client import connect, request_ack, send_command

PERIODIC_COMMANDS = {
    'teleoperation.deadman',
    'arm.joint_jog',
    'arm.cartesian_jog',
    'teleoperation.freedrive',
}


async def run(args):
    payload = json.loads(args.payload)
    if not isinstance(payload, dict):
        raise SystemExit('--payload debe ser un objeto JSON')
    async with connect(args.url, args.timeout) as socket:
        if args.name in PERIODIC_COMMANDS:
            await send_command(socket, args.name, **payload)
            print('Comando enviado; los comandos periódicos no reciben ACK.')
        else:
            ack = await request_ack(socket, args.name, args.timeout, **payload)
            print(json.dumps(ack, indent=2))


def main():
    parser = argparse.ArgumentParser(description='Envía un comando JSON al backend HMI.')
    parser.add_argument('name', help='Nombre del comando, por ejemplo teleoperation.enable')
    parser.add_argument('--payload', default='{}', help='Payload adicional en JSON')
    parser.add_argument('--url', default='ws://127.0.0.1:8765')
    parser.add_argument('--timeout', type=float, default=5.0)
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
