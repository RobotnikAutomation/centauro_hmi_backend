"""Mantiene un jog articular durante un tiempo determinado."""

import argparse
import asyncio

from .common.client import connect, request_ack, send_command


async def run(args):
    velocities = [float(value) for value in args.velocities.split(',')]
    if len(velocities) != 6:
        raise SystemExit('--velocities debe contener exactamente 6 valores')

    async with connect(args.url, args.open_timeout) as socket:
        await request_ack(socket, 'teleoperation.enable', args.open_timeout)
        await request_ack(socket, 'teleoperation.set_speed', args.open_timeout, percentage=args.speed)
        loop = asyncio.get_running_loop()
        end = loop.time() + args.seconds
        while loop.time() < end:
            await send_command(socket, 'teleoperation.deadman', active=True)
            await send_command(socket, 'arm.joint_jog', velocities=velocities)
            await asyncio.sleep(0.1)
        await send_command(socket, 'arm.joint_jog', velocities=[0.0] * 6)
        await send_command(socket, 'teleoperation.deadman', active=False)
        await request_ack(socket, 'teleoperation.disable', args.open_timeout)
        print(f'Jog finalizado después de {args.seconds:.2f} s')


def main():
    parser = argparse.ArgumentParser(description='Envía jog articular durante X segundos')
    parser.add_argument('seconds', type=float, help='Duración del jog en segundos')
    parser.add_argument('--velocities', default='0.3,0,0,0,0,0')
    parser.add_argument('--speed', type=float, default=25.0)
    parser.add_argument('--url', default='ws://127.0.0.1:8765')
    parser.add_argument('--open-timeout', type=float, default=5.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('seconds debe ser mayor que cero')
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
