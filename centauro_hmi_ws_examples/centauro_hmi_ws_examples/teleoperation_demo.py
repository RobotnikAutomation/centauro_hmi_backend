import asyncio

from .common.client import connect, request_ack, send_command


async def run():
    async with connect('ws://127.0.0.1:8765', 5.0) as socket:
        print('ACK:', await request_ack(socket, 'teleoperation.enable', 5.0))
        await send_command(socket, 'teleoperation.deadman', active=True)
        print('ACK:', await request_ack(socket, 'teleoperation.set_speed', 5.0, percentage=25))
        await send_command(socket, 'arm.joint_jog', velocities=[0.3, 0, 0, 0, 0, 0])
        for _ in range(10):
            print('EVENT:', await socket.recv())
        await send_command(socket, 'teleoperation.deadman', active=False)
        print('ACK:', await request_ack(socket, 'teleoperation.disable', 5.0))


def main():
    asyncio.run(run())


if __name__ == '__main__':
    main()
