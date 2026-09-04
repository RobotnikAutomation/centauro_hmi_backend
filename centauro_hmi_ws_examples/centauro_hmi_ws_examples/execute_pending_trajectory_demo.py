import argparse
import asyncio
import json

from .common.client import connect, receive_until, request_ack, require_accepted


async def run(args):
    async with connect(args.url, args.timeout) as socket:
        ack = await request_ack(
            socket,
            'arm.execute_pending_trajectory',
            args.timeout,
        )
        print('ACK de ejecución:')
        print(json.dumps(ack, indent=2))
        require_accepted(ack)
        operation_id = ack['payload']['result']['operation_id']

        status = await receive_until(
            socket,
            lambda message: (
                message.get('type') == 'operation_status'
                and message.get('payload', {}).get('operation_id') == operation_id
                and message.get('payload', {}).get('status') in ('succeeded', 'cancelled', 'failed')
            ),
            args.timeout + 5.0,
        )
        print('Estado final:')
        print(json.dumps(status, indent=2))


def main():
    parser = argparse.ArgumentParser(description='Ejecuta la trayectoria pendiente de confirmación.')
    parser.add_argument('--url', default='ws://127.0.0.1:8765')
    parser.add_argument('--timeout', type=float, default=5.0)
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
