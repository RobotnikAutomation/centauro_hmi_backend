import argparse
import asyncio
import json
import uuid

from .common.client import connect, receive_until, request_ack, require_accepted


async def run(args):
    operation_id = args.operation_id or f'cancel-{uuid.uuid4()}'
    async with connect(args.url, args.timeout) as socket:
        require_accepted(await request_ack(
            socket,
            'arm.plan_to_joint_configuration',
            args.timeout,
            operation_id=operation_id,
            positions=[0.2, -1.2, 0.4, -1.4, 0.0, 0.2],
        ))
        await receive_until(
            socket,
            lambda message: message.get('type') == 'planned_trajectory' and message.get('payload', {}).get('operation_id') == operation_id,
            args.timeout,
        )
        ack = await request_ack(socket, 'operation.cancel', args.timeout, operation_id=operation_id)
        print(json.dumps(ack, indent=2))
        require_accepted(ack)
        status = await receive_until(
            socket,
            lambda message: (
                message.get('type') == 'operation_status'
                and message.get('payload', {}).get('operation_id') == operation_id
                and message.get('payload', {}).get('status') == 'cancelled'
            ),
            args.timeout,
        )
        print(json.dumps(status, indent=2))


def main():
    parser = argparse.ArgumentParser(description='Planifica y cancela una operación mediante WebSocket.')
    parser.add_argument('--url', default='ws://127.0.0.1:8765')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--operation-id')
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
