import argparse
import asyncio
import json
import uuid

from .common.client import connect, receive_until, request_ack, require_accepted


def parse_positions(value):
    positions = [float(item) for item in value.split(',')]
    if len(positions) != 6:
        raise argparse.ArgumentTypeError('se esperan exactamente seis posiciones articulares')
    return positions


async def run(args):
    operation_id = args.operation_id or f'plan-{uuid.uuid4()}'
    async with connect(args.url, args.timeout) as socket:
        ack = await request_ack(
            socket,
            'arm.plan_to_joint_configuration',
            args.timeout,
            operation_id=operation_id,
            positions=args.positions,
        )
        print('ACK de planificación:')
        print(json.dumps(ack, indent=2))
        require_accepted(ack)

        planned = await receive_until(
            socket,
            lambda message: (
                message.get('type') == 'planned_trajectory'
                and message.get('payload', {}).get('operation_id') == operation_id
            ),
            args.timeout,
        )
        trajectory = planned['payload']
        print('Trayectoria planificada:')
        print(json.dumps(trajectory, indent=2))
        print(f"Puntos recibidos: {len(trajectory['trajectory']['points'])}")

        if not args.execute:
            print('No se ejecuta la trayectoria. Usa --execute para confirmarla.')
            return

        ack = await request_ack(
            socket,
            'arm.execute_trajectory',
            args.timeout,
            operation_id=operation_id,
            trajectory_id=trajectory['trajectory_id'],
        )
        print('ACK de ejecución:')
        print(json.dumps(ack, indent=2))
        require_accepted(ack)

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
    parser = argparse.ArgumentParser(description='Planifica una trayectoria articular mediante WebSocket.')
    parser.add_argument('--url', default='ws://127.0.0.1:8765')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--operation-id')
    parser.add_argument('--positions', type=parse_positions, default=parse_positions('0.2,-1.2,0.4,-1.4,0.0,0.2'))
    parser.add_argument('--execute', action='store_true', help='Confirma y ejecuta la trayectoria planificada.')
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
