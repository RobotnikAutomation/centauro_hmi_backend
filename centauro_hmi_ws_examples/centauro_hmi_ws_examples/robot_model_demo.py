import argparse
import asyncio
import base64
import hashlib
import json
from pathlib import Path

from .common.client import connect, request_ack, require_accepted


async def run(args):
    async with connect(args.url, args.timeout) as socket:
        manifest_ack = await request_ack(socket, 'robot.model.get', args.timeout)
        require_accepted(manifest_ack)
        manifest = manifest_ack['payload']['result']
        print(f"Modelo: {manifest['model_id']} ({manifest['size']} bytes)")

        download_ack = await request_ack(socket, 'robot.model.download', args.timeout)
        require_accepted(download_ack)
        if download_ack['payload']['result']['model_id'] != manifest['model_id']:
            raise RuntimeError('el modelo cambió entre get y download')

        total = (manifest['size'] + manifest['chunk_size'] - 1) // manifest['chunk_size']
        chunks = [None] * total
        while any(chunk is None for chunk in chunks):
            message = await asyncio.wait_for(socket.recv(), args.timeout)
            chunk = json.loads(message)
            if chunk.get('type') != 'robot_model_chunk':
                continue
            payload = chunk['payload']
            if payload['model_id'] != manifest['model_id']:
                raise RuntimeError('chunk recibido para otro modelo')
            chunks[payload['sequence']] = base64.b64decode(payload['data'])

        data = b''.join(chunks)
        if len(data) != manifest['size']:
            raise RuntimeError('el tamaño descargado no coincide con el manifiesto')
        digest = hashlib.sha256(data).hexdigest()
        if digest != manifest['sha256']:
            raise RuntimeError('el SHA-256 descargado no coincide con el manifiesto')
        Path(args.output).write_bytes(data)
        print(f'Modelo descargado y validado en {args.output}')


def main():
    parser = argparse.ArgumentParser(description='Descarga y valida el modelo URDF del robot.')
    parser.add_argument('--url', default='ws://127.0.0.1:8765')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--output', default='robot_model.zip', help='Archivo ZIP de salida')
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
