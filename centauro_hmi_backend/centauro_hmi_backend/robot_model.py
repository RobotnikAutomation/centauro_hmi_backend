"""Robot model manifest and download provider."""

import base64
import hashlib
import io
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile


ROBOT_DESCRIPTION_TOPIC = '/robot/robot_description'


def load_mock_robot_description() -> str:
    from ament_index_python.packages import get_package_share_directory
    import xacro

    path = Path(get_package_share_directory('robotnik_description')) / (
        'robots/rbkairos/rbkairos_plus.urdf.xacro'
    )
    return xacro.process_file(
        str(path), mappings={'namespace': 'robot', 'prefix': 'robot_'}
    ).toxml()


def _description_assets(description: str) -> dict[str, bytes]:
    from ament_index_python.packages import get_package_share_directory

    root = ET.fromstring(description)
    files = {}
    for element in root.iter():
        uri = element.get('filename')
        if uri is None:
            continue
        if not uri.startswith('package://'):
            raise ValueError(f'Unsupported robot asset URI: {uri}')
        package, relative = uri[len('package://'):].split('/', 1)
        # Keep installed symlinks: colcon assets may point into the source tree.
        share = Path(os.path.abspath(get_package_share_directory(package)))
        path = Path(os.path.abspath(share / relative))
        if not path.is_relative_to(share):
            raise ValueError(f'Robot asset outside package: {uri}')
        name = f'meshes/{package}/{relative}'
        files[name] = path.read_bytes()
        element.set('filename', name)
        if path.suffix.lower() == '.dae':
            collada = ET.fromstring(files[name])
            for image in collada.findall('.//{*}image/{*}init_from'):
                texture = (image.text or '').strip()
                texture_path = Path(os.path.abspath(path.parent / texture))
                if not texture or not texture_path.is_relative_to(share):
                    raise ValueError(f'Invalid COLLADA texture in {uri}: {texture}')
                texture_name = f'meshes/{package}/{texture_path.relative_to(share).as_posix()}'
                files[texture_name] = texture_path.read_bytes()
    return {'robot.urdf': ET.tostring(root, encoding='utf-8'), **files}


class RobotModelProvider:
    """Builds and caches a distributable URDF model bundle."""

    def __init__(self, model_root, max_size_bytes=50 * 1024 * 1024, chunk_size=64 * 1024,
                 robot_description=None):
        self.model_root = Path(model_root)
        self.max_size_bytes = int(max_size_bytes)
        self.chunk_size = int(chunk_size)
        self.robot_description = robot_description
        self._cache = {}
        self._manifest = None

    def manifest(self):
        self._ensure_bundle()
        return json.loads(json.dumps(self._manifest))

    def bundle(self):
        self._ensure_bundle()
        return self._cache[self._manifest['model_id']]

    def chunks(self):
        data = self.bundle()
        total = (len(data) + self.chunk_size - 1) // self.chunk_size
        for sequence in range(total):
            start = sequence * self.chunk_size
            yield {
                'model_id': self._manifest['model_id'],
                'sequence': sequence,
                'total': total,
                'data': base64.b64encode(data[start:start + self.chunk_size]).decode('ascii'),
            }

    def _ensure_bundle(self):
        if self._manifest is not None:
            return
        if self.robot_description is not None:
            files = _description_assets(self.robot_description)
        else:
            urdf_path = self.model_root / 'urdf' / 'arm.urdf'
            meshes_root = self.model_root / 'meshes'
            if not urdf_path.is_file():
                raise FileNotFoundError(f'URDF no encontrado: {urdf_path}')

            urdf = urdf_path.read_text(encoding='utf-8')
            urdf = urdf.replace('package://centauro_hmi_backend/meshes/', 'meshes/')
            files = {'robot.urdf': urdf.encode('utf-8')}
            if meshes_root.is_dir():
                for path in sorted(meshes_root.rglob('*')):
                    if path.is_file():
                        files[f'meshes/{path.relative_to(meshes_root).as_posix()}'] = path.read_bytes()

        archive = io.BytesIO()
        with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as bundle:
            for name, data in files.items():
                bundle.writestr(name, data)
        bundle_data = archive.getvalue()
        if len(bundle_data) > self.max_size_bytes:
            raise ValueError(
                f'el modelo ocupa {len(bundle_data)} bytes y supera el máximo de '
                f'{self.max_size_bytes} bytes'
            )

        model_hash = hashlib.sha256(bundle_data).hexdigest()
        model_name = 'rbkairos-plus' if self.robot_description is not None else 'mock-arm'
        model_id = f'{model_name}-{model_hash[:16]}'
        self._cache[model_id] = bundle_data
        self._manifest = {
            'model_id': model_id,
            'format': 'urdf-zip',
            'root_file': 'robot.urdf',
            'size': len(bundle_data),
            'sha256': model_hash,
            'chunk_size': self.chunk_size,
            'assets': [
                {
                    'path': name,
                    'size': len(data),
                    'sha256': hashlib.sha256(data).hexdigest(),
                }
                for name, data in files.items()
            ],
        }
