import base64
import hashlib
import io
import zipfile

import pytest

from centauro_hmi_backend.robot_model import RobotModelProvider


def test_manifest_and_chunks_describe_reconstructable_zip(tmp_path):
    (tmp_path / 'urdf').mkdir()
    (tmp_path / 'meshes').mkdir()
    (tmp_path / 'urdf' / 'arm.urdf').write_text(
        '<mesh filename="package://centauro_hmi_backend/meshes/arm.stl"/>',
        encoding='utf-8',
    )
    (tmp_path / 'meshes' / 'arm.stl').write_bytes(b'solid arm')
    provider = RobotModelProvider(tmp_path, chunk_size=5)

    manifest = provider.manifest()
    chunks = list(provider.chunks())
    reconstructed = b''.join(base64.b64decode(chunk['data']) for chunk in chunks)

    assert manifest['format'] == 'urdf-zip'
    assert manifest['root_file'] == 'robot.urdf'
    assert manifest['size'] == len(reconstructed)
    assert manifest['sha256'] == hashlib.sha256(reconstructed).hexdigest()
    assert [chunk['sequence'] for chunk in chunks] == list(range(len(chunks)))
    with zipfile.ZipFile(io.BytesIO(reconstructed)) as archive:
        assert archive.read('robot.urdf') == b'<mesh filename="meshes/arm.stl"/>'
        assert archive.read('meshes/arm.stl') == b'solid arm'


def test_provider_caches_bundle_and_manifest(tmp_path):
    (tmp_path / 'urdf').mkdir()
    (tmp_path / 'urdf' / 'arm.urdf').write_text('<robot/>', encoding='utf-8')
    provider = RobotModelProvider(tmp_path)

    first = provider.bundle()
    model_id = provider.manifest()['model_id']

    assert provider.bundle() is first
    assert model_id in provider._cache


def test_provider_rejects_bundle_above_configured_limit(tmp_path):
    (tmp_path / 'urdf').mkdir()
    (tmp_path / 'urdf' / 'arm.urdf').write_text('<robot/>', encoding='utf-8')
    provider = RobotModelProvider(tmp_path, max_size_bytes=1)

    with pytest.raises(ValueError, match='supera el máximo'):
        provider.manifest()
