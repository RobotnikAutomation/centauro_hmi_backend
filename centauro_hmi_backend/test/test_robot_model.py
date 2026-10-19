import base64
import hashlib
import io
import zipfile
import xml.etree.ElementTree as ET

import pytest

from centauro_hmi_backend.robot_model import RobotModelProvider, load_mock_robot_description


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


def test_description_bundle_resolves_package_assets_and_collada_textures(tmp_path, monkeypatch):
    from ament_index_python import packages

    (tmp_path / 'arm.dae').write_text(
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">'
        '<library_images><image><init_from>texture.png</init_from></image>'
        '</library_images></COLLADA>', encoding='utf-8',
    )
    (tmp_path / 'texture.png').write_bytes(b'texture')
    monkeypatch.setattr(packages, 'get_package_share_directory', lambda _: str(tmp_path))
    description = '<robot><link><visual><geometry><mesh filename="package://example/arm.dae"/>'
    description += '</geometry></visual></link></robot>'
    provider = RobotModelProvider(tmp_path, robot_description=description)

    with zipfile.ZipFile(io.BytesIO(provider.bundle())) as archive:
        root = ET.fromstring(archive.read('robot.urdf'))
        assert root.find('.//mesh').get('filename') == 'meshes/example/arm.dae'
        assert archive.read('meshes/example/texture.png') == b'texture'
    assert provider.manifest()['model_id'].startswith('rbkairos-plus-')


def test_description_bundle_reports_missing_assets(tmp_path, monkeypatch):
    from ament_index_python import packages

    monkeypatch.setattr(packages, 'get_package_share_directory', lambda _: str(tmp_path))
    provider = RobotModelProvider(
        tmp_path, robot_description='<robot><mesh filename="package://example/missing.stl"/></robot>',
    )

    with pytest.raises(FileNotFoundError):
        provider.bundle()


def test_installed_rbkairos_description_and_bundle(tmp_path):
    description = load_mock_robot_description()
    root = ET.fromstring(description)
    assert root.get('name') == 'rbkairos_plus'
    assert root.find("./link[@name='robot_base_link']") is not None
    assert root.find("./joint[@name='robot_arm_shoulder_pan_joint']") is not None
    provider = RobotModelProvider(tmp_path, robot_description=description)

    with zipfile.ZipFile(io.BytesIO(provider.bundle())) as archive:
        bundled = ET.fromstring(archive.read('robot.urdf'))
        for element in bundled.iter():
            filename = element.get('filename')
            if filename is not None:
                assert not filename.startswith('package://')
                assert filename in archive.namelist()
        assert any(name.endswith('intel_d435.png') for name in archive.namelist())
