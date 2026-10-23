import ast
from pathlib import Path

import pytest

import prepare_jetson_bundle as bundle


def test_minimal_bundle_has_all_static_internal_dependencies_and_no_ros_node():
    paths = {name for name in bundle.FOLLOW_PROFILE_TOOLS if name.startswith('src/')}
    for name in paths:
        tree = ast.parse((bundle.ROOT/name).read_text(encoding='utf-8'), feature_version=(3, 10))
        imports = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.module]
        for imported in imports:
            if imported.startswith(('arena_autonomy.', 'arena_vehicle_interface.')):
                package, module = imported.split('.')
                assert f'src/{package}/{package}/{module}.py' in paths
        assert not any(imported.startswith(('rclpy', 'rosbag2_py')) for imported in imports)
    assert all((bundle.ROOT/name).is_file() for name in bundle.FOLLOW_PROFILE_TOOLS)


def test_minimal_scripts_parse_as_python310():
    for name in ('scripts/profile_follow_trace.py', 'scripts/audit_hardware_capture.py',
                 'scripts/audit_follow_occupancy.py'):
        ast.parse((bundle.ROOT/name).read_text(encoding='utf-8'), feature_version=(3, 10))
    # 문법 검사일 뿐 ARM/Jetson 실제 의존성 검증은 아니다.


def test_explicit_trace_policy_and_no_bulk_artifact_collection(tmp_path, monkeypatch):
    monkeypatch.setattr(bundle, 'ROOT', tmp_path)
    artifact = tmp_path/'artifacts'
    artifact.mkdir()
    raw = artifact/'trace.jsonl'
    raw.write_text('{}')
    with pytest.raises(ValueError, match='explicit'):
        bundle.follow_profile_files([])
    with pytest.raises(ValueError, match='unique'):
        bundle.follow_profile_files([tmp_path/'private.jsonl'])
    with pytest.raises(ValueError, match='unique'):
        bundle.follow_profile_files([artifact])
    with pytest.raises(ValueError, match='unique'):
        bundle.follow_profile_files([raw, raw])
    files = bundle.follow_profile_files([raw])
    assert len(files) == len(bundle.FOLLOW_PROFILE_TOOLS)+1 and files[-1] == raw


def test_bundle_hash_supports_python310(tmp_path, monkeypatch):
    import hashlib
    monkeypatch.delattr(hashlib, 'file_digest', raising=False)
    path = tmp_path/'input'
    path.write_bytes(b'private trace')
    assert bundle.digest(path) == hashlib.sha256(b'private trace').hexdigest()


def tiny_archive(tmp_path, name='artifacts/trace.jsonl', duplicate=False):
    import io
    import json
    import tarfile
    content = json.dumps({'kind': 'header', 'data': {'scope': 'g1_decision_stage_only'}}).encode()+b'\n'
    path = tmp_path/'bundle.tar.gz'
    with tarfile.open(path, 'w:gz') as tar:
        for _ in range(2 if duplicate else 1):
            info = tarfile.TarInfo(name)
            info.size = len(content)
            tar.addfile(info, io.BytesIO(content))
    import hashlib
    manifest = dict(bundle_mode='follow-profile-only', includes_follow_traces=True, includes_recording=False,
                    archive_sha256=bundle.digest(path), archive_size=path.stat().st_size,
                    files={name: dict(size=len(content), sha256=hashlib.sha256(content).hexdigest())})
    return path, manifest


def test_bundle_verification_does_not_extract_or_authorize_remote(tmp_path):
    from verify_follow_profile_bundle import verify
    path, manifest = tiny_archive(tmp_path)
    result = verify(path, manifest)
    assert result['passed'] and not result['extracted'] and not result['remote_execution_verified']
    assert not (tmp_path/'artifacts').exists()
    manifest['files']['artifacts/trace.jsonl']['sha256'] = '0'*64
    assert not verify(path, manifest)['passed']


@pytest.mark.parametrize('name,duplicate', [('../outside.jsonl', False), ('/root/absolute', False),
                                          ('C:/outside.jsonl', False), ('artifacts/trace.jsonl', True)])
def test_bundle_verifier_rejects_path_escape_and_duplicates(tmp_path, name, duplicate):
    from verify_follow_profile_bundle import verify
    path, manifest = tiny_archive(tmp_path, name, duplicate)
    with pytest.raises(ValueError, match='unsafe'):
        verify(path, manifest)
