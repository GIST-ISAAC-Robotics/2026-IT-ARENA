import ast
from pathlib import Path

import pytest

import prepare_jetson_bundle as bundle


def test_minimal_bundle_has_all_static_internal_dependencies_and_no_ros_node():
    closure = bundle.follow_profile_closure()
    tools = bundle.follow_profile_tools()
    assert tools == tuple(closure['files'])+bundle.FOLLOW_PROFILE_DATA and len(set(tools)) == len(tools)
    paths = {name for name in tools if name.startswith('src/')}
    for name in paths:
        tree = ast.parse((bundle.ROOT/name).read_text(encoding='utf-8'), feature_version=(3, 10))
        imports = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.module]
        for imported in imports:
            if imported.startswith(('arena_autonomy.', 'arena_vehicle_interface.')):
                package, module = imported.split('.')
                assert f'src/{package}/{package}/{module}.py' in paths
        assert not any(imported.startswith(('rclpy', 'rosbag2_py')) for imported in imports)
    # 고정 목록이 빠뜨렸던 최근 보조 모듈(r38-r41)과 계측 도구 자신의 의존성.
    for module in ('piecewise_wall', 'moving_pass', 'masked_motion_speed', 'short_entry_transition',
                   'adaptive_corridor_offset', 'corner_boundary_support'):
        assert f'src/arena_autonomy/arena_autonomy/{module}.py' in paths
    assert 'scripts/python_dependency_closure.py' in tools
    assert set(bundle.third_party(closure)) <= set(bundle.FOLLOW_PROFILE_THIRD_PARTY)
    assert all((bundle.ROOT/name).is_file() for name in tools)


def test_follow_profile_rejects_unapproved_third_party(monkeypatch):
    monkeypatch.setattr(bundle, 'FOLLOW_PROFILE_THIRD_PARTY', ('pytest',))
    with pytest.raises(ValueError, match='numpy'):
        bundle.follow_profile_closure()


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
    files = bundle.follow_profile_files([raw], tools=('scripts/tool.py',))
    assert files == [tmp_path/'scripts/tool.py', raw]


def test_bundle_hash_supports_python310(tmp_path, monkeypatch):
    import hashlib
    monkeypatch.delattr(hashlib, 'file_digest', raising=False)
    path = tmp_path/'input'
    path.write_bytes(b'private trace')
    assert bundle.digest(path) == hashlib.sha256(b'private trace').hexdigest()


def tiny_archive(tmp_path, name='artifacts/trace.jsonl', duplicate=False, header_data=None, extra=None):
    import io
    import json
    import tarfile
    content = json.dumps({'kind': 'header', 'data': header_data or {'scope': 'g1_decision_stage_only'}}).encode()+b'\n'
    members = [(name, content)]*(2 if duplicate else 1)+list((extra or {}).items())
    path = tmp_path/'bundle.tar.gz'
    with tarfile.open(path, 'w:gz') as tar:
        for member, data in members:
            info = tarfile.TarInfo(member)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    import hashlib
    manifest = dict(bundle_mode='follow-profile-only', includes_follow_traces=True, includes_recording=False,
                    archive_sha256=bundle.digest(path), archive_size=path.stat().st_size,
                    files={member: dict(size=len(data), sha256=hashlib.sha256(data).hexdigest())
                           for member, data in members})
    return path, manifest


CLOSURE_SPEC = dict(entries=['scripts/entry.py'], roots=['src/arena_autonomy', 'scripts'], files=3,
                    external={'numpy': 'third_party'}, allowed_third_party=['numpy'])
PKG = 'src/arena_autonomy/arena_autonomy'


@pytest.mark.parametrize('members,spec,problem', [
    ({f'{PKG}/__init__.py': b'', f'{PKG}/core.py': b'import numpy\n'}, {}, None),
    ({f'{PKG}/__init__.py': b''}, {}, 'missing_internal_dependency'),
    ({f'{PKG}/__init__.py': b'', f'{PKG}/core.py': b'import numpy\n'}, {'allowed_third_party': []},
     'unexpected_third_party'),
    ({f'{PKG}/__init__.py': b'', f'{PKG}/core.py': b'import numpy\nimport json\n'}, {},
     'closure_differs_from_manifest'),
])
def test_bundle_verifier_recloses_imports_from_archive_members(tmp_path, members, spec, problem):
    from verify_follow_profile_bundle import verify
    extra = {'scripts/entry.py': b'from arena_autonomy.core import *\n', **members}
    path, manifest = tiny_archive(tmp_path, extra=extra)
    manifest['dependency_closure'] = {**CLOSURE_SPEC, **spec}
    result = verify(path, manifest)
    assert result['dependency_closure']['checked'] and not result['extracted']
    assert result['passed'] == (problem is None)
    if problem:
        assert problem in result['problems'][-1]['reason']


def test_previous_bundle_without_closure_record_is_marked_unchecked(tmp_path):
    from verify_follow_profile_bundle import verify
    path, manifest = tiny_archive(tmp_path)
    assert verify(path, manifest)['dependency_closure'] == {'checked': False}


def test_bundle_verification_does_not_extract_or_authorize_remote(tmp_path):
    from verify_follow_profile_bundle import verify
    path, manifest = tiny_archive(tmp_path)
    result = verify(path, manifest)
    assert result['passed'] and not result['extracted'] and not result['remote_execution_verified']
    assert not (tmp_path/'artifacts').exists()
    manifest['files']['artifacts/trace.jsonl']['sha256'] = '0'*64
    assert not verify(path, manifest)['passed']


@pytest.mark.parametrize('schema,mode,expected', [(5,'g1_g2_handoff_trial',True),
    (4,'g1_g2_handoff_trial',False),(5,'g1_follow',False)])
def test_bundle_handoff_scope_requires_correct_schema_and_controller(tmp_path,schema,mode,expected):
    from verify_follow_profile_bundle import verify
    path,manifest=tiny_archive(tmp_path,header_data=dict(schema=schema,
        scope='g1_g2_handoff_decision_stage_only',controller_mode=mode))
    result=verify(path,manifest)
    assert result['passed']==expected and not result['remote_execution_verified']


@pytest.mark.parametrize('name,duplicate', [('../outside.jsonl', False), ('/root/absolute', False),
                                          ('C:/outside.jsonl', False), ('artifacts/trace.jsonl', True)])
def test_bundle_verifier_rejects_path_escape_and_duplicates(tmp_path, name, duplicate):
    from verify_follow_profile_bundle import verify
    path, manifest = tiny_archive(tmp_path, name, duplicate)
    with pytest.raises(ValueError, match='unsafe'):
        verify(path, manifest)


def test_profile_cli_required_snapshot_files_are_all_in_bundle():
    from profile_follow_trace import SNAPSHOT_CLOSURE, SNAPSHOT_OPTIONAL
    tools = set(bundle.follow_profile_tools())
    assert set(SNAPSHOT_CLOSURE['files']) <= tools
    # 선택 기록 시험은 ROS 보조 시험을 끌어오므로 묶음 밖이며 CLI는 누락으로 보고한다.
    assert not set(SNAPSHOT_OPTIONAL) & tools
