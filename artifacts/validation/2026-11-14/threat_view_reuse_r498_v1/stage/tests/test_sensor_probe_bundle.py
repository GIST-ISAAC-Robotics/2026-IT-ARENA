import ast
import hashlib
import io
import tarfile

import pytest

import prepare_jetson_bundle as bundle
from verify_sensor_probe_bundle import EXPECTED, LEGACY_EXPECTED, INVENTORIES, verify


def test_sensor_tool_list_has_all_internal_dependencies():
    paths = set(bundle.SENSOR_PROBE_TOOLS)
    assert paths == EXPECTED and len(bundle.SENSOR_PROBE_TOOLS) == len(paths)
    script_names = {p.rsplit('/', 1)[-1][:-3]: p for p in paths if p.startswith('scripts/') and p.endswith('.py')}
    for rel in paths:
        path = bundle.ROOT/rel
        assert path.is_file()
        if path.suffix != '.py':
            continue
        tree = ast.parse(path.read_text(encoding='utf-8'), feature_version=(3, 10))
        for item in ast.walk(tree):
            if not isinstance(item, ast.ImportFrom) or not item.module:
                continue
            if item.module.startswith('arena_vehicle_interface.'):
                assert 'src/arena_vehicle_interface/'+item.module.replace('.', '/')+'.py' in paths
            elif item.module in script_names:
                assert script_names[item.module] in paths
            elif (bundle.ROOT/'scripts'/f'{item.module}.py').is_file():
                pytest.fail('internal dependency missing: '+item.module)
    assert not any('autonomy/' in p or p.startswith(('artifacts/', 'config/')) for p in paths)
    assert 'scripts/replay_sensor_bag.py' not in paths


def make_archive(tmp_path, change=None, version=4):
    inventory = INVENTORIES[version]
    data = {p: b'# fixture\n' for p in inventory}
    extra = None
    if change == 'missing':
        data.pop('scripts/record_sensor_bag.py')
    elif change == 'syntax':
        data['scripts/record_sensor_bag.py'] = b'invalid python syntax\n'
    elif change in ('escape', 'link', 'extra', 'duplicate'):
        extra = change
    archive = tmp_path/'bundle.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for path, contents in data.items():
            entry = tarfile.TarInfo(path)
            entry.size = len(contents)
            tar.addfile(entry, io.BytesIO(contents))
        if extra:
            name = '../escape' if extra == 'escape' else 'scripts/record_sensor_bag.py' if extra == 'duplicate' else 'unexpected'
            entry = tarfile.TarInfo(name)
            if extra == 'link':
                entry.type, entry.linkname = tarfile.SYMTYPE, '/etc/passwd'
            tar.addfile(entry, io.BytesIO(b''))
    manifest = dict(bundle_mode='sensor-probe-tools-only', includes_recording=False, includes_follow_traces=False,
                    archive_size=archive.stat().st_size, archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                    files={p: dict(size=len(data.get(p, b'')), sha256=hashlib.sha256(data.get(p, b'')).hexdigest()) for p in inventory})
    if version != 1:
        manifest['sensor_probe_tools_version'] = version
    return archive, manifest


def test_bundle_verifies_without_extracting_or_execution(tmp_path):
    archive, manifest = make_archive(tmp_path)
    result = verify(archive, manifest)
    assert result['passed'] and result['members'] == 23 and result['python310_parsed'] == 22
    assert result['sensor_probe_tools_version'] == 4
    assert not any(result[k] for k in ('extracted', 'transferred', 'jetson_verified', 'ready_to_drive'))
    assert list(tmp_path.iterdir()) == [archive]


def test_legacy_fifteen_file_bundle_still_checks_without_modification(tmp_path):
    archive, manifest = make_archive(tmp_path, version=1)
    result = verify(archive, manifest)
    assert result['passed'] and result['members'] == 15
    assert result['sensor_probe_tools_version'] == 1
    assert 'sensor_probe_tools_version' not in manifest


def test_prior_nineteen_file_bundle_still_checks(tmp_path):
    archive, manifest = make_archive(tmp_path, version=2)
    result = verify(archive, manifest)
    assert result['passed'] and result['members'] == 19
    assert result['sensor_probe_tools_version'] == 2


def test_prior_twenty_one_file_bundle_still_checks(tmp_path):
    archive, manifest = make_archive(tmp_path, version=3)
    result = verify(archive, manifest)
    assert result['passed'] and result['members'] == 21
    assert result['sensor_probe_tools_version'] == 3


@pytest.mark.parametrize('version', [False, True, None, '4', 0, 5, 4.0])
def test_unknown_or_ambiguous_inventory_version_rejected(tmp_path, version):
    archive, manifest = make_archive(tmp_path)
    manifest['sensor_probe_tools_version'] = version
    with pytest.raises(ValueError, match='version'):
        verify(archive, manifest)


@pytest.mark.parametrize('source,target', [(s,t) for s in range(1,5) for t in range(1,5) if s!=t])
def test_inventory_cannot_be_mixed_across_versions(tmp_path, source, target):
    archive, manifest = make_archive(tmp_path, version=source)
    manifest['sensor_probe_tools_version'] = target
    with pytest.raises(ValueError, match='inventory'):
        verify(archive, manifest)


@pytest.mark.parametrize('change', ['escape', 'link', 'extra', 'duplicate'])
def test_unsafe_archive_rejected(tmp_path, change):
    archive, manifest = make_archive(tmp_path, change)
    with pytest.raises(ValueError, match='unsafe'):
        verify(archive, manifest)


@pytest.mark.parametrize('change', ['missing', 'syntax', 'archive_digest', 'file_digest'])
def test_bad_content_fails(tmp_path, change):
    archive, manifest = make_archive(tmp_path, change)
    if change == 'archive_digest':
        manifest['archive_sha256'] = '0'*64
    elif change == 'file_digest':
        manifest['files']['scripts/record_sensor_bag.py']['sha256'] = '0'*64
    assert not verify(archive, manifest)['passed']


@pytest.mark.parametrize('field,value', [('bundle_mode', 'source'), ('includes_recording', True),
                                      ('includes_follow_traces', True), ('includes_recording', 0)])
def test_non_probe_bundle_rejected(tmp_path, field, value):
    archive, manifest = make_archive(tmp_path)
    manifest[field] = value
    with pytest.raises(ValueError):
        verify(archive, manifest)
