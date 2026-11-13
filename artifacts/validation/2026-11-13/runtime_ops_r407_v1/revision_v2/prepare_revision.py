"""원 archive의 208개 입력에서 하네스만 파생한다. ROS/공통 코드 실행 없음."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
ORIGINAL = HERE.parent


def sha(data):
    return hashlib.sha256(data).hexdigest()


def write_new(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream:
        stream.write(data)


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf8')


def main():
    old_manifest = json.loads((ORIGINAL / 'manifest.json').read_bytes())
    old_manifest_sha = sha((ORIGINAL / 'manifest.json').read_bytes())
    assert old_manifest_sha == '5d0718dc4246939383399211f29351a37ab2f572e531cbb22659b3eab59c8d2d'
    assert all(sha((ORIGINAL / n).read_bytes()) == h for n, h in old_manifest['files'].items())
    old_zip = ORIGINAL / 'bundle_sources.zip'
    assert sha(old_zip.read_bytes()) == '152dd6a50703ca09c6bfdc7363383b1dda017db3c79ebb570c15e184a1384db1'
    with zipfile.ZipFile(old_zip) as source:
        old_bundle_manifest = json.loads(source.read('bundle_manifest.json'))
        members = {name: source.read(name) for name in old_bundle_manifest['files']}
    assert len(members) == 208
    assert all(sha(data) == old_bundle_manifest['files'][name] for name, data in members.items())
    old_run = members['r407/run_once.py']
    members['r407/run_once.py'] = (HERE / 'staged/r407/run_once.py').read_bytes()
    members['r407/operator_receipt.py'] = (HERE / 'staged/r407/operator_receipt.py').read_bytes()
    for name in ('test_operator_receipt.py', 'failure_fixture.json'):
        members['r407/tests/' + name] = (HERE / 'tests' / name).read_bytes()
    changed = [n for n, h in old_bundle_manifest['files'].items() if sha(members[n]) != h]
    added = sorted(set(members) - set(old_bundle_manifest['files']))
    assert changed == ['r407/run_once.py']
    assert added == ['r407/operator_receipt.py', 'r407/tests/failure_fixture.json', 'r407/tests/test_operator_receipt.py']
    assert all(sha(members[n]) == h for n, h in old_bundle_manifest['files'].items() if n.startswith(('src/', 'scripts/')))
    for name, data in members.items():
        if name.endswith('.py'):
            ast.parse(data, filename=name)
    for name in ('execute_isolated.py', 'prepare_revision.py', 'audit_original.py'):
        ast.parse((HERE / name).read_bytes(), filename=name)
    write_new(HERE / 'reference_original/run_once.py', old_run)
    for name in ('manifest.json', 'bundle_manifest.json', 'execute_isolated.py'):
        write_new(HERE / 'reference_original' / name, (ORIGINAL / name).read_bytes())
    patch = ''.join(difflib.unified_diff(old_run.decode().splitlines(keepends=True),
        members['r407/run_once.py'].decode().splitlines(keepends=True),
        fromfile='original/r407/run_once.py', tofile='revision_v2/r407/run_once.py'))
    patch += ''.join(difflib.unified_diff([], members['r407/operator_receipt.py'].decode().splitlines(keepends=True),
        fromfile='/dev/null', tofile='revision_v2/r407/operator_receipt.py'))
    write_new(HERE / 'harness.patch', patch.encode('utf8'))
    bundle_manifest = dict(schema='r407_ops_bundle_v2', files={n: sha(data) for n, data in sorted(members.items())},
        original_manifest_sha256=old_manifest_sha, original_archive_sha256=sha(old_zip.read_bytes()),
        original_bundle_manifest_sha256=sha((ORIGINAL / 'bundle_manifest.json').read_bytes()),
        replaced_original_members=changed, new_members=added, original_unchanged_members=207,
        common_source_edits=0, original_pipeline_passed=False, new_ros_execution_performed=False)
    write_new(HERE / 'bundle_manifest.json', json_bytes(bundle_manifest))
    with zipfile.ZipFile(HERE / 'bundle_sources.zip', 'x', zipfile.ZIP_DEFLATED) as target:
        for name, data in sorted(members.items()):
            target.writestr(name, data)
        target.writestr('bundle_manifest.json', json_bytes(bundle_manifest))
    evidence = json.loads((HERE / 'original_audit.json').read_bytes())['evidence_sha256']
    assert all(sha((ORIGINAL / n).read_bytes()) == h for n, h in evidence.items())
    result = dict(schema='r407_v2_static_preparation_v1', passed=True,
        source_files=len(members), changed_original_members=changed, added_members=added,
        unchanged_original_members=207, common_source_changes=[], original_result_evidence_unchanged=True,
        profile_bindings_clock_fake_mcu_byte_identical=True, ast_parse_passed=True,
        bundle_sha256=sha((HERE / 'bundle_sources.zip').read_bytes()),
        original_pipeline_passed=False, ros_executed=False, physical_devices_opened=False)
    write_new(HERE / 'static_preparation.json', json_bytes(result))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
