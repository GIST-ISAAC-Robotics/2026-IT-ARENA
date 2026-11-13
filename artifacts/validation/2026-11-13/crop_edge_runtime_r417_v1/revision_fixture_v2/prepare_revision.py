"""v1을 보존하며 wrapper 경로 fixture만 고치는 stdlib 준비 도구. 실행 검증은 하지 않는다."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

R = Path(__file__).resolve().parent
A = R.parent
ROOT = A.parents[3]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, text):
    with path.open('x', encoding='utf8', newline='\n') as stream:
        stream.write(text)


def revise(name, output, edits):
    old = (A/name).read_text(encoding='utf8')
    new = old
    for before, after, count in edits:
        assert new.count(before) == count, (name, before, new.count(before), count)
        new = new.replace(before, after)
    write(R/output, new)
    return list(difflib.unified_diff(old.splitlines(keepends=True), new.splitlines(keepends=True),
        fromfile='v1/'+name, tofile='revision_fixture_v2/'+output))


def main():
    original = json.loads((A/'manifest.json').read_bytes())
    assert digest(A/'manifest.json') == '430bb3e9c4d20a42044a310befd73e468d8316975f6106fc777da557b9ed803d'
    for item in original['files']:
        assert digest(ROOT/item['path']) == item['sha256'], item['path']
    diff = revise('test_crop_edge_runtime.py', 'test_crop_edge_runtime_v2.py', [
        ('A = Path(__file__).resolve().parent\n', 'A = Path(__file__).resolve().parent.parent\n', 1),
        ('follow.partial_policy', 'follow.normal.partial_policy', 3),
        ('one.partial_policy', 'one.normal.partial_policy', 7),
        ('two.partial_policy', 'two.normal.partial_policy', 7),
    ])
    diff += revise('test_crop_edge_ros.py', 'test_crop_edge_ros_v2.py', [
        ('from test_crop_edge_runtime import', 'from test_crop_edge_runtime_v2 import', 1),
        ('node.follow.partial_policy', 'node.follow.normal.partial_policy', 5),
    ])
    diff += revise('run_verify.py', 'run_verify.py', [
        ('A = Path(__file__).resolve().parent\n',
         'R = Path(__file__).resolve().parent\nA = R.parent\n', 1),
        ("(A/'manifest.json')", "(R/'manifest.json')", 1),
        ("str(A/'test_crop_edge_runtime.py')", "str(R/'test_crop_edge_runtime_v2.py')", 1),
        ("str(A/'test_crop_edge_ros.py')", "str(R/'test_crop_edge_ros_v2.py')", 1),
    ])
    write(R/'fixture.patch', ''.join(diff))
    parsed = sorted(R.glob('*.py'))
    for path in parsed:
        ast.parse(path.read_text(encoding='utf8'), filename=str(path))
    stages = json.loads((A/'static_preparation.json').read_bytes())['staged_files']
    for item in stages:
        assert digest(A/'staged'/item['path']) == item['staged_sha256'], item['path']
    report = dict(status='fixture_revision_prepared_not_executed',
        previous_result=dict(passed=179, failed=4, skipped=0, actual_ros='not_started'),
        previous_manifest_sha256=digest(A/'manifest.json'), original_input_files_unchanged=len(original['files']),
        unchanged_product_files=stages, revised_python_ast=len(parsed),
        fixture_changes=['actual BypassHandoffFollow.normal.partial_policy access',
                         'unique v2 test module names and revision runner paths'],
        limits=['child EOF has no traceback in original related.log/xml',
                'v2 execution must confirm no further errors after first failing accesses'])
    write(R/'static_preparation.json', json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    inputs = {ROOT/item['path'] for item in original['files']}
    inputs.add(A/'manifest.json')
    inputs.update((A/'verify_native_v1').glob('*'))
    inputs.update(path for path in R.iterdir() if path.is_file())
    manifest = dict(schema='r417_fixture_revision_v2',
        product_change=False, original_manifest_sha256=digest(A/'manifest.json'),
        files=[dict(path=path.relative_to(ROOT).as_posix(), sha256=digest(path), bytes=path.stat().st_size)
               for path in sorted(inputs)])
    write(R/'manifest.json', json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(manifest_sha256=digest(R/'manifest.json'), files=len(inputs),
                         product_files_unchanged=len(stages), python_ast=len(parsed))))


if __name__ == '__main__':
    main()
