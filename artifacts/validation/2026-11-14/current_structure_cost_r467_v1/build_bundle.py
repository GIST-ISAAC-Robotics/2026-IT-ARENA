"""r463 두 정책과 실제 current 네 건을 새 ARM 계산 묶음으로 고정한다."""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
DAY = ROOT / 'artifacts/validation/2026-11-13'
R463 = DAY / 'solo_current_mask_r463_v1'
R462 = DAY / 'estimated_guard_transfer_r462_v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    bundle = A / 'bundle'
    bundle.mkdir(exist_ok=False)
    files, sources = {}, {}
    def copy(source, name):
        target = bundle / name
        target.parent.mkdir(parents=True, exist_ok=True)
        original = sha(source)
        shutil.copyfile(source, target)
        assert sha(target) == original == sha(source)
        files[name] = original
        sources[source.relative_to(ROOT).as_posix()] = original
    for version in ('base', 'stage'):
        for source in sorted((R463 / version / 'src').rglob('*')):
            if source.is_file() and source.suffix in ('.py', '.cpp', '.hpp', '.h'):
                copy(source, version + '/' + source.relative_to(R463 / version).as_posix())
        copy(R462 / 'bundle/src/arena_autonomy/setup.py', version + '/src/arena_autonomy/setup.py')
    base = {p[len('base/'):]: h for p, h in files.items() if p.startswith('base/')}
    stage = {p[len('stage/'):]: h for p, h in files.items() if p.startswith('stage/')}
    assert base.keys() == stage.keys()
    changed = [p for p in base if base[p] != stage[p]]
    integration = json.loads((R463 / 'integration.json').read_bytes())
    expected = [r for r in integration['changes'] if r['path'].startswith('src/')]
    assert set(changed) == {r['path'] for r in expected} and len(changed) == 4
    assert all(base[r['path']] == r['before'] and stage[r['path']] == r['after'] for r in expected)
    native = json.loads((R462 / 'source_manifest.json').read_bytes())['native_reuse']
    for path, digest in native['original_source_sha256'].items():
        assert base[path] == stage[path] == digest
    # r463/base는 실제 r457 sources.zip에서 추출한 원문이며 setup도 같은 원본과 대조한다.
    with zipfile.ZipFile(DAY / 'solo_full_lap_r457_v1/sources.zip') as archive:
        for path, digest in base.items():
            assert hashlib.sha256(archive.read(path)).hexdigest() == digest, path
    records = []
    source_input = R463 / 'inputs.json'
    data = json.loads(source_input.read_bytes())
    sources[source_input.relative_to(ROOT).as_posix()] = sha(source_input)
    headers = {'r457': data['header']}
    for row in data['calls']:
        value = row['data']
        records.append(dict(id=f'r457:{row["seq"]}', dataset='r457',
            plan_output=value['output'], plan_input=value['compute']['input'],
            current=value['current_rebase']['input'], retained=value['compute']['route_check']['retained'],
            expected=value['current_rebase']['output'], actual_recorded_current=True))
    heavy_source = R462 / 'bundle/inputs/currents.jsonl'
    heavy = next(json.loads(line) for line in heavy_source.read_text().splitlines()
                 if json.loads(line)['id'] == 'r452:519')
    records.append(heavy)
    headers['r452'] = json.loads((R462 / 'bundle/inputs/r452_header.json').read_bytes())
    sources[heavy_source.relative_to(ROOT).as_posix()] = sha(heavy_source)
    assert [r['id'] for r in records] == ['r457:1360', 'r457:1590', 'r457:1986', 'r452:519']
    target = bundle / 'inputs.json'
    target.write_text(json.dumps(dict(records=records, headers=headers), allow_nan=False), encoding='utf8')
    files['inputs.json'] = sha(target)
    for name in ('run_cost.py', 'execute_package.py'):
        copy(A / name, name)
    copy(R462 / 'bundle/environment_reference.py', 'environment_reference.py')
    native['source'] = '/home/q/it-arena-r462-a/src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-310-aarch64-linux-gnu.so'
    manifest = dict(files=files, original_sources=sources, policy_changes=expected,
        native_reuse=native, input_ids=[r['id'] for r in records],
        computations=24, warmups=2, rounds=3, root_reviewed=True,
        jetson_exclusive_resource_assigned=True, physical_measurement=False,
        instrumentation='round0 original functions; rounds1/2 pass-through wrappers; output exact required',
        timing_scope='Evaluator current wall only; import/init/request construction/hash/write/IPC excluded')
    for file in bundle.rglob('*.py'):
        ast.parse(file.read_bytes())
    for destination in (A / 'source_manifest.json', bundle / 'source_manifest.json'):
        destination.write_text(json.dumps(manifest, indent=2), encoding='utf8')
    archive = A / 'bundle_sources.zip'
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as z:
        for file in sorted(bundle.rglob('*')):
            if file.is_file():
                z.write(file, file.relative_to(bundle).as_posix())
    result = dict(source_count=len(files), source_manifest_sha256=sha(A / 'source_manifest.json'),
        archive_sha256=sha(archive), native_sha256=native['sha256'], policy_changes=expected,
        input_sha256=files['inputs.json'], computations=24, warmups=2)
    (A / 'prepared.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
