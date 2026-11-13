"""r460 기준 recipe/정적 연결 준비. nominal 성공 전에는 실행 동결하지 않는다."""
import argparse
import ast
import json
from pathlib import Path
import zipfile
import xml.etree.ElementTree as ET

from prepare import A, ROOT, WORLD, SCENE, CASES
from variation_launch import raw_variant, sha, write_new

BASE = A.parent/'solo_full_lap_r457_v1'


def read(path):
    return json.loads(path.read_bytes())


def case_dir(name):
    assert name in CASES
    return A/'cases'/name


def arguments(name):
    args = list(read(BASE/'prepared.json')['args'])
    args[0] = (A/'solo_runner.py').relative_to(ROOT).as_posix()
    for key, value in {'--config': A/'config.json', '--output': case_dir(name)/'run1'}.items():
        args[args.index(key)+1] = value.relative_to(ROOT).as_posix()
    return args


def baseline_inputs():
    baseline = read(BASE/'prepared.json')
    assert sha(BASE/'sources.zip') == baseline['source_archive_sha256']
    selected = {p:h for p,h in baseline['frozen_inputs'].items()
        if p.startswith(('src/', 'scripts/', 'tests/', 'config/')) or 'old_signal_610_r326_v1/' in p}
    for path in (WORLD, SCENE, BASE/'prepared.json', BASE/'sources.zip'):
        selected[path.relative_to(ROOT).as_posix()] = sha(path)
    for name, record in read(A/'baseline_copy.json').items():
        path = BASE/name
        assert sha(path) == record['baseline_sha256'], f'review changed baseline recipe: {name}'
        selected[path.relative_to(ROOT).as_posix()] = sha(path)
    changed = [p for p,h in selected.items() if not (ROOT/p).is_file() or sha(ROOT/p) != h]
    assert not changed, changed
    return selected


def nominal_success():
    report = read(BASE/'run1/report.json')
    integrity = read(BASE/'integrity.json')
    assert report['passed'] is True and report['shutdown_clean'] is True, 'nominal failure holds all variants'
    assert report['evaluation']['gates']['full_lap'] is True
    assert report['evaluation']['no_actor']['facility_count'] == 3
    assert not integrity['changed_inputs'] and integrity['archive_unchanged'] is True
    processes = read(BASE/'execution_v1/owned_processes_after.json')
    assert not processes['remaining_owned_work'], 'baseline must return WSL before variations'
    return {p.relative_to(ROOT).as_posix():sha(p) for p in
        (BASE/'run1/report.json', BASE/'integrity.json', BASE/'execution_v1/owned_processes_after.json')}


def prepare_connection():
    baseline = baseline_inputs()
    output = A/'connection_v1'
    assert not output.exists()
    output.mkdir()
    raw_files = {}
    for name in CASES:
        tree, _ = raw_variant(WORLD, name, absolute_uris=False)
        path = output/f'{name}.sdf'
        ET.ElementTree(tree).write(path, encoding='utf8', xml_declaration=True)
        raw_files[name] = dict(path=path.relative_to(ROOT).as_posix(), sha256=sha(path))
    for path in [*A.glob('*.py'), *(A/'launch').glob('*.py'), *(A/'execution').glob('*.py')]:
        ast.parse(path.read_bytes())
    assert read(A/'static_v1/result.json')['passed'] is True
    files = {p.relative_to(ROOT).as_posix():sha(p) for p in
        [*A.glob('*.py'), *A.glob('*.sdf'), A/'config.json', A/'expected_header.json', A/'baseline_copy.json',
         A/'static_v1/result.json', A/'static_check_v1.log', A/'static_check_v1.exit',
         *(A/'prepared').glob('*'), *(A/'launch').glob('*.py'), *(A/'execution').glob('*')] if p.is_file()}
    value = dict(baseline_inputs=baseline, artifact_inputs=files, raw_files=raw_files,
        args={name:arguments(name) for name in CASES}, actor_count=0, facilities=3,
        real_time_factor=.25, max_sim_seconds=80, no_progress_stop_sim_seconds=8,
        original_static_manifest_sha256=sha(A/'prepared/manifest.json'),
        original_runtime_probe_failure_sha256=sha(A/'prepared/runtime_probe_initial_failure.json'),
        control_configuration_changed=False, source_delta_from_r457=[],
        requires_successful_nominal_solo=True, execution_ready=False, simulation_started=False)
    write_new(output/'manifest.json', value)
    return dict(prepared_connection=True, frozen_for_execution=False, simulation_started=False,
        baseline_input_count=len(baseline), artifact_inputs=len(files), cases=len(CASES))


def verify_connection():
    value = read(A/'connection_v1/manifest.json')
    selected = {**value['baseline_inputs'], **value['artifact_inputs'],
        **{r['path']:r['sha256'] for r in value['raw_files'].values()}}
    changed = [p for p,h in selected.items() if not (ROOT/p).is_file() or sha(ROOT/p) != h]
    assert not changed, changed
    return value, selected


def freeze():
    value, selected = verify_connection()
    selected.update(nominal_success())
    assert read(A/'static_v1/result.json')['passed'] is True
    assert read(A/'runtime_preflight/result.json')['passed'] is True
    for path in (A/'connection_v1/manifest.json', A/'static_v1/result.json', A/'runtime_preflight/result.json'):
        selected[path.relative_to(ROOT).as_posix()] = sha(path)
    with zipfile.ZipFile(A/'execution_sources.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in selected:
            if not path.endswith('/sources.zip'):
                bundle.write(ROOT/path, path)
    value = dict(frozen_inputs=selected, args=value['args'],
        source_archive_sha256=sha(A/'execution_sources.zip'),
        nominal_success_required=True, actor_count=0, facilities=3, execution_limit_per_case=1,
        repeat_on_failure=False, control_configuration_changed=False, root_execution_review_required=True)
    write_new(A/'execution_prepared.json', value)
    return dict(frozen=True, prepared_sha256=sha(A/'execution_prepared.json'),
        source_archive_sha256=value['source_archive_sha256'], input_count=len(selected))


def verify_prepared():
    value = read(A/'execution_prepared.json')
    changed = [p for p,h in value['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p) != h]
    assert not changed, changed
    assert sha(A/'execution_sources.zip') == value['source_archive_sha256']
    return value


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--freeze', action='store_true')
    options = parser.parse_args()
    print(json.dumps(freeze() if options.freeze else
        dict(connection_unchanged=True, inputs=len(verify_connection()[1])) if options.check else prepare_connection()))
