"""Windows에서 recipe 확인. 최종 freeze/실행은 root 검토 후 진행한다."""
import argparse
import ast
import copy
import hashlib
import json
from pathlib import Path
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
BASE = ROOT / 'artifacts/validation/2026-11-14/solo_full_lap_r475_v1'
CASE_SOURCE = ROOT / 'artifacts/validation/2026-11-07/traffic_course_migration_v1/new_track_course.json'


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write('\n')


def verify_recipe():
    provenance = read(A / 'provenance.json')
    for path, digest in provenance['original_files'].items():
        assert sha(ROOT / path) == digest, path
    assert sha(A / 'originals.zip') == provenance['originals_zip_sha256']
    for item in provenance['copies']:
        source = ROOT / item['from']
        assert sha(source) == item['sha256']
        expected = source.read_bytes() if item['change'] == 'byte_exact' else source.read_text(encoding='utf-8').replace(
            'solo_full_lap_r475_v1', 'traffic_current_route_r478_v1').replace('r475', 'r478').replace('\r\n', '\n').encode()
        assert (A / item['to']).read_bytes() == expected, item['to']
    config = read(A / 'config.json')
    old = read(CASE_SOURCE)
    case = next(c for c in old['cases'] if c['name'] == 'straight_moving_lead')
    assert config['cases'] == [case] and config['acceptance'][case['name']] == old['acceptance'][case['name']]
    base = read(BASE / 'config.json')
    assert config['actor'] == base['actor']
    assert {k:v for k,v in config['launch'].items() if k != 'entry_g1'} == {
        k:v for k,v in base['launch'].items() if k != 'entry_g1'}
    assert config['track'] == base['track'] == 'official_v2026_11_06'
    assert config['timeouts'] == base['timeouts'] and config['seeds'] == base['seeds']
    expected = copy.deepcopy(read(BASE / 'registered.json')['args'])
    expected[0] = (A / 'run.py').relative_to(ROOT).as_posix()
    for flag, value in (('--config', (A/'config.json').relative_to(ROOT).as_posix()),
                        ('--case', case['name']), ('--output', (A/'run1').relative_to(ROOT).as_posix())):
        expected[expected.index(flag)+1] = value
    assert read(A / 'arguments.json') == expected
    assert not any('vehicle_marker_observer_enabled' in value for value in expected)
    pyfiles = sorted(A.rglob('*.py'))
    for path in pyfiles:
        ast.parse(path.read_bytes())
    for path in A.rglob('*.sh'):
        assert b'\r' not in path.read_bytes(), path
    selected = [*pyfiles, *A.rglob('*.sh'), A/'config.json', A/'arguments.json', A/'facilities_only.sdf',
                A/'expected_header.json', A/'provenance.json', A/'originals.zip']
    return dict(recipe_verified=True, recipe_files={p.relative_to(A).as_posix():sha(p) for p in selected},
        args=expected, case_exact=True, actor_r475_template_exact=True, duration_sim_s=10,
        source_case_sha256=sha(CASE_SOURCE), reference_prepared_sha256=sha(BASE/'prepared.json'),
        marker_observer_enabled=False, python_ast_files=len(pyfiles), source_frozen=False,
        wsl_accessed=False, jetson_accessed=False, build_executed=False, simulation_started=False)


def freeze():
    assert not any((A/name).exists() for name in ('prepared.json','sources.zip','registered.json','run1'))
    recipe = verify_recipe()
    assert recipe == read(A/'recipe_manifest.json')
    old = read(BASE/'prepared.json')
    assert sha(BASE/'sources.zip') == old['source_archive_sha256']
    common = {p:h for p,h in old['frozen_inputs'].items() if p.startswith(('src/','scripts/','tests/','config/'))
              or 'old_signal_610_r326_v1/' in p}
    changed = [p for p,h in common.items() if not (ROOT/p).is_file() or sha(ROOT/p) != h]
    assert not changed, changed
    selected = dict(common)
    selected.update({(A/p).relative_to(ROOT).as_posix():h for p,h in recipe['recipe_files'].items()})
    for path in (A/'recipe_manifest.json', A/'PLAN.md', BASE/'prepared.json', BASE/'build_v1/receipt.json'):
        selected[path.relative_to(ROOT).as_posix()] = sha(path)
    with zipfile.ZipFile(A/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in selected:
            bundle.write(ROOT/path, path)
    value = dict(args=recipe['args'], frozen_inputs=selected, source_archive_sha256=sha(A/'sources.zip'),
        source_policy='r475_exact_common_plus_registered_r478_recipe', common_changes=[],
        actor_count=1, facilities=3, real_time_factor=.25, max_sim_seconds=10,
        no_progress_stop_sim_seconds=8, one_execution_only=True, repeat_on_failure=False,
        root_run_confirmation_required=True, simulator_started=False, build_reused_from='r475')
    write_new(A/'prepared.json', value)
    return dict(prepared_sha256=sha(A/'prepared.json'), source_archive_sha256=value['source_archive_sha256'],
                inputs=len(selected), simulation_started=False)


def verify_prepared():
    value = read(A/'prepared.json')
    changed = [p for p,h in value['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p) != h]
    assert not changed, changed
    assert sha(A/'sources.zip') == value['source_archive_sha256']
    return value


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    for name in ('verify','freeze','check'):
        group.add_argument('--'+name, action='store_true')
    args = parser.parse_args()
    if args.verify:
        value = verify_recipe()
        write_new(A/'recipe_manifest.json', value)
    elif args.freeze:
        value = freeze()
    else:
        value = dict(checked=True, inputs=len(verify_prepared()['frozen_inputs']))
    print(json.dumps({k:v for k,v in value.items() if k not in ('recipe_files','args')}))
