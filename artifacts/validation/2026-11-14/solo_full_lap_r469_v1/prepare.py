"""r465 recipe 재사용. 통합 후 실제 delta 전부를 주 담당 검토한 뒤에만 동결한다."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import zipfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
BASE=ROOT/'artifacts/validation/2026-11-13/solo_full_lap_r465_v1'
SOURCE_SUFFIXES={'.py','.yaml','.xml','.xacro','.sdf','.cpp','.hpp','.stl','.so'}
INTEGRATION=ROOT/'artifacts/validation/2026-11-14/released_structure_r468_v1/integration.json'
EXPECTED_DELTA={
    'src/arena_autonomy/arena_autonomy/pass_release_trial.py',
    'src/arena_autonomy/arena_autonomy/race_runtime_policy/current_region.py',
    'src/arena_autonomy/arena_autonomy/structure_region.py',
    'src/arena_autonomy/arena_autonomy/known_structure.py',
    'tests/test_released_structure_region.py',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def write_new(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2)
        stream.write('\n')


def arguments():
    args=list(read(BASE/'registered.json')['args'])
    args[0]=(A/'solo_runner.py').relative_to(ROOT).as_posix()
    for flag,path in (('--config',A/'config.json'),('--output',A/'run1')):
        args[args.index(flag)+1]=path.relative_to(ROOT).as_posix()
    assert '--actor-overlay' not in args
    return args


def source_inventory():
    old=read(BASE/'prepared.json')
    assert sha(BASE/'sources.zip')==old['source_archive_sha256']
    baseline={p:h for p,h in old['frozen_inputs'].items() if p.startswith(('src/','scripts/','tests/','config/'))}
    paths={ROOT/p for p in baseline}
    # r465과 같은 source 수집 범위. 이미 포함된 config/fixture는 baseline 키로 유지한다.
    paths.update(p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in SOURCE_SUFFIXES)
    paths.update((ROOT/'scripts').glob('*.py'))
    paths.update((ROOT/'tests').rglob('*.py'))
    current={p.relative_to(ROOT).as_posix():sha(p) for p in sorted(paths) if p.is_file()}
    delta=[dict(path=p,before=baseline.get(p),after=current.get(p))
        for p in sorted(baseline.keys()|current.keys()) if baseline.get(p)!=current.get(p)]
    assert {row['path'] for row in delta}==EXPECTED_DELTA, 'r469 requires exact approved five-file delta'
    integrated=read(INTEGRATION)['files']
    assert set(integrated)==EXPECTED_DELTA
    assert all(integrated[row['path']]==dict(before=row['before'],after=row['after']) for row in delta)
    return dict(baseline='r465',baseline_archive_sha256=old['source_archive_sha256'],
        baseline_prepared_sha256=sha(BASE/'prepared.json'),common_files=current,delta=delta,
        geometry_sensor_speed_physics_recipe_unchanged=True,source_frozen=False,
        awaiting_review=True,build_executed=False,simulation_started=False)


def verify_common():
    scope=read(A/'source_scope.json')
    review=read(A/'root_source_review.json')
    assert review['root_reviewed'] is True
    assert review['source_scope_sha256']==sha(A/'source_scope.json')
    assert review['baseline_archive_sha256']==scope['baseline_archive_sha256']
    assert review['approved_delta']==scope['delta'], 'every actual delta needs exact review'
    assert review['geometry_sensor_speed_physics_unchanged'] is True
    for path,digest in review['integration_evidence'].items():
        assert sha(ROOT/path)==digest,path
    current=source_inventory()
    assert current['common_files']==scope['common_files'] and current['delta']==scope['delta']
    return scope['common_files']


def freeze():
    assert not any((A/name).exists() for name in ('sources.zip','prepared.json','registered.json','run1'))
    common=verify_common()
    build=read(A/'build_v1/receipt.json')
    assert build['build_exitcode']==build['import_cli_exitcode']==0 and build['native_import_verified']
    assert build['common_sha256']==common and build['fixed_cli_reached_ros_init'] is True
    assert build['ros_initialized'] is False and build['gazebo_started'] is False
    recipe=read(A/'recipe_manifest.json')
    assert recipe['recipe_verified'] is True
    for path,digest in recipe['recipe_files'].items():
        assert sha(A/path)==digest,path
    selected=dict(common)
    old=read(BASE/'prepared.json')
    for path,digest in old['frozen_inputs'].items():
        if 'old_signal_610_r326_v1/' in path:
            assert sha(ROOT/path)==digest
            selected[path]=digest
    local=[*A.glob('*.py'),*A.glob('*.sdf'),A/'config.json',A/'expected_header.json',A/'baseline_copy.json',
        A/'recipe_manifest.json',A/'launch_command_expected.json',A/'source_scope.json',A/'root_source_review.json',A/'build_v1/receipt.json',
        *(A/'launch').glob('*.py'),*(A/'execution_v1').glob('*.py'),*(A/'execution_v1').glob('*.sh'),
        *(A/'build_v1').glob('*.py'),*(A/'build_v1').glob('*.sh')]
    for path in local:
        if path.suffix=='.py':ast.parse(path.read_bytes())
        selected[path.relative_to(ROOT).as_posix()]=sha(path)
    review=read(A/'root_source_review.json')
    selected.update(review['integration_evidence'])
    selected[(BASE/'sources.zip').relative_to(ROOT).as_posix()]=sha(BASE/'sources.zip')
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as bundle:
        for path in selected:
            if not path.endswith('/sources.zip'):bundle.write(ROOT/path,path)
    value=dict(args=arguments(),frozen_inputs=selected,source_archive_sha256=sha(A/'sources.zip'),
        baseline_archive_sha256=sha(BASE/'sources.zip'),common_delta=read(A/'source_scope.json')['delta'],
        common_files=len(common),actor_count=0,facilities=3,real_time_factor=.25,max_sim_seconds=80,
        no_progress_stop_sim_seconds=8,source_policy='r465_plus_exact_root_reviewed_delta',
        header_selection_unchanged=True,one_execution_only=True,repeat_on_failure=False,
        root_run_confirmation_required=True,simulator_started=False,build_executed=build['build_executed'])
    write_new(A/'prepared.json',value)
    return dict(prepared=True,prepared_sha256=sha(A/'prepared.json'),source_archive_sha256=value['source_archive_sha256'],
        files=len(selected),common_files=len(common),common_delta=len(value['common_delta']),simulation_started=False)


def verify_prepared():
    value=read(A/'prepared.json')
    changed=[p for p,h in value['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    assert not changed,changed
    assert sha(A/'sources.zip')==value['source_archive_sha256']
    return value


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    group=parser.add_mutually_exclusive_group(required=True)
    for flag in ('scope','verify-source','freeze','check'):group.add_argument('--'+flag,action='store_true')
    options=parser.parse_args()
    if options.scope:
        value=source_inventory()
        write_new(A/'source_scope.json',value)
        result=dict(scope_created=True,delta=len(value['delta']),source_frozen=False)
    elif options.verify_source:result=dict(common_files=len(verify_common()),reviewed=True)
    elif options.freeze:result=freeze()
    else:result=dict(checked=True,files=len(verify_prepared()['frozen_inputs']))
    print(json.dumps(result))
