"""r441 대비 실제 전체 source delta와 새 빌드/root 검토를 확인해 동결한다. 이 스크립트는 빌드/ROS를 실행하지 않는다."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
BASE=A.parent/'crop_crawl_drive_r441_v1'
SCOPE=A/'source_scope.json'
R416=A.parent/'runtime_installed_profile_r416_v1'
SIM='src/arena_bringup/launch/simulation.launch.py'
SOURCE_SUFFIXES={'.py','.yaml','.xml','.xacro','.sdf','.cpp','.hpp','.stl','.so'}
FACILITY_INPUTS=[
    'artifacts/validation/2026-11-13/old_signal_610_r326_v1/fixture.json',
    'artifacts/validation/2026-11-13/old_signal_610_r326_v1/support_frame_m.stl',
    'artifacts/validation/2026-11-13/old_signal_610_r326_v1/supports_trial.sdf',
    'artifacts/validation/2026-11-13/old610_common_corner_r333_v1/run.py',
    'artifacts/validation/2026-11-12/signal_post_revision_r291_v1/prepare_fixture.py',
    'artifacts/validation/2026-11-09/signal_post_source_v1/analyze_stl.py',
    'config/tests/c1_standard10_native_angles_v1.json']


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_bytes())
def write_new(path,data):
    with path.open('x',encoding='utf8') as stream:
        json.dump(data,stream,ensure_ascii=False,indent=2);stream.write('\n')


def arguments():
    args=list(read(BASE/'registered.json')['args'])
    for flag,path in {'--config':A/'config.json','--actor-overlay':A/'actors.json','--output':A/'run1'}.items():
        args[args.index(flag)+1]=path.relative_to(ROOT).as_posix()
    return args


def common_sources():
    files={p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in SOURCE_SUFFIXES}
    files.update((ROOT/'scripts').glob('*.py'));files.update((ROOT/'tests').rglob('*.py'))
    files.update(ROOT/row['path'] for row in read(R416/'integration.json')['files'])
    return {p.relative_to(ROOT).as_posix():sha(p) for p in sorted(files)}


def source_delta(current):
    old={p:h for p,h in read(BASE/'prepared.json')['frozen_inputs'].items()
         if p.startswith(('src/','scripts/','tests/'))}
    return [dict(path=p,before_sha256=old.get(p),after_sha256=current.get(p),
        kind='changed' if p in old and p in current else ('added_to_snapshot' if p in current else 'missing'))
        for p in sorted(old.keys()|current.keys()) if old.get(p)!=current.get(p)]


def checked_delta(current,integration):
    assert integration.resolve()==SCOPE
    scope=read(integration);files=scope['files']
    assert scope['baseline_archive_sha256']==sha(BASE/'sources.zip')
    for receipt in scope['integrations'].values():
        assert sha(ROOT/receipt['path'])==receipt['sha256']
    expected=[dict(path=p,**{k:v[k] for k in ('before_sha256','after_sha256','kind')}) for p,v in sorted(files.items())]
    actual=source_delta(current)
    assert actual==expected,('exact reviewed r441-relative actual source delta required',actual,expected)
    return actual


def draft_review():
    current=common_sources();rows=checked_delta(current,SCOPE)
    old={p:h for p,h in read(BASE/'prepared.json')['frozen_inputs'].items() if p.startswith(('src/','scripts/','tests/'))}
    return dict(status='review_required',baseline=BASE.relative_to(ROOT).as_posix(),
        baseline_archive_sha256=sha(BASE/'sources.zip'),integration_sha256=sha(SCOPE),
        changes=rows,unchanged_common_files=sum(current.get(p)==h for p,h in old.items()),
        current_common_files=len(current),selection_difference=['g1_complete_plan_trace:=true'],
        note='r441 대비 실제 전체 SHA 목록만 허용. 새 3패키지 build/import/native 확인; 같은 source 재사용 주장이 아님.')


def static_contract():
    config,old=read(A/'config.json'),read(BASE/'config.json')
    adjusted=json.loads(json.dumps(config));adjusted['scope']=old['scope'];adjusted['launch']['entry_g1']=old['launch']['entry_g1']
    assert adjusted==old and config['cases'][0]['duration_s']==80.
    for name in ('actors.json','expected_header.json','facility_geometry.py','header_comparison.py'):
        assert (A/name).read_bytes()==(BASE/name).read_bytes(),name
    adapter=(A/'launch_adapter.py').read_text(encoding='utf8').replace("    'g1_complete_plan_trace:=true',\n",'')
    assert adapter==(BASE/'launch_adapter.py').read_text(encoding='utf8')
    assert '--pilot-marker-z' not in arguments()
    stop=(A/'stop_owned.py').read_text(encoding='utf8').replace('guard_crawl_drive_r452_v1/run.py','crop_crawl_drive_r441_v1/run.py').replace("Node('r452_owned_stop')","Node('r441_owned_stop')")
    assert stop==(BASE/'stop_owned.py').read_text(encoding='utf8')
    for path in [*A.glob('*.py'),*(A/'launch').glob('*.py')]:ast.parse(path.read_bytes(),filename=str(path))
    return dict(config_equal_except_descriptions=True,actors_byte_equal=True,expected_header_byte_equal=True,
        launch_selection_equal_except_trace=True,owned_stop_body_preserved=True,selected_yaml_marker_m=.09,
        expected_mask_count=4,thresholds_changed=False,simulator_executed=False)


def freeze(review_path,integration):
    assert not any((A/n).exists() for n in ('prepared.json','sources.zip','registered.json','run1'))
    # 상대 CLI 경로도 먼저 절대 경로로 정규화한다. r418 준비 오류를 재현하지 않는다.
    review_path=review_path.resolve();integration=integration.resolve()
    assert review_path.is_relative_to(A) and integration==SCOPE
    build=read(A/'build_v1/receipt.json')
    assert build['build_executed'] is True and build['build_exitcode']==build['import_exitcode']==0
    assert build['native_import_verified'] is True
    assert build['packages']==['arena_autonomy','arena_vehicle_interface','arena_bringup']
    assert all(sha(ROOT/p)==h for p,h in build['files'].items())
    static=static_contract();current=common_sources();expected=checked_delta(current,integration)
    review=read(review_path)
    bare=[{k:row[k] for k in ('path','before_sha256','after_sha256','kind')} for row in review['changes']]
    assert bare==expected and review.get('status')=='reviewed' and review.get('root_freeze_approved') is True
    assert all(row.get('reason') for row in review['changes'])
    assert review['integration_sha256']==sha(integration) and review['baseline_archive_sha256']==sha(BASE/'sources.zip')
    frozen={ROOT/p for p in current};frozen.update(ROOT/p for p in FACILITY_INPUTS)
    frozen.update(BASE/n for n in ('prepared.json','registered.json','expected_header.json','config.json','actors.json',
        'run.py','stop_owned.py','facility_geometry.py','launch_adapter.py'))
    frozen.add(A.parent/'runtime_config_audit_r409_v1/audit_saved.py')
    frozen.add(integration);frozen.add(review_path);frozen.add(R416/'integration.json')
    frozen.update(p for p in A.glob('*') if p.is_file() and p.suffix in ('.py','.json','.md','.patch'))
    frozen.update((A/'launch').glob('*.py'))
    for folder in ('preflight_v1','execution_v1'):
        frozen.update((A/folder).glob('*.py'));frozen.update((A/folder).glob('*.sh'))
    frozen.update(ROOT/p for p in build['files']);frozen.add(A/'build_v1/receipt.json')
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in sorted(frozen)}
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    value=dict(**static,args=arguments(),frozen_inputs=hashes,source_archive_sha256=sha(A/'sources.zip'),
        reviewed_source_delta=review['changes'],source_scope_receipt=integration.relative_to(ROOT).as_posix(),
        source_approval_after_build=True,r441_source_archive_sha256=sha(BASE/'sources.zip'),
        runtime_policy='cad_runtime_r377_v1',current_corner_budget='r396_v1',crop_edge_component_trial=True,
        policy_dynamic_artifact_imports=False,real_time_factor=.25,one_execution_only=True,
        build_executed=True,complete_plan_trace=True,build_receipt='build_v1/receipt.json',root_run_confirmation_required=True)
    write_new(A/'prepared.json',value)
    return dict(frozen=True,files=len(hashes),prepared_sha256=sha(A/'prepared.json'),
        source_archive_sha256=value['source_archive_sha256'],simulation_started=False)


def verify_prepared():
    value=read(A/'prepared.json')
    changed=[p for p,h in value['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    assert not changed,('frozen inputs changed',changed)
    assert sha(A/'sources.zip')==value['source_archive_sha256']
    return value


def main():
    p=argparse.ArgumentParser(description=__doc__);choice=p.add_mutually_exclusive_group(required=True)
    choice.add_argument('--inventory',type=Path);choice.add_argument('--freeze',type=Path,metavar='REVIEW_JSON')
    choice.add_argument('--check',action='store_true');p.add_argument('--integration',type=Path)
    args=p.parse_args()
    if args.inventory:
        assert args.inventory.resolve().is_relative_to(A)
        value=draft_review();write_new(args.inventory,value);print(json.dumps(dict(changes=len(value['changes']),**static_contract())))
    elif args.freeze:
        assert args.integration is not None;print(json.dumps(freeze(args.freeze,args.integration)))
    else:print(json.dumps(dict(checked=True,files=len(verify_prepared()['frozen_inputs']),simulation_started=False)))


if __name__=='__main__':main()
