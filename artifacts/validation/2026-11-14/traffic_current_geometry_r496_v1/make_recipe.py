"""r490 실제 recipe의 경로 복사만 준비. 공통 동결/WSL/주행 실행 없음."""
from pathlib import Path
import ast,hashlib,json,shutil
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'traffic_return_recovery_r490_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def renamed(text):
    return text.replace('traffic_return_recovery_r490_v1','traffic_current_geometry_r496_v1').replace('r490','r496').replace('R490','R496')

def main():
    assert not (A/'recipe_provenance.json').exists()
    exact=['expected_header.json','facilities_only.sdf','facility_geometry.py']
    names=[*exact,'course.launch.py','launch_adapter.py','run.py','stop_owned.py','collect_result.py',
        'launch/demo.launch.py','launch/local_pursuit.launch.py','launch/simulation.launch.py',
        'execution_v1/execute_once.sh','execution_v1/stop_once.sh','execution_v1/observe.py',
        'execution_v1/check_owned_processes.py','execution_v1/finish_collection.bash','prepare_once.sh']
    copied=[]
    for name in names:
        source=OLD/name;target=A/name;target.parent.mkdir(parents=True,exist_ok=True)
        if name in exact:shutil.copyfile(source,target)
        else:target.write_text(renamed(source.read_text(encoding='utf-8')),encoding='utf-8',newline='\n')
        copied.append(dict(path=name,source=source.relative_to(ROOT).as_posix(),before_sha256=sha(source),
            after_sha256=sha(target),byte_exact=name in exact))
    cfg=json.loads((OLD/'config.json').read_bytes())
    cfg['scope']='r496 same r490 single 10sim-second moving lead; planned r489/r494/float/r491/r495 integration; original acceptance retained'
    cfg['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
    (A/'config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    args=[renamed(x) for x in json.loads((OLD/'arguments.json').read_bytes())]
    (A/'arguments.json').write_text(json.dumps(args,indent=2)+'\n',encoding='utf-8')
    text=(OLD/'prepare.py').read_text(encoding='utf-8')
    text=text.replace("OLD=A.parent/'traffic_current_route_r478_v1'","OLD=A.parent/'traffic_return_recovery_r490_v1'")
    text=text.replace("'traffic_current_route_r478_v1','traffic_return_recovery_r490_v1'", "'traffic_return_recovery_r490_v1','traffic_current_geometry_r496_v1'")
    text=text.replace('원 r478 입력과 승인된 5개 제어 Python 변경만 새 실행으로 동결한다.',
        '원 r490 입력과 root 승인 delta만 새 실행으로 동결한다. 현재 recipe 준비만으로는 실행되지 않는다.')
    text=text.replace('assert len(allowed)==5','assert allowed')
    text=text.replace("assert sum(x['role']=='control_python' for x in delta)==5", "assert sum(x['role']=='control_python' for x in delta)==len(allowed)")
    text=text.replace("for p in source['additional_tests']:","for p in [*source['additional_tests'],*source.get('additional_sources',[])]:")
    text=text.replace('r489_guard_integrated=False,build_reused=True',
        "included_integrations=read(A/'source_selection.json')['included_integrations'],build_reused=True")
    (A/'prepare.py').write_text(text,encoding='utf-8',newline='\n')
    text=renamed((OLD/'preflight.py').read_text(encoding='utf-8'))
    text=text.replace("p=item['path'];name=", "p=item['path']\n        if not p.startswith('src/arena_autonomy/arena_autonomy/'):continue\n        name=")
    (A/'preflight.py').write_text(text,encoding='utf-8',newline='\n')
    selection=dict(control_changes=[],noncontrol_changes=[],additional_sources=[],additional_tests=[],
        freeze_authorized=False,wsl_exclusive_owner=False,drive_authorized=False,included_integrations=[],
        status='waiting_for_root_integration_and_WSL_return',
        pending_integrations=['r489 independent guard','r494 current moving guide geometry','float serialization',
                              'r491 sender two files','r495 installed max_wheel parameter two files'],
        root_authorization='recipe preparation only; root must reconcile the complete delta against r490 after integration')
    (A/'source_selection.json').write_text(json.dumps(selection,indent=2)+'\n',encoding='utf-8')
    r494=json.loads((A.parent/'traffic_current_geometry_r494_v1/source_manifest.json').read_text(encoding='utf-8'))
    (A/'expected_r494_hashes.json').write_text(json.dumps(dict(source_manifest_sha256=sha(A.parent/'traffic_current_geometry_r494_v1/source_manifest.json'),
        files=r494['files'],purpose='review only; not current common source freeze'),indent=2),encoding='utf-8')
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    for p in A.rglob('*.sh'):assert b'\r' not in p.read_bytes(),p
    expected=json.loads((OLD/'config.json').read_bytes());restored=json.loads((A/'config.json').read_bytes())
    restored['scope']=expected['scope'];restored['launch']['entry_g1']=expected['launch']['entry_g1'];assert restored==expected
    assert not any((A/p).exists() for p in ('prepared.json','sources.zip','registered.json','run1'))
    provenance=dict(source='r490 actual recipe',files=copied,source_prepared_sha256=sha(OLD/'prepared.json'),
        original_result_sha256=sha(OLD/'run1/report.json'),config_changed_keys=['scope','launch.entry_g1'],
        actual_simulation_started=False,current_common_frozen=False,one_execution_only=True,repeat_on_failure=False)
    (A/'recipe_provenance.json').write_text(json.dumps(provenance,indent=2),encoding='utf-8')
    print(json.dumps(dict(copied=len(names),cases=1,duration_sim_s=10,current_common_frozen=False,simulation_started=False)))
if __name__=='__main__':main()
