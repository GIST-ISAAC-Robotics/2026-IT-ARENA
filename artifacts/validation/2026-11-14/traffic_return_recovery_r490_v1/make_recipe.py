"""r478의 실제 사용 recipe를 경로만 옮긴다. source freeze/주행은 별도 단계다."""
from pathlib import Path
import ast,hashlib,json,shutil
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'traffic_current_route_r478_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    assert not (A/'recipe_provenance.json').exists()
    exact=['expected_header.json','facilities_only.sdf','facility_geometry.py']
    names=[*exact,'course.launch.py','launch_adapter.py','run.py','stop_owned.py','collect_result.py',
        'launch/demo.launch.py','launch/local_pursuit.launch.py','launch/simulation.launch.py',
        'execution_v1/execute_once.sh','execution_v1/stop_once.sh','execution_v1/observe.py',
        'execution_v1/check_owned_processes.py','execution_v1/finish_collection.bash']
    copied=[]
    for name in names:
        src=OLD/name;dst=A/name;dst.parent.mkdir(parents=True,exist_ok=True)
        if name in exact:shutil.copyfile(src,dst)
        else:dst.write_text(src.read_text(encoding='utf-8').replace('traffic_current_route_r478_v1','traffic_return_recovery_r490_v1').replace('r478','r490').replace('R478','R490'),encoding='utf-8',newline='\n')
        copied.append(dict(path=name,source=src.relative_to(ROOT).as_posix(),before_sha256=sha(src),after_sha256=sha(dst),byte_exact=name in exact))
    cfg=json.loads((OLD/'config.json').read_bytes())
    cfg['scope']='r490 same r478 10sim-second moving lead; r481/r482/r486 approved Python changes only; original acceptance retained'
    cfg['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
    (A/'config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    args=json.loads((OLD/'arguments.json').read_bytes())
    args=[x.replace('traffic_current_route_r478_v1','traffic_return_recovery_r490_v1') for x in args]
    (A/'arguments.json').write_text(json.dumps(args,indent=2)+'\n')
    changes=json.loads((A.parent/'solo_variation_recovery_r485_v1/source_selection.json').read_bytes())['policy_changes']
    r486=json.loads((A.parent/'moving_pass_recovery_r486_v1/changes.json').read_bytes())
    changes.extend(dict(path=x['path'],before_sha256=x['original_sha256'],after_sha256=x['stage_sha256']) for x in r486['files'])
    selection=dict(control_changes=changes,noncontrol_changes=['scripts/jetson_access.py'],
        additional_tests=['tests/test_current_lane_region_reuse.py','tests/test_moving_pass_current_return.py'],
        freeze_authorized=True,wsl_exclusive_owner=True,drive_authorized=False,
        root_authorization='2026-11-14 root: r486 integrated and native185 passed, exclusive WSL handed over; r490 freeze allowed, Gazebo start waits root review',
        guard_r489_integrated=False)
    (A/'source_selection.json').write_text(json.dumps(selection,indent=2)+'\n')
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    provenance=dict(source='r478 actual recipe after original helper repair',files=copied,
        old_prepared_sha256=sha(OLD/'prepared.json'),old_source_archive_sha256=sha(OLD/'sources.zip'),
        original_result_sha256=sha(OLD/'run1/report.json'),config_changed_keys=['scope','launch.entry_g1'],
        actual_simulation_started=False,native_build_required=False)
    (A/'recipe_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps(dict(copied=len(copied),cases=1,duration_sim_s=10,simulation_started=False)))
if __name__=='__main__':main()
