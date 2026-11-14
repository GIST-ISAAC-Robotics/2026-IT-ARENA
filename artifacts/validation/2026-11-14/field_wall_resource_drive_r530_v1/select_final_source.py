"""root 457회귀 완료/WSL 독점 인계에 따라 r530 최종 소스를 선택한다. 실행은 별도."""
import hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'field_wall_drive_r520_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_bytes())


def main():
    assert not any((A/n).exists() for n in ('prepared.json','sources.zip','registered.json','run1'))
    selection=read(A/'source_selection.json');baseline=read(OLD/'prepared.json')['frozen_inputs']
    common={p:h for p,h in baseline.items() if p.startswith(('src/','scripts/','tests/','config/')) or 'old_signal_610_r326_v1/' in p}
    names=('follow_trace','g1_g2_node','lidar_motion_ros','lidar_safety','local_follow','partial_observation','runtime_dual_delivery','runtime_dual_state','traffic_recovery_trial')
    expected={'src/arena_autonomy/arena_autonomy/'+n+'.py' for n in names}
    noncontrol=['src/arena_autonomy/arena_autonomy/race_ops_page.py','tests/test_runtime_plan_trace.py','tests/test_independent_guard_route.py','tests/test_lidar_safety.py']
    delta=[dict(path=p,before_sha256=h,after_sha256=sha(ROOT/p)) for p,h in common.items() if sha(ROOT/p)!=h]
    assert {r['path'] for r in delta}==expected|set(noncontrol)
    selection['control_changes']=[dict(**r,role='Root final integrated r523/r524/r526/r529/r532 production') for r in delta if r['path'] in expected]
    selection['noncontrol_changes']=noncontrol;selection['reviewed_noncontrol_hashes']={p:sha(ROOT/p) for p in noncontrol}
    selection['additional_sources']=['src/arena_autonomy/arena_autonomy/guard_memory_command.py']
    selection['additional_tests']=['tests/'+n for n in ('test_compact_follow_trace.py','test_current_release_wait.py','test_fresh_current_delivery.py',
        'test_guard_memory_command.py','test_guard_memory_ray_clearance.py','test_motion_ingress_budget.py','test_operator_panel_page.py','operator_panel_page_checks.js')]
    for p in [*selection['additional_sources'],*selection['additional_tests']]:assert p not in common,p
    selection['additional_source_hashes']={p:sha(ROOT/p) for p in selection['additional_sources']}
    selection['additional_test_hashes']={p:sha(ROOT/p) for p in selection['additional_tests']}
    selection['included_integrations']=read(OLD/'source_selection.json')['included_integrations']+['r523 six-ms ingress','r524 explicit compact option; this recipe retains full trace',
        'r526 fresh-current delivery','r529 release warmup','r532 ray-supported retained-memory guard','r535 UI and DOM tests (not launched)']
    selection['pending_integrations']=[]
    selection.update(freeze_authorized=True,wsl_exclusive_owner=True,drive_authorized=True,final_production_source_selection_pending=False,
        status='Root final457 regressions passed and exclusive WSL handover; one12sim s traffic run authorized',
        root_authorization='Root explicit handover: freeze/import/competition0 then one r530 run without another approval; preserve raw result/STOP/normal shutdown, no retry')
    excluded=['scripts/'+n+'.py' for n in ('mounted_servo_setup','steering_panel_remote','jetson_wifi','jetson_wifi_remote')]
    excluded+=['tests/test_mounted_servo_setup.py','tests/test_steering_zero_profile_cli.py']
    selection['excluded_unrelated_new_files']={p:dict(sha256=sha(ROOT/p),reason='Root agrees independent hardware/operator prep unused by PC simulation') for p in excluded}
    evidence_paths=['guard_memory_ray_clearance_r532_v1/common_transfer/source.json','guard_memory_ray_clearance_r532_v1/common_transfer/result.json',
        'field_panel_contract_r535_v1/common_transfer/manifest.json','runtime_compact_trace_r524_v1/integration_v1/source.json',
        'fresh_current_delivery_r526_v1/common_transfer/manifest.json','current_release_wait_r529_v1/source.json']
    evidence_paths += ['field_guard_replay_r531_v1/'+p.name for p in (A.parent/'field_guard_replay_r531_v1').glob('*.json')]
    selection['integration_evidence_sha256']={(A.parent/p).relative_to(ROOT).as_posix():sha(A.parent/p) for p in evidence_paths}
    regression=read(A.parent/evidence_paths[1]);assert regression['passed']==regression['unique_tests']==457 and regression['failed']==0
    checks=dict(read(A.parent/evidence_paths[3])['after'])
    for p,row in read(A.parent/evidence_paths[4])['files'].items():checks[p]=row['after_sha256']
    for row in read(A.parent/evidence_paths[0])['files']:checks[row['path']]=row['integrated_sha256']
    for p,row in read(A.parent/evidence_paths[2])['files'].items():checks[p.replace('\\','/')]=row['after_sha256']
    checks['src/arena_autonomy/arena_autonomy/local_follow.py']=read(A.parent/evidence_paths[5])['stage_sha256']
    assert all(sha(ROOT/p)==h for p,h in checks.items())
    (A/'source_selection.json').write_text(json.dumps(selection,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (A/'final_source_manifest.json').write_text(json.dumps(dict(baseline_files=len(common),existing_changes=delta,
        additional_sources=selection['additional_source_hashes'],additional_tests=selection['additional_test_hashes'],
        unexpected_existing_changes=[],integration_sha_match=True,regression=regression,excluded_unrelated_new_files=selection['excluded_unrelated_new_files'],
        source_frozen=False,WSL_used=False,simulator_started=False),indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(existing_changes=len(delta),control_changes=len(selection['control_changes']),additional_sources=1,additional_tests=8,unexpected_changes=0,regressions=457)))


if __name__=='__main__':main()
