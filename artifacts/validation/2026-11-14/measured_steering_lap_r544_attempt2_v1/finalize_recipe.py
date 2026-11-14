"""r544 승인 후 레시피 확정. Windows에서만 실행하며 ROS/주행은 시작하지 않는다."""
import ast,hashlib,json,re
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'field_wall_solo_drive_r534_v1';TRAFFIC=A.parent/'field_wall_guard_relevance_drive_r539_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_bytes())
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
def edit(name,changes):
    p=A/name;s=p.read_text(encoding='utf-8')
    for before,after in changes:
        assert before in s,(name,before);s=s.replace(before,after)
    p.write_text(s,encoding='utf-8',newline='\n')
def main():
    assert not (A/'final_recipe_manifest.json').exists()
    edit('prepare.py',[
      ("TRAFFIC=A.parent/'field_wall_resource_drive_r530_v1'","TRAFFIC=A.parent/'field_wall_guard_relevance_drive_r539_v1'"),
      ("assert not integrity['changed_inputs'] and integrity['archive_unchanged']","assert integrity['changed_inputs']==['scripts/probe_esp32_motor.py'] and integrity['archive_unchanged']\n    actual=read(TRAFFIC/'actual_input_audit.json')\n    assert not actual['actual_snapshot_hash_mismatches'] and not actual['actual_live_input_changes_at_collection']\n    assert not actual['actual_vs_frozen_hash_mismatches'] and actual['local_sensor_profile']['unchanged_vs_frozen_and_runtime_metadata']\n    assert review['traffic_actual_input_audit_sha256']==sha(TRAFFIC/'actual_input_audit.json')"),
      ("assert arguments()==[v.replace(SOLO.name,A.name) for v in read(SOLO/'prepared.json')['args']]","expected_args=[v.replace(SOLO.name,A.name) for v in read(SOLO/'prepared.json')['args']]\n    expected_args[expected_args.index('--vehicle-config')+1]='src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml'\n    assert arguments()==expected_args"),
      ("elif before!=after:assert p in selection['noncontrol_changes'],p","elif before!=after:assert p in selection['noncontrol_changes'],p"),
      ("value=dict(**info,frozen_inputs=selected,source_archive_sha256=sha(A/'sources.zip'),","required,observed=classify(selected)\n    value=dict(**info,frozen_inputs=required,observed_inputs=observed,source_archive_sha256=sha(A/'sources.zip'),"),
      ("assert not changed,changed","observed=[p for p,h in data.get('observed_inputs',{}).items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]\n    data['observational_changes_at_check']=observed\n    assert not changed,changed")])
    # 전체 목록은 ZIP에 남기고 실행에 사용되는 범위를 별도로 강제한다.
    p=A/'prepare.py';s=p.read_text();pos=s.index('\ndef freeze():')
    s=s[:pos]+'''\ndef classify(selected):
    actual=set(read(TRAFFIC/'run1/report.json')['source_sha256'])|set(read(SOLO/'run1/report.json')['source_sha256'])
    local=A.relative_to(ROOT).as_posix()+'/'
    required={p:h for p,h in selected.items() if p in actual or p.startswith(('src/',local)) or 'old_signal_610_r326_v1/' in p}
    for flag in ('--vehicle-config','--local-sensor-config','--lidar-native-sampling-profile'):
        args=arguments();p=args[args.index(flag)+1];required[p]=selected[p]
    for p in selected:
        if p.startswith(TRAFFIC.relative_to(ROOT).as_posix()+'/'):required[p]=selected[p]
    observed={p:h for p,h in selected.items() if p not in required}
    return required,observed

'''+s[pos:];p.write_text(s,encoding='utf-8',newline='\n')
    edit('run.py',[
      ("configure(target,prepared['frozen_inputs']);args=arguments()","configure(target,prepared['frozen_inputs']);args=arguments()"),
      ("input_count=len(prepared['frozen_inputs']),","input_count=len(prepared['frozen_inputs']),\n            observational_changes=[p for p,h in prepared.get('observed_inputs',{}).items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h],\n            observational_count=len(prepared.get('observed_inputs',{})),")])
    # 実際 snapshotにも選択センサ YAML・native/package closure を保持。
    edit('collect_evidence.py',[("if payload[0]>0 and normal.get('selector_mode')=='FOLLOW':normal_positive+=1","if payload[0]>0 and (payload[2] or {}).get('handoff_state')=='FOLLOW' and normal.get('reason')=='local_follow' and not normal.get('current_wall_reacquire'):normal_positive+=1")])
    p=A/'launch/simulation.launch.py';s=p.read_text();s=s.replace('generate_launch_description=module.generate_launch_description','from steering_runtime_audit import install\ninstall(module)\ngenerate_launch_description=module.generate_launch_description');p.write_text(s,encoding='utf-8',newline='\n')
    baseline=read(TRAFFIC/'prepared.json');selection=read(A/'source_selection.json');delta=[]
    for p,h in baseline['frozen_inputs'].items():
        if p.startswith(('src/','scripts/','tests/','config/')) and sha(ROOT/p)!=h:delta.append(dict(path=p,before_sha256=h,after_sha256=sha(ROOT/p)))
    expected='src/arena_autonomy/arena_autonomy/guard_memory_command.py'
    assert {x['path'] for x in delta}=={expected,'scripts/probe_esp32_motor.py'},delta
    assert sha(ROOT/expected)=='0bc97fe6ec5e1fa596fa0f536f9706b85c09b4fd920c78b3165df163bc957742'
    selection.update(freeze_authorized=True,wsl_exclusive_owner=True,drive_authorized=True,final_production_source_selection_pending=False,
        control_changes=[x for x in delta if x['path']==expected],noncontrol_changes=['scripts/probe_esp32_motor.py'],
        reviewed_noncontrol_hashes={'scripts/probe_esp32_motor.py':sha(ROOT/'scripts/probe_esp32_motor.py')},
        included_integrations=['r540 exact-angle reuse','r542 explicit measured-count-derived vehicle YAML'],status='root final handover received; one solo run allowed, no repeat')
    new_test='tests/test_guard_memory_angle_cache.py'
    if (ROOT/new_test).exists():selection['additional_tests']=[new_test];selection['additional_test_hashes']={new_test:sha(ROOT/new_test)}
    write(A/'source_selection.json',selection)
    review=dict(root_reviewed=True,solo_authorized_after_traffic_review=True,original_traffic_passed=True,
        traffic_report_sha256=sha(TRAFFIC/'run1/report.json'),traffic_integrity_sha256=sha(TRAFFIC/'integrity.json'),
        traffic_archive_sha256=sha(TRAFFIC/'sources.zip'),traffic_actual_input_audit_sha256=sha(TRAFFIC/'actual_input_audit.json'),
        traffic_normal_shutdown_and_pid0_confirmed=True,reason='root r544 execution handover; raw PASS and unrelated whole-list drift retained separately')
    write(A/'traffic_review.json',review)
    provenance=read(A/'recipe_provenance.json')
    for item in provenance['files']:item['sha256']=sha(A/item['path'])
    provenance.update(final_source_selection_pending=False,explicit_measured_steering_selected=True,source_baseline=TRAFFIC.relative_to(ROOT).as_posix())
    write(A/'recipe_provenance.json',provenance)
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    cfg=read(A/'config.json');old=read(OLD/'config.json');assert cfg['acceptance']==old['acceptance'] and cfg['cases']==old['cases']
    assert sha(A/'field_world.sdf')==sha(OLD/'field_world.sdf')=='c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09'
    assert sha(A/'solo_runner.py')==sha(OLD/'solo_runner.py')
    write(A/'final_recipe_manifest.json',dict(passed=True,source_delta=delta,original_acceptance_stop_speed_unchanged=True,actor_count=0,
        maximum_sim_seconds=80,real_time_factor=.25,no_progress_stop_sim_seconds=8,world_sha256=sha(A/'field_world.sdf'),
        vehicle_config=selection['additional_sources'][0],vehicle_sha256=selection['additional_source_hashes'][selection['additional_sources'][0]],
        observed_vs_required_inventory=True,WSL_used=False,actual_runs=0))
    print('r544 Windows recipe final; final WSL import/freeze/run pending')
if __name__=='__main__':main()
