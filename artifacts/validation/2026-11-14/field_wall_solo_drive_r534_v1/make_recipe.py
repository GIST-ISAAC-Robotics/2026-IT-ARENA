"""r475 단독 평가와 r530 파생 월드를 연결하는 Windows 준비. 동결/실행 없음."""
import hashlib,json,shutil
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
SOLO=A.parent/'solo_full_lap_r475_v1';RECOVERY=A.parent/'solo_guard_recovery_r492_v1';TRAFFIC=A.parent/'field_wall_resource_drive_r530_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_bytes())


def save(p,text):
    p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x',encoding='utf-8',newline='\n') as f:f.write(text)


def write(p,data):save(p,json.dumps(data,ensure_ascii=False,indent=2)+'\n')


def main():
    assert not (A/'config.json').exists()
    copies=[]
    groups=[(SOLO,['solo_evaluation.py','solo_finalize.py','audit_runtime.py','header_comparison.py',
        'expected_header.json','facility_geometry.py','course.launch.py','launch_adapter.py']),
        (TRAFFIC,['field_world.sdf','facilities_only.sdf','world_resources.py','field_wall_geometry.py','resource_manifest.json',
        'resource_check.json','field_geometry_audit.json','field_static_geometry.json','geometry_right_side.json',
        'launch/demo.launch.py','launch/local_pursuit.launch.py','launch/simulation.launch.py','.gitignore'])]
    for source,names in groups:
        for n in names:
            target=A/n;target.parent.mkdir(parents=True,exist_ok=True);assert not target.exists();shutil.copyfile(source/n,target)
            copies.append(dict(path=n,source=(source/n).relative_to(ROOT).as_posix(),source_sha256=sha(source/n),byte_exact=True,sha256=sha(target)))
    local=[(SOLO,'solo_runner.py'),(SOLO,'stop_owned.py'),(SOLO,'execution_v1/observe.py'),
           (TRAFFIC,'execution_v1/execute_once.sh'),(TRAFFIC,'execution_v1/check_owned_processes.py'),
           (TRAFFIC,'execution_v1/stop_once.sh'),(TRAFFIC,'prepare_once.sh')]
    for source,n in local:
        original=(source/n).read_text(encoding='utf-8');content=original.replace(source.name,A.name).replace('r475','r534').replace('r530','r534')
        if n=='solo_runner.py':
            assert sha(SOLO/n)==sha(RECOVERY/n)
            content=original
            content=content.replace("    status_stream = (output/'autonomy_status.jsonl').open('w', encoding='utf-8')",
                "    status_stream = (output/'autonomy_status.jsonl').open('w', encoding='utf-8')\n    safety_stream = (output/'safety_status.jsonl').open('w', encoding='utf-8')")
            content=content.replace('    def receive_dynamics(message):',
                "    def receive_safety(message):\n        status = json.loads(message.data)\n        remember(status, 'safety')\n        safety_stream.write(json.dumps(dict(receipt_monotonic_s=time.monotonic(),data=status))+'\\n')\n\n    def receive_dynamics(message):")
            content=content.replace("lambda m: remember(json.loads(m.data), 'safety'), 10)","receive_safety, 10)")
            content=content.replace('    \n','    \n')
            content=content.replace('        status_stream.close()', '        status_stream.close()\n        safety_stream.close()')
            content=content.replace("for name in ('scans_received', 'autonomy_status'):","for name in ('scans_received', 'autonomy_status', 'safety_status'):")
        save(A/n,content)
        copies.append(dict(path=n,source=(source/n).relative_to(ROOT).as_posix(),source_sha256=sha(source/n),byte_exact=False,sha256=sha(A/n)))
    cfg=read(SOLO/'config.json');cfg['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
    cfg['scope']='r534 nominal actor0 official-course lap with r530 first-straight30mm curves/20mm joints; unmeasured sensitivity, not all-wall deformation'
    cfg['registered_at']='2026-11-14 preparation only; r530 traffic result review and final root authorization pending'
    write(A/'config.json',cfg);write(A/'expected_config.json',cfg)
    write(A/'arguments.json',[v.replace(SOLO.name,A.name) for v in read(SOLO/'prepared.json')['args']])
    write(A/'source_selection.json',dict(freeze_authorized=False,wsl_exclusive_owner=False,drive_authorized=False,
        final_production_source_selection_pending=True,source_baseline=TRAFFIC.relative_to(ROOT).as_posix(),
        control_changes=[],noncontrol_changes=[],reviewed_noncontrol_hashes={},additional_sources=[],additional_tests=[],
        additional_source_hashes={},additional_test_hashes={},included_integrations=[],
        status='Final source selection follows r530 actual traffic result review; no WSL/freeze/run yet'))
    write(A/'traffic_review_template.json',dict(root_reviewed=False,solo_authorized_after_traffic_review=False,
        original_traffic_passed=None,traffic_report_sha256=None,traffic_integrity_sha256=None,
        traffic_archive_sha256=None,traffic_normal_shutdown_and_pid0_confirmed=False,
        reason='Fill a separate traffic_review.json only after actual r530 result and resource return review; no automatic pass substitution'))
    preserved={}
    for base,names in [(SOLO,['RESULT.md','prepared.json','sources.zip','run1/report.json','solo_runner.py','solo_evaluation.py','solo_finalize.py']),
                       (RECOVERY,['RESULT.md','execution_prepared.json','solo_runner.py','cases/start_left20_yaw_plus3/run1/report.json']),
                       (TRAFFIC,['field_world.sdf','resource_manifest.json','resource_check.json','facilities_only.sdf','field_geometry_audit.json'])]:
        for n in names:preserved[(base/n).relative_to(ROOT).as_posix()]=sha(base/n)
    write(A/'recipe_provenance.json',dict(files=copies,original_evidence=preserved,original_solo_runner_equal_r475_r492=True,
        geometry_source=TRAFFIC.relative_to(ROOT).as_posix(),geometry_source_sha256=sha(TRAFFIC/'field_world.sdf'),
        unchanged_solo_speed_stop_acceptance=True,nominal_start=True,spawn_variation_added=False,
        no_progress_stop_sim_s=8,max_sim_seconds=80,actor_count=0,facilities=3,real_time_factor=.25,
        final_source_selection_pending=True,WSL_used=False,simulator_started=False))
    print(json.dumps(dict(copied=len(copies),wall_sha256=sha(A/'field_world.sdf'),actor_count=0,max_sim_seconds=80,WSL_used=False,run_count=0)))


if __name__=='__main__':main()
