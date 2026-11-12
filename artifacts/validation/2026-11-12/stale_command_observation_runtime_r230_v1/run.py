"""r230: 명령 만료 중 관측 갱신 분리·80초 지연 교통1회."""
from datetime import datetime, timezone
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
BASE=ROOT/'artifacts/validation/2026-11-11/native_traffic_baseline_r87_v1'
F=ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
sys.path[:0]=[str(F),str(ROOT/'scripts'),str(ROOT/'tests')]
import validate_g1_traffic as target
from traffic_posts_setup import configure
from launch_prefix import instrument_start


def main():
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    latest='config/tests/stale_command_observation_r229_sha256.json'
    chain=[];name=latest
    while True:
        record=json.loads((ROOT/name).read_bytes());chain.append(record)
        if name=='config/tests/development_speed_limit_r162_sha256.json':break
        assert sha(ROOT/record['parent_contract'])==record['parent_contract_sha256']
        name=record['parent_contract']
    expected={}
    for record in reversed(chain):
        expected.update(record.get('new_dependencies',{}))
        for group in ('approved_changes','approved_evaluation_scripts'):
            expected.update({p:change['after'] for p,change in record.get(group,{}).items()})
    for name,digest in expected.items():assert sha(ROOT/name)==digest,name
    profile=A/'delivery_profile.json'
    evidence=json.loads(profile.read_bytes())
    for name,digest in evidence['source_sha256'].items():assert sha(ROOT/name)==digest
    assert not sys.argv[1:], 'one registered trial only'
    d=A/'two_actor_run1';d.mkdir(exist_ok=False)
    parent=BASE/'two_actor_run1/registered.json';record=json.loads(parent.read_bytes())
    args=list(record['args']);args[args.index('--output')+1]=str(d/'run1')
    args.extend(['--handoff-decision-execution','process_candidate'])
    config=json.loads((BASE/'two_actor_run1/recessed_config.json').read_bytes())
    assert config['launch']['control_rate_hz']==50.
    config['launch']['entry_g1'] = 'g1_follow.launch.py; explicit r230 delay only prefix; r229 independent stale-command observation; r227 equivalent queries'
    config['scope'] = '80초·0.8m/s 각본 상대 두 대·시설/ID20·30 누락·native C1·동일 지연. r229 명령 만료 중 관측 갱신 분리 대조.'
    case=config['cases'][0]
    assert case['name']=='two_actor_course_lap' and case['duration_s']==60.
    case['duration_s']=80.
    config['acceptance'][case['name']]['maximum_actor_laps']=1.5
    config['acceptance'][case['name']]['basis']='80초 상대64m 경로 감사. 자차1랩·원 지속/간격/노면/충돌/STOP 기준 유지.'
    case['schedule'][0]['to_s']=80.
    case['expected']='80초 동일 교통 지속 주행/1랩 확인. 원60초 실패는 보존.'
    config['vehicle_and_sensor_unchanged'] = '모터축 단일 28카운트, 11.9117647059:1(2.5*81/17), 65 mm; 현행 B0390/C1 JR60P20 모델. 이전 평가 설명의 8.970588은 역사값.'
    (d/'recessed_config.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf8')
    args[args.index('--config')+1]=str(d/'recessed_config.json')
    overlay=json.loads((BASE/'two_actor_run1/additional_actors.json').read_bytes())
    assert overlay['additional_actors'][0]['schedule'][0]['to_s']==60.
    overlay['additional_actors'][0]['schedule'][0]['to_s']=80.
    overlay['scope']='two centerline actors 0.8m/s,80seconds,initial6m apart,no map input'
    (d/'additional_actors.json').write_text(json.dumps(overlay,indent=2)+'\n')
    args[args.index('--actor-overlay')+1]=str(d/'additional_actors.json')
    assert '--lidar-test-profile' not in args
    assert args.count('--lidar-native-sampling-profile')==1
    files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
    files.update(p.relative_to(ROOT).as_posix() for p in A.glob('*') if p.is_file() and p.suffix in ('.py','.sh'))
    files.update(p.relative_to(ROOT).as_posix() for p in (
        ROOT/'artifacts/validation/2026-11-12/expiry_candidate_null_r216_v1/candidate.py',
        ROOT/'artifacts/validation/2026-11-12/guard_approach_cap_r181_v1/candidate.py',
        ROOT/'artifacts/validation/2026-11-12/adjacent_low_retry_r178_v1/candidate.py',
        ROOT/'artifacts/validation/2026-11-12/released_parallel_r173_v1/current.py',ROOT/'artifacts/validation/2026-11-12/present_guard_runtime_r171_v1/rebase.py',ROOT/'artifacts/validation/2026-11-12/present_guard_current_r170_v1/current.py',ROOT/'artifacts/validation/2026-11-12/released_runtime_r168_v1/rebase.py',ROOT/'artifacts/validation/2026-11-12/adjacent_current_r151_v1/current.py',ROOT/'artifacts/validation/2026-11-12/released_current_r167_v1/released.py',ROOT/'artifacts/validation/2026-11-12/adjacent_live_r152_v1/rebase.py',ROOT/'artifacts/validation/2026-11-12/adjacent_live_r152_v1/candidate.py',ROOT/'artifacts/validation/2026-11-12/adjacent_mount_gap_r149_v1/candidate.py',ROOT/'artifacts/validation/2026-11-12/guard_horizon_r148_v1/candidate.py',ROOT/'artifacts/validation/2026-11-11/follow_rebase_r126_v1/rebase.py',ROOT/'artifacts/validation/2026-11-12/guard_current_r142_v1/candidate.py',ROOT/'artifacts/validation/2026-11-11/guard_recheck_r139_v1/candidate.py',ROOT/'artifacts/validation/2026-11-11/guard_recheck_r139_v1/rebase.py',ROOT/'artifacts/validation/2026-11-11/body_nonreturn_r136_v1/candidate.py',F/'traffic_posts_setup.py',F/'traffic_facility_fixture.py',F/'traffic_posts_assess.py',
        F/'marker_posts_provisional.sdf',F/'marker_posts_provisional.json',
        ROOT/'artifacts/validation/2026-11-10/combined_recessed_facility_v1/audit_shapes.py',
        ROOT/'artifacts/validation/2026-11-08/masked_motion_speed_v1/assess.py',
        ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1/outline_moving.py',
        ROOT/'artifacts/validation/2026-11-10/facility_traffic_stability_v1/analyze_continuity.py',
        d/'recessed_config.json',d/'additional_actors.json',profile))
    hashes={name:sha(ROOT/name) for name in sorted(files) if (ROOT/name).is_file()}
    with zipfile.ZipFile(d/'extra_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name in hashes:z.write(ROOT/name,name)
    registration=dict(registered_at_utc=datetime.now(timezone.utc).isoformat(),
        parent=parent.relative_to(ROOT).as_posix(),parent_sha256=sha(parent),args=args,extra_sources=hashes,
        snapshot_sha256=sha(d/'extra_sources.zip'),gates=copy.deepcopy(record['gates']),
        comparison=dict(parent_control_rate_hz=50.,expected_planning_dt_s=.1,
                        decision_execution='process_candidate',independent_guard_rate_unchanged=True,
                        baseline='traffic_solo_gap_runtime_r225_v1', duration_s=80., expiry_reference='common_selected_recent_v3', static_side_clock='reference_elapsed_trapezoid_r209', released_body_gap_trial=True, guard_observation_policy='selected_r181',body_nonreturn_candidate='common_selected_r181',guard_recheck='common_selected_r181',recent_boundary_confirmation='expired_geometry_current_observation_v1',current_follow_rebase='common_selected_v1',completion_window_s=.025,delay_profile_sha256=sha(profile),
                        delay_profile_schema=evidence['schema']),
        selected_change='r229 stale commands remain zero while valid observations continue; r227 identical motion-query batch',
        scope='simulation delivery floor from earlier Jetson stage costs; original source stamp; not CPU emulation or actual latency prediction')
    (d/'registered.json').write_text(json.dumps(registration,indent=2)+'\n',encoding='utf8')
    target.start_demo_process=instrument_start(target.start_demo_process,profile,d/'launch_command.json')
    configure(target,d/'stage');sys.argv=args
    from rclpy.node import Node
    original_subscribe=Node.create_subscription
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data
    clock_probe={'count':0,'first_s':None,'last_s':None}
    def clock_received(message):
        stamp=message.clock.sec+message.clock.nanosec*1e-9
        clock_probe['count']+=1
        if clock_probe['first_s'] is None:clock_probe['first_s']=stamp
        clock_probe['last_s']=stamp
    status_stream=(d/'full_status.jsonl').open('x',encoding='utf8')
    safety_stream=(d/'full_safety.jsonl').open('x',encoding='utf8')
    def subscribe(node,msg_type,topic,callback,*rest,**kwargs):
        if topic=='/autonomy/status':
            original_subscribe(node,Clock,'/clock',clock_received,qos_profile_sensor_data)
            from std_msgs.msg import String
            original_subscribe(node,String,'/safety/status',lambda message:safety_stream.write(message.data+'\n'),10)
            base=callback
            def callback(message):
                status_stream.write(message.data+'\n');base(message)
        return original_subscribe(node,msg_type,topic,callback,*rest,**kwargs)
    Node.create_subscription=subscribe
    try:return target.main()
    finally:
        Node.create_subscription=original_subscribe;status_stream.close();safety_stream.close()
        (d/'clock_probe.json').write_text(json.dumps(clock_probe,indent=2)+'\n')
        result=dict(changed=[p for p,h in hashes.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h],
            snapshot_unchanged=sha(d/'extra_sources.zip')==registration['snapshot_sha256'])
        (d/'extra_integrity.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')


if __name__=='__main__':raise SystemExit(main())
