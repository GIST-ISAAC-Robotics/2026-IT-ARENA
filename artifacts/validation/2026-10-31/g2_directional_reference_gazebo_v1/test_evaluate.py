"""새 모드 평가가 빈 증거·실제 실패를 통과로 바꾸지 않는지 검사한다."""
import copy
from evaluate import checks_for,postreturn_progress,INTEGRITY,STOP,COMMON


def valid():
    report=dict(evaluation=dict(harness=dict(valid=True),gates={k:True for k in COMMON}),
        integrity={k:True for k in INTEGRITY},stop_checks={k:True for k in STOP},shutdown_clean=True,
        additional_actors=dict(harness_valid=True,combined_contacts=dict(valid=True,ego_events=0),actors={
            'side':dict(gates=dict(complete_gap_samples=True,complete_swept_intervals=True,no_ego_contact=True),min_sampled_gap_m=.07)}))
    runtime=dict(runtime_contract=dict(passed=True),source_unchanged=True)
    surface=dict(samples=1,sample_gaps_over_limit=0,wall_wheel_samples=0,
        interpretations={k:dict(max_off_wheels=2,penalty_events_merged=0) for k in ('any','center','full')})
    replay=dict(passed=True,input_unchanged=True,calls=10,matched=10)
    return [report,runtime,surface,True,True,replay]


def test_registered_positive_and_old_g1_not_required():
    args=valid();args[0]['passed']=False;args[0]['integrity']['controller_mode_matches']=False
    assert all(checks_for(*args).values())


def test_full_course_requires_full_lap():
    assert 'full_lap' in COMMON
    for value in (False,None):
        args=valid();args[0]['evaluation']['gates']['full_lap']=value
        assert checks_for(*args)['course_full_lap'] is False
    assert checks_for(*valid())['course_full_lap'] is True


def test_each_required_integrity_stop_and_course_failure_survives():
    for group,keys in (('integrity',INTEGRITY),('stop_checks',STOP)):
        for key in keys:
            for value in (False,None):
                args=valid();args[0][group][key]=value
                assert not all(checks_for(*args).values()),(group,key)
    for key in COMMON:
        args=valid();del args[0]['evaluation']['gates'][key]
        assert not all(checks_for(*args).values()),key


def test_missing_actor_contact_or_replay_evidence_fails():
    args=valid();args[0]['additional_actors']={}
    assert not all(checks_for(*args).values())
    args=valid();args[0]['additional_actors']['combined_contacts']['ego_events']=1
    assert not all(checks_for(*args).values())
    args=valid();args[5]['matched']=9
    assert not all(checks_for(*args).values())


def test_surface_and_modes_do_not_bypass_gate():
    for i in (3,4):
        args=valid();args[i]=False;assert not all(checks_for(*args).values())
    args=valid();args[2]['interpretations']['any']['max_off_wheels']=3
    assert not all(checks_for(*args).values())
    args=valid();args[2]['interpretations'].pop('full')
    assert not all(checks_for(*args).values())


def test_return_requires_bypass_and_continuous_follow_progress():
    transitions=[dict(time_s=1,state='BYPASS'),dict(time_s=2,state='FOLLOW',reason='handoff_return')]
    rows=[dict(t=2+i*.1,progress_m=i*.1,autonomy=dict(mode='g1_g2_handoff_trial',behavior='FOLLOW')) for i in range(61)]
    assert postreturn_progress(transitions,rows,8)==(True,6.)
    assert postreturn_progress(transitions[1:],rows,8)==(False,0.)
    # 우회로 되돌아간 뒤의 진행량을 FOLLOW 성과로 더하지 않는다.
    rows[30]['autonomy']['behavior']='BYPASS'
    assert postreturn_progress(transitions,rows,8)[1]<5.
    # 입력 공백 역시 이어진 FOLLOW로 합산하지 않는다.
    rows=[r for i,r in enumerate(rows) if not 28<=i<=32]
    assert postreturn_progress(transitions,rows,8)[1]<5.


def test_actor_overlay_must_match_actual_case():
    import json
    from pathlib import Path
    import pytest
    from traffic_actors import overlay_case,actor_specs
    base=Path(__file__).resolve().parent
    config=json.loads((base/'traffic_config.json').read_text())
    case=config['cases'][0]
    assert len(actor_specs(config,overlay_case(case,base/'actor_overlay.json')))==2
    with pytest.raises(ValueError,match='matching base case'):
        overlay_case(dict(case,name='wrong_case'),base/'actor_overlay.json')


def test_blocked_route_keeps_actor_geometry_and_registered_schedule():
    import json
    from pathlib import Path
    import pytest
    from shapely.geometry import box
    from blocked_route import blocked_route
    config=json.loads((Path(__file__).resolve().parent/'traffic_config.json').read_text())
    case=config['cases'][0]
    class Line:
        lap=10.
        def pose(self,s):return s,0.,0.
    line=Line();road=box(-1,-1,3,1)
    assert blocked_route(config,line,case,0.,road,[])['actor_travel_m']==1.4
    with pytest.raises(ValueError,match='outside road'):
        blocked_route(config,line,case,0.,box(-1,-1,.5,1),[])
    with pytest.raises(ValueError,match='into wall'):
        blocked_route(config,line,case,0.,road,[box(.3,-.1,.5,.1)])
    changed=copy.deepcopy(case);changed['schedule'][-1]['speed_mps']=.1
    with pytest.raises(ValueError,match='registered blocked route'):
        blocked_route(config,line,changed,0.,road,[])


def test_fault_reset_is_not_a_successful_return():
    rows=[dict(t=2+i*.1,progress_m=i*.1,autonomy=dict(mode='g1_g2_handoff_trial',behavior='FOLLOW')) for i in range(61)]
    for reason in ('handoff_input_fault','handoff_reverse_motion_hold','handoff_source_reset',None):
        transitions=[dict(time_s=1,state='BYPASS'),dict(time_s=2,state='FOLLOW',reason=reason)]
        assert postreturn_progress(transitions,rows,8)==(False,0.)


def test_launch_dependency_must_exist_and_be_in_snapshot(tmp_path):
    import importlib.util,hashlib
    from pathlib import Path
    import pytest
    path=Path(__file__).with_name('verify_prepared.py')
    spec=importlib.util.spec_from_file_location('launch_dependency_check',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    base=tmp_path/'artifacts/validation/date/trial';base.mkdir(parents=True)
    with pytest.raises(AssertionError,match='missing'):module.verify_launch_dependency(base,{})
    launch=base/'simulation.launch.py';launch.write_text('def generate_launch_description(): return None\n')
    with pytest.raises(AssertionError,match='not in source snapshot'):module.verify_launch_dependency(base,{})
    key=launch.relative_to(base.parents[3]).as_posix()
    hashes={key:hashlib.sha256(launch.read_bytes()).hexdigest()}
    module.verify_launch_dependency(base,hashes)
    launch.write_text('changed=True\n')
    with pytest.raises(AssertionError,match='changed'):module.verify_launch_dependency(base,hashes)


def test_actual_runtime_trace_schema_matches_distance():
    import json
    from pathlib import Path
    case=json.loads(Path(__file__).with_name('trajectory_schema_fixture.json').read_text())
    rows=case['rows'];distance=rows[-1]['progress_m']-rows[0]['progress_m']
    assert distance>0
    assert postreturn_progress(case['transitions'],rows,rows[-1]['t'])==(True,distance)
    # 다른 제어 모드에서 같은 behavior 이름을 출력해도 G1/G2 복귀로 세지 않는다.
    for row in rows:row['autonomy']['mode']='other'
    assert postreturn_progress(case['transitions'],rows,rows[-1]['t'])==(True,0.)


