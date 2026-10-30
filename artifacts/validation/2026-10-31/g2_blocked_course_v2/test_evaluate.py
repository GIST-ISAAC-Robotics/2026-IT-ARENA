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
    transitions=[dict(time_s=1,state='BYPASS'),dict(time_s=2,state='FOLLOW')]
    rows=[dict(t=2+i*.1,progress_m=i*.1,autonomy=dict(handoff_state='FOLLOW')) for i in range(61)]
    assert postreturn_progress(transitions,rows,8)==(True,6.)
    assert postreturn_progress(transitions[1:],rows,8)==(False,0.)
    # 우회로 되돌아간 뒤의 진행량을 FOLLOW 성과로 더하지 않는다.
    rows[30]['autonomy']['handoff_state']='BYPASS'
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
