import copy

import pytest

from m4_operation_trial import OperatorTrial, TrialError
from m4_operation_eval import evaluate_trial


def initial(new_race=False):
    trial = OperatorTrial(new_race=new_race)
    race = dict(mode='b3_candidate', session='trial', race_epoch=0, vision_epoch=0,
        intervention=1, latch='process_start', rearm_baseline={'boot': 0}, rearm_required=True,
        started=False, visual_start_fact=False, policy_allowed_if_sensors_ready=False)
    mcu = dict(boot='boot', generation=0, state='DISARMED', now_us=1000000,
               estop_held=False, stationary_ready=True, output_ready=True)
    return trial, race, mcu


def step(trial, race, mcu, t):
    mcu['now_us'] = round(t*1e6)
    return trial.tick(t, race, mcu, 3.)


def accept(trial, race, t):
    p = trial.pending
    if 'payload' in p:
        v = p['payload']
        response = dict(request=v['request'], op=v['op'], accepted=True, session='trial', race_epoch=race['race_epoch'],
                        intervention=race['intervention'], latch=race['latch'])
    else:
        response = dict(success=True, message='ok')
    trial.complete(response, t)


def full_sequence(new_race=False):
    tr, r, m = initial(new_race)
    assert step(tr, r, m, 1.) is None
    assert step(tr, r, m, 2.1)['kind'] == 'initial_ack'
    r['latch'] = None
    accept(tr, r, 2.11)
    assert step(tr, r, m, 2.12)['kind'] == 'initial_arm'
    accept(tr, r, 2.13)
    m.update(state='ACTIVE', generation=1)
    r.update(started=True, visual_start_fact=True, rearm_required=False, policy_allowed_if_sensors_ready=True)
    assert step(tr, r, m, 5.)['kind'] == ('new_race' if new_race else 'policy_stop')
    r.update(latch='new_race' if new_race else 'explicit_stop', intervention=2,
             rearm_required=True, policy_allowed_if_sensors_ready=False)
    old_status = dict(r, latch=None, intervention=1)
    if new_race:
        r.update(race_epoch=1,vision_epoch=1,started=False,visual_start_fact=False)
    accept(tr, r, 5.005)
    if new_race:
        assert step(tr, old_status, m, 5.006) is None
    assert step(tr, r, m, 5.01)['kind'] == 'mcu_stop'
    accept(tr, r, 5.02)
    m.update(state='ESTOP_LATCHED', reason='software_stop', generation=2)
    assert step(tr, r, m, 5.1) is None
    assert step(tr, r, m, 6.55)['kind'] == 'resume_ack'
    r['latch'] = None
    accept(tr, r, 6.56)
    assert step(tr, r, m, 6.9) is None
    assert step(tr, r, m, 7.07)['kind'] == 'clear_stop'
    accept(tr, r, 7.08)
    m.update(state='DISARMED', generation=3, reason='cleared')
    assert step(tr, r, m, 7.09) is None
    assert step(tr, r, m, 7.6)['kind'] == 'rearm'
    accept(tr, r, 7.61)
    m.update(state='ACTIVE', generation=4)
    r.update(rearm_required=False, policy_allowed_if_sensors_ready=not new_race,
             policy_reason='wait_visual_start' if new_race else 'race_permitted')
    assert step(tr, r, m, 7.7) is None
    assert tr.phase == 'done'
    return tr, r, m


def test_new_race_rearms_but_requires_new_camera_start_fact():
    tr,r,m=full_sequence(True)
    assert tr.initial_identity == ('trial',0,0) and tr.identity == ('trial',1,1)
    assert not r['started'] and not r['policy_allowed_if_sensors_ready']


def new_race_evidence():
    data,report,ego,mcu=fixture_evidence()
    tr,r,m=full_sequence(True)
    data['operator']=tr.snapshot()
    for s in data['samples']:
        if s['time_s']>5.005:
            s['race'].update(race_epoch=1,vision_epoch=1,started=False,visual_start_fact=False,
                policy_allowed_if_sensors_ready=False,policy_reason='wait_visual_start')
        if s['time_s']>=7.7:
            s['mcu'].update(state='ACTIVE')
        s['safe_speed']=0. if s['time_s']>5.005 else .2
    ego=[(t,.2*min(t,5.5),y,a) for t,x,y,a in ego]
    report['gates']={k:True for k in ['green_observed','data_coverage','sampled_gap','swept_gap',
        'no_sampled_overlap','no_actor_contact','inside_road','ego_stays_behind']}
    report['gates'].update(hold=False,restart=False,progress_after_restart=False,final_gap=False)
    return data,report,ego,mcu


def test_new_race_negative_control_preserves_failed_old_traffic_criteria():
    data,report,ego,mcu=new_race_evidence()
    original=copy.deepcopy(report)
    result=evaluate_trial(data,report,ego,mcu)
    assert result['passed'],result
    assert report == original and report['gates']['restart'] is False


@pytest.mark.parametrize('defect',['old_fact','moving','no_arm','old_epoch','unknown_gap'])
def test_new_race_wait_requires_physical_stop_and_new_epoch(defect):
    data,report,ego,mcu=new_race_evidence()
    if defect=='old_fact': data['samples'][-1]['race']['visual_start_fact']=True
    elif defect=='moving': ego=[(t,x+.02*max(0.,t-9.),y,a) for t,x,y,a in ego]
    elif defect=='no_arm': data['samples'][-1]['mcu']['state']='DISARMED'
    elif defect=='old_epoch': data['samples'][-1]['race']['vision_epoch']=0
    elif defect=='unknown_gap': data['samples']=[s for s in data['samples'] if not 10.<s['time_s']<10.2]
    assert not evaluate_trial(data,report,ego,mcu)['passed']


def test_fixed_sequence_waits_for_distinct_ack_clear_and_arm():
    tr, _, _ = full_sequence()
    assert [e['kind'] for e in tr.events if e['event']=='request'] == [
        'initial_ack','initial_arm','policy_stop','mcu_stop','resume_ack','clear_stop','rearm']
    assert tr.stop_identity == ('boot', 2)
    assert tr.initial_arm_ok and tr.rearm_s > tr.ack_s+.5


@pytest.mark.parametrize('change', [dict(state='FAULT_LATCHED',reason='control_deadline_missed'),
                                  dict(state='ESTOP_LATCHED',reason='software_stop'),dict(boot='other')])
def test_unplanned_fault_stop_or_reboot_is_not_auto_cleared(change):
    tr, r, m = initial()
    step(tr,r,m,1.)
    m.update(change)
    with pytest.raises(TrialError):
        step(tr,r,m,2.)
    assert tr.pending is None


def test_response_loss_never_resends():
    tr,r,m=initial()
    step(tr,r,m,1.)
    step(tr,r,m,2.1)
    assert step(tr,r,m,2.5) is None
    with pytest.raises(TrialError,match='timeout'):
        step(tr,r,m,4.2)
    assert len(tr.events)==1


def test_new_session_or_intervention_is_not_silently_acknowledged():
    tr,r,m=full_sequence()
    r['intervention']+=1
    with pytest.raises(TrialError,match='intervention'):
        step(tr,r,m,8.)


def fixture_evidence():
    tr,r,m=full_sequence()
    ego=[]
    for i in range(2001):
        t=i*.01
        x=.2*min(t,5.5)+.35*max(0.,min(t,18.)-7.6)
        ego.append((t,x,0.,0.))
    samples=[]
    for i in range(50,901):
        t=i*.02
        rr=dict(r, rearm_required=5.01<=t<7.7, policy_allowed_if_sensors_ready=not(5.01<=t<7.7))
        samples.append(dict(time_s=t,race=rr,mcu=dict(m),safe_speed=0. if t<7.6 else .35,autonomy_stamp_s=t))
    actuation=[dict(m,state='ESTOP_LATCHED',reason='software_stop',generation=2,now_us=6_000_000),
               dict(m,state='DISARMED',reason='explicit_clear',generation=3,now_us=7_090_000),
               dict(m,state='ESTOP_LATCHED',reason='software_stop',generation=5,now_us=19_000_000)]
    report=dict(passed=False,run_valid=True,abort=None,gates={'original':True},
        case_window={'end_sim_s':18.},stop_request_clock_ns=19_000_000_000,
        stop_checks=dict(model_stop_stable=True,dense_pose_stop_evidence=True,explicit_mcu_stop=True,
                         no_mcu_latch_before_stop=False,no_deadline_diagnostic=True,no_wall_collision=True))
    return dict(operator=tr.snapshot(),samples=samples), report, ego, actuation


def test_evaluation_preserves_original_failure_and_only_accepts_registered_stop():
    data,report,ego,mcu=fixture_evidence()
    original=copy.deepcopy(report)
    result=evaluate_trial(data,report,ego,mcu)
    assert result['passed'],result
    assert report==original and report['passed'] is False
    assert result['registered_intermediate_latched_samples']==1


def test_clear_same_source_instant_uses_generation_and_stream_order_not_pose_clock():
    data,report,ego,mcu=fixture_evidence()
    data['operator']['stop_closed_s']=7.089  # 평가 pose 시각은 MCU보다 1 ms 뒤처짐
    mcu.insert(1,dict(mcu[0],now_us=7_090_000))
    assert evaluate_trial(data,report,ego,mcu)['passed']
    mcu.insert(3,dict(mcu[0],now_us=7_090_000))  # CLEAR 관측 뒤 옛 STOP은 미허용
    assert not evaluate_trial(data,report,ego,mcu)['passed']


@pytest.mark.parametrize('defect',['unplanned_stop','fault','wrong_generation','pose_gap','moving_during_ack',
    'early_permission','stale_status','lost_visual','new_epoch','no_prior_motion','legacy_integrity','missing_clear'])
def test_invalid_stop_or_missing_physical_evidence_cannot_pass(defect):
    data,report,ego,mcu=fixture_evidence()
    if defect=='unplanned_stop': mcu.append(dict(mcu[0],now_us=4_000_000))
    elif defect=='fault': mcu.append(dict(mcu[0],state='FAULT_LATCHED'))
    elif defect=='wrong_generation': mcu[0]['generation']=3
    elif defect=='pose_gap': ego=[p for p in ego if not 6.7<p[0]<7.]
    elif defect=='moving_during_ack': ego=[(t,x+.02*max(0.,t-6.5),y,a) for t,x,y,a in ego]
    elif defect=='early_permission': data['samples'][290]['race']['policy_allowed_if_sensors_ready']=True
    elif defect=='stale_status': data['samples'][290]['autonomy_stamp_s']-=1.
    elif defect=='lost_visual': data['samples'][300]['race']['visual_start_fact']=False
    elif defect=='new_epoch': data['samples'][300]['race']['race_epoch']=1
    elif defect=='no_prior_motion': ego=[(t,0. if t<5.5 else x,y,a) for t,x,y,a in ego]
    elif defect=='legacy_integrity': report['run_valid']=False
    elif defect=='missing_clear': mcu.pop(1)
    assert evaluate_trial(data,report,ego,mcu)['passed'] is False
