import pytest
from g1_motion_recovery_eval import evaluate

CONFIG={'imu':{'delay_s':.08,'start_s':8.,'end_s':10.}}


def evidence():
    ego=[];x=0.
    for i in range(1501):
        t=i*.01
        v=0. if 8.2<=t<=10.5 else .4
        x+=v*.01
        ego.append([t,x,0.,0.])
    trace=[dict(t=t,x=x,y=y,progress_m=x,ground_speed_mps=0. if 8.2<=t<=10.5 else .4,
                safe_speed_mps=0. if 8.1<=t<=10.5 else .4,
                safety_reason='motion_stale' if 8.1<=t<=10.5 else 'clear') for t,x,y,_ in ego[::10]]
    return trace,ego


def test_stable_stop_and_recovery_need_sampled_and_dense_evidence():
    trace,ego=evidence();r=evaluate(trace,ego,CONFIG)
    assert r['passed'] and r['dense_stop']['complete']
    assert r['sampled_stop_recovery']['resumed_more_than_1m']


@pytest.mark.parametrize('fault',['pose_gap','hidden_movement','trace_gap','no_recovery','no_block','initially_stopped'])
def test_missing_or_contradictory_evidence_cannot_pass(fault):
    trace,ego=evidence()
    if fault=='pose_gap':ego=[r for r in ego if not 8.25<r[0]<8.4]
    if fault=='hidden_movement':ego[828][1]+=.01
    if fault=='trace_gap':trace=[r for r in trace if not 8.1<r['t']<8.7]
    if fault=='no_recovery':
        for r in trace:
            if r['t']>10:r['ground_speed_mps']=0.
    if fault=='no_block':
        for r in trace:r['safety_reason']='clear'
    if fault=='initially_stopped':
        for r in trace:
            if r['t']<8:r['ground_speed_mps']=0.
    assert not evaluate(trace,ego,CONFIG)['passed']


def test_wrong_profile_and_nonincreasing_input_rejected():
    trace,ego=evidence()
    with pytest.raises(ValueError):evaluate(trace,ego,{'imu':{'drop':True}})
    with pytest.raises(ValueError):evaluate(trace,ego,{**CONFIG,'wheels':{'scale':.95}})
    trace[10]['t']=trace[9]['t']
    with pytest.raises(ValueError):evaluate(trace,ego,CONFIG)
