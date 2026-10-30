import math
from pathlib import Path
import pytest
from shapely.geometry import box
from merge_actor import MergeActor
from merge_eval import output_caps,transition_summary


class Straight:
    lap=100.
    def pose(self,s):return 10.,20.+s,math.pi/2
    def unwrap(self,s,reference):return s


def test_eval_frame_trigger_and_historical_pose():
    trial=MergeActor();line=Straight()
    initial=trial.pose({},line,1.2,{},0.)
    assert initial==pytest.approx((10.,21.2,math.pi/2))
    trial.observe(-.1,(10.,22.,math.pi/2));assert trial.scenario.start is None
    trial.observe(4.,(10.,21.14,math.pi/2));assert trial.scenario.start is None
    trial.observe(4.1,(10.,21.16,math.pi/2));assert trial.scenario.start==4.1
    assert trial.pose({},line,1.2,{},4.)==pytest.approx(initial)
    trial.observe(5.,(10.,22.,math.pi/2));assert trial.scenario.start==4.1
    assert trial.speed_at(4.2)==pytest.approx(.1)
    assert trial.speed_at(9.7)==0.


def test_route_check_restores_trigger_and_rejects_wall():
    t=MergeActor();line=Straight();road=box(9.775,19.,10.225,26.)
    cfg={'first_corner_s_m':5.3}
    r=t.validate_route(cfg,line,{},1.2,road,[])
    assert r['wall_clear'] and t.scenario.start is None
    with pytest.raises(ValueError,match='touches wall'):
        t.validate_route(cfg,line,{},1.2,road,[box(9.,21.,11.,21.4)])
    assert t.scenario.start is None


def call(t,state,speed):
    return dict(input=dict(reference_s=t),output=[speed,0.,dict(handoff_state=state)])


def test_state_caps_reject_mixed_invalid_outputs():
    assert output_caps([call(1,'FOLLOW',1.),call(2,'BYPASS',.35),call(3,'REACQUIRE',0.)])['passed']
    for c in [call(1,'BYPASS',.36),call(1,'FOLLOW',-1.),call(1,'bad',0.)]:
        assert not output_caps([c])['passed']
    assert not output_caps([])['passed']


def test_transition_times_use_decision_reference():
    calls=[call(1,'FOLLOW',.3),call(2,'BYPASS',.3),call(2.1,'BYPASS',.3),call(3,'FOLLOW',.3)]
    assert [r['time_s'] for r in transition_summary(calls,1.5,3)]==[2,3]


def test_effective_source_has_only_evaluator_trigger_hook():
    import run
    rows=run.effective_sources()
    for _,changed,name in rows:compile(changed,name,'exec')
    text=rows[1][1]
    assert text.count('MERGE_TRIAL.observe(when-t_green, ego[-1][1:])')==1
    assert 'MERGE_TRIAL' not in run.launch_source(Path('/tmp/unused'))[0]
