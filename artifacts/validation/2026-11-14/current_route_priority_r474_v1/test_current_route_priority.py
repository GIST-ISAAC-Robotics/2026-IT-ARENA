"""저장 current의 경로·점유·시각/STOP 경계. ROS/발행 없음."""
import copy,json,math
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace
import pytest
from arena_autonomy.race_runtime_policy import current_route_priority as policy
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
A=Path(__file__).resolve().parent

@lru_cache(maxsize=1)
def saved():
    trace=A.parent/'solo_full_lap_r469_v1/run1/follow_decisions.jsonl'
    rows=[json.loads(line) for line in trace.open(encoding='utf-8')]
    return (next(r['data']['header'] for r in rows if r['kind']=='runtime_evaluator_header'),
            next(r['data'] for r in rows if r['seq']==823))

def arguments(change=None):
    header,data=copy.deepcopy(saved());output=data['output']
    output[2]['normal']['selector_observation_policy']=policy.POLICY
    old=data['input'];current=data['current_rebase']['input']
    if change:change(output,old,current)
    _,raw,history=restore_call(current,MotionConfig(**header['motion_config']))
    scene=motion_scene(raw,history,current['reference_s'],timing_verified=current['timing_verified'],
        epoch=current['epoch'],rear_half_angle_deg=current['rear_half_angle_deg'])
    return output,old,current,scene,history,FollowConfig(**header['follow_config'])

def test_actual_point367_current_remains_positive_and_owns_original_inputs():
    args=arguments();before=copy.deepcopy(args[:3]);result=policy.rebase(*args)
    assert args[2]['speed_mps']==pytest.approx(.3673502520864255)
    assert 0<result['speed_mps']<=args[0][0]
    assert result['current_observation_policy']==policy.POLICY
    assert result['state_transition'] is False and not result['unknown_filled']
    assert result['independent_guard_required'] and not result['runtime_authorized']
    assert result['reference_s']==args[2]['reference_s'] and args[:3]==before

def test_real_finite_front_return_stops_actual_occupied_sweep():
    front=min(arguments()[3].rays,key=lambda r:abs(math.atan2(r.direction_xy[1],r.direction_xy[0])))
    def change(out,old,current):current['scan']['measurements']['ranges_m'][front.source_index]=.08
    args=arguments(change);result=policy.rebase(*args)
    assert args[3].rays[front.source_index].status=='return'
    assert result['speed_mps']==0 and result['reason']=='rebase_current_gap_hold'

def test_total_front_return_loss_stops():
    front=[r.source_index for r in arguments()[3].rays
        if abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(100)]
    def change(out,old,current):
        for i in front:current['scan']['measurements']['ranges_m'][i]={'nonfinite':'inf'}
    result=policy.rebase(*arguments(change))
    assert result['speed_mps']==0 and result['reason']=='current_route_front_observation_lost'

@pytest.mark.parametrize('fault',['path','expired','epoch','bad_dt','bad_speed'])
def test_path_time_and_motion_failure_still_stop(fault):
    def change(out,old,current):
        if fault=='path':out[2]['normal']['path_points']=[]
        elif fault=='expired':out[2]['normal']['path_source_s']-=1
        elif fault=='epoch':old['epoch']+=1
        elif fault=='bad_dt':current['dt_s']=0.
        elif fault=='bad_speed':current['speed_mps']=float('nan')
    assert policy.rebase(*arguments(change))['speed_mps']==0

@pytest.mark.parametrize('fault',['zero','STOP'])
def test_zero_and_stop_are_not_revived(fault):
    args=list(arguments())
    if fault=='zero':args[0][0]=0.
    else:args[0][2]['handoff_state']='STOP'
    assert not policy.supported(args[0]) and policy.rebase(*args) is None

@pytest.mark.parametrize('fault',['disabled','baseline_zero','memory','old_source','partial_rejected'])
def test_plan_baseline_failure_is_not_bypassed(fault):
    cfg=SimpleNamespace(traffic_recovery_trial=True,guard_observation_policy='selected_r181')
    scene=SimpleNamespace(source_first_s=1.)
    normal=dict(reason='local_follow',boundary_from_memory=False,path_source_s=1.,partial_observation={'allowed':True})
    baseline=[.8,.1,normal];view={side:{'present':False} for side in ('left','right')}
    if fault=='disabled':cfg.traffic_recovery_trial=False
    elif fault=='baseline_zero':baseline[0]=0.
    elif fault=='memory':normal['boundary_from_memory']=True
    elif fault=='old_source':normal['path_source_s']=.9
    else:normal['partial_observation']['allowed']=False
    assert policy.select(cfg,scene,baseline,view,{'selector_mode':'RELEASED'}) is None

def test_side_presence_alone_does_not_override_clear_current_baseline():
    cfg=SimpleNamespace(traffic_recovery_trial=True,guard_observation_policy='selected_r181')
    normal=dict(reason='local_follow',boundary_from_memory=False,path_source_s=1.,partial_observation={'allowed':True})
    detail={'selector_mode':'GUARD','selector_sides':{'left':{'seen':True,'present':True}}}
    out=policy.select(cfg,SimpleNamespace(source_first_s=1.),(.8,.1,normal),
        {'left':{'present':True},'right':{'present':False}},detail)
    assert out[0]==.8 and out[2]['selector_mode']=='GUARD' and out[2]['selector_sides']==detail['selector_sides']
