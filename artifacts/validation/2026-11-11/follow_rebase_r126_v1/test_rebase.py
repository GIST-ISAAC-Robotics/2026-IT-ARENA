from copy import deepcopy
import math
import numpy as np
import pytest

from rebase import rebase
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.local_follow import FollowConfig
from test_local_follow import history


def example(ranges=None):
    data=dict(reference_s=.4,dt_s=.1,epoch=0,side='left',previous_steering_rad=0.,
        speed_mps=.2,timing_verified=True,scan=dict(representation='native_measurements_v1',
        measurements=dict(source_epoch=0)))
    current={**data,'reference_s':.5}
    normal=dict(reason='local_follow',selector_mode='SOLO',selector_choice='baseline',
        path_source_s=.3,path_points=[[float(x),0.] for x in np.linspace(-.1,2.,43)])
    output=(.35,0.,dict(handoff_state='FOLLOW',normal=normal))
    scene=scene_from_scan([3.]*500 if ranges is None else ranges,angle_min=-math.pi,
        angle_increment=math.tau/500,range_min=.05,range_max=12.,
        acquired_at_s=(.4+np.arange(500)*.0002).tolist(),t_ref_s=.5)
    return output,data,current,scene,history(v=.2),FollowConfig()


def test_rebase_transports_path_and_generates_current_capped_request():
    out,data,current,scene,h,cfg=example();original=deepcopy(out)
    result=rebase(out,data,current,scene,h,cfg)
    assert 0<result['speed_mps']<=.35 and result['steering_rad']==pytest.approx(0.)
    assert result['reference_s']==.5 and result['original_reference_s']==.4
    assert result['path_source_s']==.3 and result['path_points'][0][0]==pytest.approx(-.12)
    assert result['independent_guard_required'] and not result['runtime_authorized']
    assert out==original


def test_new_current_obstacle_stops_old_positive_result():
    values=[3.]*500;values[245:256]=[.18]*11
    result=rebase(*example(values))
    assert result['speed_mps']==0 and result['reason']=='rebase_current_gap_hold'


@pytest.mark.parametrize('field,value',[('handoff_state','BYPASS'),('selector_mode','GUARD'),
    ('selector_choice','parallel'),('reason','follow_boundary_unconfirmed')])
def test_other_modes_and_lanes_are_not_rebased(field,value):
    out,*rest=example()
    if field=='handoff_state':out[2][field]=value
    else:out[2]['normal'][field]=value
    assert rebase(out,*rest) is None


def test_zero_result_is_not_replaced_by_previous_positive_plan():
    out,*rest=example();assert rebase((0.,out[1],out[2]),*rest) is None


@pytest.mark.parametrize('change', ['old_path','epoch','side','request','timing','sensor_epoch'])
def test_invalid_current_contract_does_not_fall_back_to_old_positive(change):
    out,data,current,scene,h,cfg=example()
    if change=='old_path':out[2]['normal']['path_source_s']=.1
    if change=='epoch':current['epoch']=1
    if change=='side':current['side']='right'
    if change=='request':current['previous_steering_rad']=.01
    if change=='timing':current['timing_verified']=False
    if change=='sensor_epoch':
        current=deepcopy(current);current['scan']['measurements']['source_epoch']=1
    assert rebase(out,data,current,scene,h,cfg)['speed_mps']==0


def test_missing_motion_stops_instead_of_using_unmoved_geometry():
    out,data,current,scene,h,cfg=example();h.reset()
    assert rebase(out,data,current,scene,h,cfg)['speed_mps']==0
