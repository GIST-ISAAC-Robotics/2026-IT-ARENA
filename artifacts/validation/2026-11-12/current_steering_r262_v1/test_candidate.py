import copy
from dataclasses import replace
import math
import pytest
from test_current_path_recheck import fixture_for, SCOPES
from arena_autonomy.follow_trace import restore_call, freeze, equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_input import scan_angles
from arena_autonomy import guard_observation_policy as baseline
import candidate_policy as candidate


@pytest.mark.parametrize('scope',SCOPES)
def test_unchanged_request_keeps_all_existing_output(scope):
    args=fixture_for(scope)
    assert equal_output(freeze(candidate.rebase(*args)),freeze(baseline.rebase(*args)))


@pytest.mark.parametrize('scope',SCOPES)
@pytest.mark.parametrize('delta',(-.04,.04))
def test_new_request_is_used_for_rate_limit_without_mutating_old_input(scope,delta):
    args=fixture_for(scope)
    args[2]['previous_steering_rad']+=delta
    old=copy.deepcopy(args[:3])
    assert baseline.rebase(*args)['speed_mps']==0
    result=candidate.rebase(*args)
    assert result['reason']!='rebase_current_input_invalid'
    assert abs(result['steering_rad']-args[2]['previous_steering_rad'])<=3*args[2]['dt_s']+1e-12
    assert 0<=result['speed_mps']<=args[0][0]
    assert result['path_source_s']==args[0][2]['normal']['path_source_s']
    assert args[:3]==old


@pytest.mark.parametrize('scope',SCOPES)
@pytest.mark.parametrize('fault',('epoch','side','path_expired','clock_unverified','steering_range','front_contact'))
def test_current_fault_still_stops_with_new_previous_request(scope,fault):
    out,submitted,current,scene,history,cfg=fixture_for(scope)
    current['previous_steering_rad']+=.04
    if fault=='epoch':current['epoch']+=1
    elif fault=='side':current['side']='right' if current['side']=='left' else 'left'
    elif fault=='path_expired':out[2]['normal']['path_source_s']-=1
    elif fault=='clock_unverified':current['timing_verified']=False
    elif fault=='steering_range':current['previous_steering_rad']=cfg.max_steering_rad+.01
    else:
        _,scan,_=restore_call(current,history.config)
        angles=scan_angles(scan)+history.config.lidar_yaw_rad
        distances=list(scan.ranges_m)
        for i,angle in enumerate(angles):
            if abs(math.remainder(angle,math.tau))<.15:distances[i]=scan.range_min_m+.01
        scan=replace(scan,ranges_m=tuple(distances))
        current['scan']['measurements']['ranges_m']=distances
        scene=motion_scene(scan,history,current['reference_s'],timing_verified=True,
                           epoch=current['epoch'],rear_half_angle_deg=current['rear_half_angle_deg'])
    result=candidate.rebase(out,submitted,current,scene,history,cfg)
    assert result['speed_mps']==0


@pytest.mark.parametrize('scope',SCOPES)
def test_zero_planner_command_is_not_reopened(scope):
    out=fixture_for(scope)[0]
    out[0]=0
    assert not candidate.supported(out)
    assert candidate.rebase(out,None,None,None,None,None) is None
