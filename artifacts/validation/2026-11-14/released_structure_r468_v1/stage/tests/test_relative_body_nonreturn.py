"""고정 거리 대신 현재 질의/오차 범위를 사용하는 명시 저속 정책의 반례."""
import math
import numpy as np
import pytest

from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan, history


def case(*, neighbor=.59, point=(-.047,.0753), missing=(451,452), value=math.inf,
         padding=.1, speed=0., target=.1, relative=True, mismatch=False):
    values=[4.]*500
    for i in missing:values[i]=value
    values[min(missing)-1]=neighbor
    raw=scan(.5,values)
    scene=motion_scene(raw,history(),.6,timing_verified=True)
    if mismatch:raw.header.stamp.nanosec+=1000000
    return bounded_body_nonreturn(scene,raw,[point],np.full(500,padding),.2,.15,speed,target,
                                  relative_neighbor=relative),raw,scene


def test_default_fixed_policy_is_preserved():
    out,_,_=case(relative=False)
    assert not out['accepted'] and out['reason']=='near_gap_neighbor'
    assert out['minimum_neighbor_range_m']==.6


def test_relative_policy_uses_query_extent_and_unchanged_padding():
    out,raw,scene=case()
    assert out['accepted'] and out['policy']=='guard_relative_body_nonreturn_v1'
    assert out['minimum_neighbor_range_m']==pytest.approx(out['query_reach_m']+.1)
    assert not out['unknown_filled'] and out['residual_hidden_object_risk']
    assert math.isinf(raw.ranges[451]) and scene.rays[451].status=='unknown'
    assert out['maximum_chord_m']<=.05


@pytest.mark.parametrize('neighbor',[.05,.1,.16,.19])
def test_known_near_obstacle_and_padding_remain_blocking(neighbor):
    out,_,_=case(neighbor=neighbor)
    assert not out['accepted']


def test_larger_uncertainty_does_not_gain_permission():
    assert case()[0]['accepted']
    assert not case(padding=.6)[0]['accepted']


@pytest.mark.parametrize('value',[math.nan,-math.inf,-1.,.02])
def test_invalid_or_too_near_return_is_not_a_missing_far_ray(value):
    assert not case(value=value)[0]['accepted']


@pytest.mark.parametrize('point',[(-.101,.079),(-.047,.086),(-.047,.074)])
def test_existing_body_strip_limit_is_unchanged(point):
    assert case(point=point)[0]['reason']=='outside_small_body_side'


def test_wide_gap_and_bad_raw_timestamp_reject():
    assert not case(missing=tuple(range(446,462)))[0]['accepted']
    assert case(mismatch=True)[0]['reason']=='raw_scene_mismatch'


@pytest.mark.parametrize('speed,target',[(-.001,.1),(.151,.1),(0,.101),(0,0),(math.nan,.1),(0,math.inf)])
def test_narrow_speed_scope(speed,target):
    assert not case(speed=speed,target=target)[0]['accepted']


def test_invalid_option_does_not_select_policy():
    assert not case(relative='yes')[0]['accepted']


def test_right_side_is_symmetric():
    out,_,_=case(point=(-.047,-.0753),missing=(48,49))
    assert out['accepted'] and not out['unknown_filled']


def test_front_gap_remains_outside_scope():
    # 전방 사분면이면서 차체 옆 띠 안인 점. 직접 반환 공간이 아니라 공백이다.
    out,_,_=case(point=(.08,.0753),missing=(354,355))
    assert not out['accepted'] and out['reason']=='front_gap_outside_scope'
