"""C에서도 후보 속도를 높인 뒤 최종 경로/독립 보호를 통과하는지 검증."""
import json
import math
import numpy as np
import pytest
from dataclasses import replace
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_path import mask_scan
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan, history
from test_boundary_bridge import CONFIG

def sectors(s):
    return [(math.degrees(math.atan2(y,x-s)),math.degrees(math.asin(.0035/math.hypot(x-s,y)))+.5)
            for x,y in [(x,y) for x in [.080,-.080] for y in [.062,-.062]]]

def trial(s=.05,policy=True):
    cfg=FollowConfig(**CONFIG, boundary_bridge=True, partial_observation_policy='bounded_local_v4',
                     known_structure_policy='structure_aware_v1' if policy else 'off',
                     known_structure_sectors_json=json.dumps(sectors(s)) if policy else '')
    ctl=LocalFollow(cfg); h=history()
    h.config=replace(h.config,lidar_x_m=s)
    return ctl,h

def corridor(first,s=.05,fault=None):
    angles=-math.pi+np.arange(500)*math.tau/500
    raw=scan(first,np.minimum(8./np.maximum(abs(np.cos(angles)),1e-8),
                              .425/np.maximum(abs(np.sin(angles)),1e-8)).tolist())
    raw=mask_scan(raw,{'rear_half_angle_deg':0.,'sectors_deg':sectors(s)})
    if fault=='missing':raw.ranges[250]=float('nan')
    if fault=='side_missing':raw.ranges[361]=float('nan')  # 약 80°, 구조 각도 밖 일반 결측
    if fault=='flank_missing':raw.ranges[344]=float('nan')  # +64° 앞 지주 각도의 바깥 옆 광선
    if fault=='near':
        for i in range(242,259): raw.ranges[i]=.16
    return raw

def command(ctl,h,first,s=.05,fault=None,speed=1.):
    raw=corridor(first,s,fault); before=np.asarray(raw.ranges).copy()
    local=motion_scene(raw,h,first+.1,timing_verified=True,rear_half_angle_deg=0.)
    out=ctl.command(local,h,speed,'left',raw_scan=raw)
    np.testing.assert_array_equal(before,raw.ranges)
    return out,local

@pytest.mark.parametrize('s',[.03,.05,.065])
def test_full_controller_can_exceed_base_speed_with_c_pillars(s):
    ctl,h=trial(s)
    for first in [.3,.4,.5,.6,.7,.8]:out,local=command(ctl,h,first,s)
    v,_,meta=out
    assert v>1.2,meta
    assert meta['open_route_speed_enabled'] and meta['open_route_requires_structure_check']
    assert meta['partial_observation']['known_structure']['evaluated']
    assert meta['partial_observation']['known_structure']['unknown_filled'] is False
    assert any(r.status!='return' and r.hit_xy_m is None for r in local.rays)

def test_new_cruise_candidate_is_not_final_permission():
    ctl,h=trial()
    for first in [.3,.4,.5,.6,.7]: command(ctl,h,first)
    out,_=command(ctl,h,.8,fault='near')
    assert out[0]==0.,out[2]

@pytest.mark.parametrize('fault,reason',[('missing','bounded_forward_missing'),('side_missing','bounded_side_missing')])
def test_ordinary_missing_keeps_base_target_and_v4_cap_instead_of_stopping(fault,reason):
    # r7: 일반 결측이 있으면 1.2 m/s 초과 증속 후보를 내지 않는다. 기존 기본 목표가
    # v4 예산 판정을 받아 0보다 큰 기존 상한으로 이어진다(거리 합성·자유 공간 가정 없음).
    ctl,h=trial()
    for first in [.3,.4,.5,.6,.7]: command(ctl,h,first)
    (v,_,meta),_=command(ctl,h,.8,fault=fault)
    po=meta['partial_observation']
    assert not meta['open_route_speed_enabled'] and po['reason']==reason,meta
    assert 0.<v<=po['speed_cap_mps']<=1.2 and po['unknown_filled'] is False

def test_measured_speed_above_v4_range_keeps_existing_limit():
    # 측정 속도 자체가 1.2 m/s를 넘으면 기존 v4 제한(불허→정지)이 그대로 남는다. r7은 이를 바꾸지 않는다.
    ctl,h=trial()
    for first in [.3,.4,.5,.6,.7]: command(ctl,h,first)
    (v,_,meta),_=command(ctl,h,.8,fault='missing',speed=1.3)
    assert v==0. and meta['partial_observation']['reason']=='partial_speed_outside_trial',meta

def test_structure_with_missing_flank_is_not_excused_and_not_a_speed_up_premise():
    ctl,h=trial()
    for first in [.3,.4,.5,.6,.7]: command(ctl,h,first)
    (v,_,meta),local=command(ctl,h,.8,fault='flank_missing')
    po=meta['partial_observation']
    assert not meta['open_route_speed_enabled'] and po['reason']!='fully_observed',meta
    pillar=set(range(335,344))
    assert pillar & {r.source_index for r in local.rays if r.status!='return'}==pillar
    assert not pillar & ctl.partial_policy.structure_excused_indices
    assert any(abs(c-64.18)<.1 for c in po['known_structure']['unexplained_sector_deg'])
    assert v<=1.

def test_speed_up_premise_does_not_evaluate_the_policy_twice():
    ctl,h=trial(); calls=[]
    original=ctl.partial_policy.evaluate
    ctl.partial_policy.evaluate=lambda *a,**k: calls.append(1) or original(*a,**k)
    for first in [.3,.4,.5,.6,.7,.8]:
        command(ctl,h,first,fault='missing' if first==.8 else None)
    assert len(calls)==6 and ctl.structure_explained_now is None

def test_no_structure_policy_preserves_the_original_front_contract():
    ctl,h=trial(policy=False)
    for first in [.3,.4,.5,.6,.7]:out,_=command(ctl,h,first)
    assert out[0]==0.
