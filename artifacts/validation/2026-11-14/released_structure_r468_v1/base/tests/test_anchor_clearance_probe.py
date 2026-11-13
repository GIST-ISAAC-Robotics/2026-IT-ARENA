"""명시 반환만 쓰는 옛 위치 대조. 미연결 진단이며 장애물 신원 추적이 아니다."""
from dataclasses import replace
import math
import numpy as np
import pytest
from probe_anchor_clearance import fresh_clearance
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan,history


def clearance_fixture(retained=None,modify=None):
    raw=scan(.5);h=history();a=raw.angle_min+np.arange(500)*raw.angle_increment
    values=[]
    for angle in a:
        c,s=math.cos(angle),math.sin(angle)
        values.append(min((1.5-.06)/c if c>1e-8 else (-.5-.06)/c if c< -1e-8 else math.inf,
            .425/abs(s) if abs(s)>1e-8 else math.inf))
    values=np.array(values);values[np.abs(a)>math.radians(150)]=math.inf
    if modify:modify(values,a)
    raw.ranges=values.tolist()
    p,ids,_=h.project(values,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,.5,raw.time_increment,.6,'both',True)
    scene=motion_scene(raw,h,.6,timing_verified=True)
    retained=np.array([[.35,.1]]) if retained is None else np.asarray(retained,dtype=float)
    background=np.vstack((p,retained))
    return retained,background,.5,scene,h,p,ids


def test_actual_farther_returns_support_old_location_without_input_mutation():
    args=clearance_fixture();before=[x.copy() for x in (args[0],args[1],args[5],args[6])]
    result=fresh_clearance(*args)
    assert result['accepted'],result
    assert result['unknown_filled'] is False and result['identity_proven'] is False
    for old,new in zip(before,(args[0],args[1],args[5],args[6])):np.testing.assert_array_equal(old,new)


def test_current_occupied_point_cannot_be_cleared():
    r=fresh_clearance(*clearance_fixture([[.35,.425]]))
    assert not r['accepted'] and r['proofs'][0]['occupied']


def test_all_original_threats_must_be_supported():
    r=fresh_clearance(*clearance_fixture([[.35,.1],[.35,.425]]))
    assert not r['accepted'] and r['proofs'][0]['clear'] and not r['proofs'][1]['clear']


@pytest.mark.parametrize('kind',['missing','occluded'])
def test_actual_query_gap_or_closer_return_cannot_be_filled(kind):
    def modify(values,angles):
        if kind=='missing':values[(angles>.22)&(angles<.44)]=math.inf
        else:values[(angles>.20)&(angles<.5)]=.20
    r=fresh_clearance(*clearance_fixture(modify=modify))
    assert not r['accepted']


def test_rear_mask_is_not_free():
    r=fresh_clearance(*clearance_fixture([[-.2,0.]]))
    assert not r['accepted']


@pytest.mark.parametrize('reference',[float('nan'),.61,-.4000001])
def test_invalid_history_age(reference):
    args=list(clearance_fixture());args[2]=reference
    assert fresh_clearance(*args)['reason']=='invalid_or_expired'


@pytest.mark.parametrize('fault',['nan','shape','empty','oversized','id_order','id_noninteger','shifted','too_few'])
def test_invalid_point_or_scene_correspondence_rejects(fault):
    args=list(clearance_fixture())
    if fault=='nan':args[0][0,0]=math.nan
    elif fault=='shape':args[0]=np.zeros((3,3))
    elif fault=='empty':args[0]=np.empty((0,2))
    elif fault=='oversized':args[0]=np.zeros((1025,2))
    elif fault=='id_order':args[6]=args[6][::-1]
    elif fault=='id_noninteger':args[6]=args[6].astype(float)
    elif fault=='shifted':args[5]=args[5]+.001
    elif fault=='too_few':args[1]=args[1][:20]
    assert not fresh_clearance(*args)['accepted']


def test_failed_registration_never_uses_proposed_shift(monkeypatch):
    import arena_autonomy.anchor_clearance_trial as module
    monkeypatch.setattr(module,'surface_translation',lambda *a:dict(accepted=False,reason='weak_improvement',translation_m=[1.,0.]))
    assert fresh_clearance(*clearance_fixture())['reason']=='surface_rejected'


def test_time_padding_can_withhold_old_location_even_when_ray_extends_beyond():
    args=list(clearance_fixture([[.35,.31]]))
    r=fresh_clearance(*args)
    # 옛 위치보다 먼 반환이 있어도 취득 이후 이동 여유까지 확보해야 한다.
    assert not r['accepted']
