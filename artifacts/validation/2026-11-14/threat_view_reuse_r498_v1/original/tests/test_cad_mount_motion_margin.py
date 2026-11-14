"""장착 흔들림 여유의 공간 범위와 거리 제한 자기 반환 유지."""
import json
import math
import numpy as np
import pytest

from arena_autonomy.cad_mount import bounds
from arena_autonomy.local_scene import blind_parameter
from arena_autonomy.local_path import mask_scan
from test_cad_mount_profile import make_scan, profile, MODEL, launch_module


def test_zero_margin_is_old_bounds_and_shifted_vertices_are_enclosed():
    polygon=[[.03,.05],[.04,.05],[.04,.06],[.03,.06]]
    origin=(.0754,0.)
    assert bounds(polygon,origin,.5,.002)==bounds(polygon,origin,.5,.002,0.)
    sector,band=bounds(polygon,origin,.5,.002,.003)
    for angle in np.linspace(-math.pi,math.pi,73):
        delta=.003*np.array([math.cos(angle),math.sin(angle)])
        for point in polygon:
            p=np.array(point)+delta-np.array(origin)
            bearing=math.degrees(math.atan2(p[1],p[0]))
            assert abs(math.remainder(bearing-sector[0],360))<=sector[1]
            assert band[0]<=np.linalg.norm(p)<=band[1]


def test_nearer_structure_gets_more_angle_for_same_linear_motion():
    p=[[.1,-.003],[.11,-.003],[.11,.003],[.1,.003]]
    def extra(points):return bounds(points,(0.,0.),.5,.002,.003)[0][1]-bounds(points,(0.,0.),.5,.002)[0][1]
    assert extra(p)>extra([[x*2,y*2]for x,y in p])


def test_expanded_mask_preserves_return_outside_near_band():
    p=profile('frontwire_4mm')
    p['lidar']['mount_motion_margin_m']=.003
    _,text=launch_module()._mount_trial(MODEL,p,.0325,.06,[-.1005,0.,.18],.05,True)
    spec=blind_parameter(0.,text)
    center=math.radians(spec['sectors_deg'][0][0]);lo,hi=spec['range_limits_m'][0]
    scan=make_scan([(lo+hi)/2,hi+.02,lo-.01,math.inf],[center]*4)
    result=mask_scan(scan,spec)
    assert math.isnan(result.ranges[0])
    assert result.ranges[1:3]==scan.ranges[1:3]
    assert math.isnan(result.ranges[3]) # 미반환은 free가 아닌 masked/unknown으로 남는다.


@pytest.mark.parametrize('margin',[-.001,float('nan'),float('inf'),.011])
def test_invalid_profile_sway_rejected(margin):
    p=profile();p['lidar']['mount_motion_margin_m']=margin
    with pytest.raises(ValueError):launch_module()._mount_trial(MODEL,p,.0325,.06,[-.1005,0.,.18],.05,True)


def test_sway_reaching_sensor_rejected():
    with pytest.raises(ValueError):bounds([[.002,-.001],[.003,-.001],[.003,.001],[.002,.001]],(0.,0.),.5,.001,.002)
