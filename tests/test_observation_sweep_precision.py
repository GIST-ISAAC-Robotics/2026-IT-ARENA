"""직진 극한의 원호·기존 몸체 제외 규칙. 실제 작은 후방 진입은 지우지 않는다."""
import math

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_support import sweep_queries, sweep_support, outside_body
from arena_autonomy.linear_observation import clip_halfplane
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import history, scan
from validate_pipeline_faults_ros import corridor_range


def straight_missing_scene():
    raw = scan(.5)
    raw.angle_min=float(np.float32(raw.angle_min))
    raw.angle_increment=float(np.float32(raw.angle_increment))
    raw.ranges=np.asarray([corridor_range(raw.angle_min+i*raw.angle_increment,.06)
                           for i in range(500)],dtype=np.float32).tolist()
    raw.ranges[375]=math.inf
    return motion_scene(raw,history(),.635,timing_verified=True,rear_half_angle_deg=30.)


@pytest.mark.parametrize('angle', [0.,-9.6137665e-9])
def test_straight_limit_does_not_create_false_rear_queries(angle):
    scene=straight_missing_scene();cfg=FollowConfig(wheelbase_m=.135)
    result=sweep_support([scene],0.,.2,angle,angle,cfg,protected_sensor_pose=(.06,0.,0.))
    assert result['supported'], result
    assert not result['unknown_filled']


def test_tiny_turn_into_actual_side_missing_ray_still_stops():
    # 양의 회전은 실제 90도 결측 광선 쪽으로 새 외곽을 만든다.
    # 뒤쪽 수치 문제와 달리 이 결측을 조향 deadband로 숨기면 안 된다.
    cfg=FollowConfig(wheelbase_m=.135)
    result=sweep_support([straight_missing_scene()],0.,.2,9.6137665e-9,9.6137665e-9,
                         cfg,protected_sensor_pose=(.06,0.,0.))
    assert not result['supported'] and result['reason']=='sweep_unobserved'


@pytest.mark.parametrize('angle', [-1e-7,1e-7,-.001,.001])
def test_true_rear_swing_is_not_ignored_as_numerical_zero(angle):
    result=sweep_support([straight_missing_scene()],0.,.2,angle,angle,
                         FollowConfig(wheelbase_m=.135),protected_sensor_pose=(.06,0.,0.))
    assert not result['supported']


@pytest.mark.parametrize('angle', [-.175,0.,.175])
def test_clipped_queries_apply_same_existing_body_rule_as_border(angle):
    cfg=FollowConfig(wheelbase_m=.135)
    queries,_=sweep_queries(straight_missing_scene(),angle,.273,cfg)
    assert len(queries)
    assert np.all((abs(queries[:,0])>cfg.length_m/2+1e-10)|
                  (abs(queries[:,1])>cfg.width_m/2+1e-10))


def test_axis_rejection_matches_original_clipping_including_tiny_intrusions():
    def original(polygon):
        result=[]
        for origin,direction,sign in (((.1,0.),(0.,1.),-1),((-.1,0.),(0.,1.),1),
                                       ((0.,.075),(1.,0.),1),((0.,-.075),(1.,0.),-1)):
            p=clip_halfplane(polygon,origin,direction,sign)
            area=abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(p,p[1:]+p[:1])))*.5
            if len(p)>=3 and area>1e-14:result.append(p)
        return result
    rng=np.random.default_rng(1017)
    polygons=[[],[(-.1,-.075),(.1,-.075),(.1,.075),(-.1,.075)]]
    for scale in (1e-11,1e-8,1e-4,1.):
        for _ in range(100):
            center=rng.uniform(-.3,.3,2);theta=rng.uniform(-math.pi,math.pi)
            c,s=math.cos(theta),math.sin(theta)
            poly=np.array([[-.1,-.075],[.1,-.075],[.1,.075],[-.1,.075]])
            polygons.append((poly@np.array([[c,s],[-s,c]])*scale+center).tolist())
        for y in (.075-scale,-.075+scale):
            polygons.append([[.05,y],[.06,y],[.06,y+2*scale],[.05,y+2*scale]])
    for polygon in polygons:
        assert outside_body(polygon,.2,.15)==original(polygon)
