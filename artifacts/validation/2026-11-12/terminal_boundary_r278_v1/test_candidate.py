"""반대 벽의 저속 재획득 조건과 기존 점유/관측/확인 순서를 검사한다."""
from dataclasses import replace
import math
import numpy as np
import pytest
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.local_scene import SensorRay, LocalScene
from test_local_follow import history
import candidate


@pytest.fixture(autouse=True)
def installed():
    candidate.install()
    yield
    candidate.uninstall()


def observed(t, *, left=False, short=False, blocked=False, missing=False, same_source=False):
    x=np.linspace(-.25,.35 if short else 2.,140)
    points=[np.column_stack((x,np.full_like(x,-.425)))]
    if left:
        y=np.linspace(-.25,2.,140)
        points.append(np.column_stack((y,np.full_like(y,.425))))
    if blocked:
        points.append(np.column_stack((np.full(30,.20),np.linspace(-.12,.12,30))))
    rays=[];first=.3 if same_source else t-.1
    for i,p in enumerate(np.vstack(points)):
        direction=p-np.array([.06,0.]);direction/=np.linalg.norm(direction)
        rays.append(SensorRay((.06,0.),tuple(p),first,'return','laser_frame',0,i,tuple(direction)))
    if missing:
        rays.append(SensorRay((.06,0.),None,first,'unknown','laser_frame',0,len(rays),(1.,0.)))
    return LocalScene(t,first,first,'laser_frame',0,tuple(rays))


def sequence(*, enabled=True, speed=0., junction=True, **conditions):
    ctl=LocalFollow(FollowConfig(standing_wall_reacquire=enabled,junction_continuity=junction))
    h=history();rows=[]
    for t in (.4,.5,.6,.7):
        rows.append(ctl._baseline_command(observed(t,**conditions),h,speed,'left'))
    return ctl,rows


def test_long_current_opposite_wall_reacquires_after_distinct_observations():
    ctl,rows=sequence()
    assert rows[0][0]==0 and rows[0][2]['single_wall_reacquire']['side']=='right'
    assert rows[-1][0]>0 and ctl.path_wall=='right'
    assert not rows[-1][2]['boundary_from_memory']
    assert rows[-1][2]['unfiltered_occupancy']


def test_original_off_does_not_gain_opposite_wall_permission():
    _,rows=sequence(enabled=False)
    assert all(r[0]==0 and 'single_wall_reacquire' not in r[2] for r in rows)


@pytest.mark.parametrize('speed',[.10001,.2,-.10001,-.2])
def test_moving_measurement_does_not_acquire_new_opposite_wall(speed):
    _,rows=sequence(speed=speed)
    assert all(r[0]==0 and 'single_wall_reacquire' not in r[2] for r in rows)


def test_short_object_is_not_promoted_to_long_wall():
    _,rows=sequence(short=True)
    assert all(r[0]==0 and 'single_wall_reacquire' not in r[2] for r in rows)


def test_front_occupancy_still_blocks_forward_after_path_selection():
    _,rows=sequence(blocked=True)
    assert any('single_wall_reacquire' in r[2] for r in rows)
    assert all(r[0]==0 for r in rows)
    # 정지 조향은 허용되어도 전진은 금지된다. 기존 재개 상태의 세부 이유로 범위를 좁히지 않는다.
    assert rows[-1][2]['reason'] in ('follow_gap_hold','follow_release_wait','follow_steer_in_place')


def test_unknown_front_is_not_filled():
    _,rows=sequence(missing=True)
    assert all(r[0]==0 and r[2]['reason']=='follow_front_unobserved' for r in rows)


def test_preferred_observed_path_remains_preferred():
    ctl,rows=sequence(left=True)
    assert rows[-1][0]>0 and ctl.path_wall=='left'
    assert all('single_wall_reacquire' not in r[2] for r in rows)


def test_same_source_cannot_accumulate_confirmation():
    _,rows=sequence(same_source=True)
    assert all(r[0]==0 for r in rows)


def test_retained_path_does_not_trigger_new_opposite_acquisition():
    ctl,rows=sequence()
    out=ctl._baseline_command(observed(.8),history(),.0,'left')
    assert 'single_wall_reacquire' not in out[2]


def test_without_existing_opposite_refresh_memory_still_expires():
    ctl,rows=sequence(junction=False)
    assert rows[-1][2]['boundary_from_memory']
    out=ctl._baseline_command(observed(.8),history(),.0,'left')
    assert out[0]==0 and 'single_wall_reacquire' in out[2]
