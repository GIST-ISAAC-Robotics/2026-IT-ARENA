"""갈림길 마커 누락 시 관측된 긴 본선 경로의 우선순위와 장애물/만료 경계."""
from dataclasses import replace
import numpy as np
import pytest
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.local_scene import LocalScene, SensorRay
from test_local_follow import history, scene
from test_boundary_bridge import CONFIG, settle


def opened_side(now, side='left', obstacle=False):
    sign = 1 if side == 'left' else -1
    near = np.linspace(-.25, .42, 40)
    far = np.linspace(-.25, 3., 150)
    points = np.vstack((np.column_stack((near, sign*np.full_like(near,.425))),
                        np.column_stack((far, -sign*np.full_like(far,.425)))))
    if obstacle:
        points=np.vstack((points,np.column_stack((np.full(20,.13),np.linspace(-.12,.12,20)))))
    rays=[]
    for i,p in enumerate(points):
        direction=p-np.array([.06,0.]); direction/=np.linalg.norm(direction)
        rays.append(SensorRay((.06,0.),tuple(p),now-.1,'return','laser_frame',0,i,tuple(direction)))
    return LocalScene(now,now-.1,now-.1,'laser_frame',0,tuple(rays))


def make_controller(on=True,side='left'):
    c=LocalFollow(FollowConfig(**CONFIG,boundary_bridge=True,junction_continuity=on))
    h=history()
    for t in (.4,.5,.6,.7,.8):
        c.command(scene(t),h,0.,side)
    return c,h


@pytest.mark.parametrize('side',['left','right'])
def test_current_long_opposite_precedes_short_same_side(side):
    c,h=make_controller(side=side)
    speed,steer,detail=c.command(opened_side(.9,side),h,1.,side)
    assert speed>0 and abs(steer)<.01
    assert detail['path_wall']!=side
    assert detail['observed_path_ahead_m']>1.5
    assert detail['boundary_selection']=='junction_opposite_continuity'


def test_disabling_option_keeps_the_short_path_policy():
    c,h=make_controller(False)
    _,_,detail=c.command(opened_side(.9),h,1.,'left')
    assert detail['path_wall']=='left'
    assert detail['boundary_selection']=='short_current_history_agreement'


def test_retaining_long_wall_does_not_remove_occupied_front():
    c,h=make_controller()
    speed,_,detail=c.command(opened_side(.9,obstacle=True),h,1.,'left')
    assert speed==0 and detail['behavior']=='HOLD'
    assert detail['reason']!='follow_boundary_unconfirmed'


def test_no_current_return_cannot_refresh_path_forever():
    c,h=make_controller()
    current=opened_side(.9)
    empty=LocalScene(1.6,None,None,'laser_frame',0,())
    c.command(current,h,1.,'left')
    assert c.command(empty,h,1.,'left')[0]==0
    c.reset()
    assert c.path is None and c.recent_path is None
