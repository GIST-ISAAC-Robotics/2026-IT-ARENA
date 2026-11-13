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


def test_remembered_opposite_wall_stays_preferred_over_diverging_branch():
    c,h=make_controller()
    c.path_wall='right'
    near=np.linspace(-.25,.25,40)
    branch=np.linspace(.25,1.3,70)
    outer=np.linspace(-.25,3.,160)
    points=np.vstack((np.column_stack((near,np.full_like(near,.425))),
        np.column_stack((branch,.425+branch-.25)),
        np.column_stack((outer,np.full_like(outer,-.425)))))
    rays=[]
    for i,p in enumerate(points):
        d=p-np.array([.06,0.]); d/=np.linalg.norm(d)
        rays.append(SensorRay((.06,0.),tuple(p),.8,'return','laser_frame',0,i,tuple(d)))
    current=LocalScene(.9,.8,.8,'laser_frame',0,tuple(rays))
    speed,steer,detail=c.command(current,h,1.,'left')
    assert speed>0 and abs(steer)<.01
    assert detail['path_wall']=='right'
    assert detail['boundary_selection']=='junction_retained_wall_refresh'


def test_short_near_tangent_agreement_is_not_branch_agreement():
    from arena_autonomy.local_follow import path_agrees, path_agrees_arc
    x=np.linspace(-.2,1.5,100)
    straight=np.column_stack((x,0*x))
    branching=np.column_stack((x,np.maximum(0.,x-.5)))
    # 실제 D2에서 가까운 접선 비교는 통과한 뒤, 그 바깥 분기를 조금씩 따라갔다.
    assert path_agrees(branching,straight,.1)
    assert not path_agrees_arc(branching,straight,.1)


def test_agreeing_full_paths_allow_normal_preferred_side_refresh():
    c,h=make_controller()
    c.path_wall='right'
    speed,_,detail=c.command(scene(.9),h,1.,'left')
    assert speed>0 and detail['path_wall']=='left'
    assert detail.get('boundary_selection')!='junction_retained_wall_refresh'
