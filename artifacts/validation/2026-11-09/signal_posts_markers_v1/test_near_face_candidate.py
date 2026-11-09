"""후속 근접 면 후보의 제한 합성 반례. 실제 주행 성공 증명이 아니다."""
import numpy as np
from near_face_candidate import NearbyFace
from arena_autonomy.bypass_corridor_trial import CorridorPolicy
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import LocalScene,SensorRay
from test_local_follow import history

CFG=FollowConfig(wall_offset_m=.505)
WALL=dict(slope=0.,intercept_m=-.275)

def observed(now,y=-.2,car=True):
    points=[(.9,.12)]
    if car:points=[(x,y) for x in np.linspace(-.25,.10,30)]+points
    rays=[]
    for i,p in enumerate(points):
        d=np.asarray(p)-[.06,0.];d/=np.linalg.norm(d)
        rays.append(SensorRay((.06,0.),p,now-.1,'return','laser_frame',0,i,tuple(d)))
    return LocalScene(now,now-.1,now-.1,'laser_frame',0,tuple(rays))

def evaluate(c,now,y=-.2,car=True):
    scene=observed(now,y,car)
    original=np.full(len(scene.rays),.13)
    after,evidence=c.apply(scene,CFG,'left',WALL,history(),original)
    assert np.all(after<=original) and np.all(after>=.025+CFG.measurement_padding_m)
    return after,evidence

def test_current_adjacent_face_gets_own_window_without_reducing_far_post():
    c=NearbyFace(CorridorPolicy())
    for now in (.5,.6,.7,.8):
        after,e=evaluate(c,now)
        assert e['changed']==0
    after,e=evaluate(c,.9)
    assert e['state']['stable'] and e['changed']==30
    assert after[-1]==.13
    assert all(after[:-1]<.13)

def test_same_scan_does_not_manufacture_stability():
    c=NearbyFace(CorridorPolicy())
    for _ in range(8):
        after,e=evaluate(c,.5)
    assert not e['state']['stable'] and e['changed']==0

def test_approaching_adjacent_face_keeps_original_padding():
    c=NearbyFace(CorridorPolicy())
    for i,now in enumerate((.5,.6,.7,.8,.9)):
        after,e=evaluate(c,now,y=-.2+i*.015)
    assert not e['state']['stable'] and e['changed']==0

def test_disappearance_does_not_credit_another_face():
    c=NearbyFace(CorridorPolicy())
    for now in (.5,.6,.7,.8,.9):evaluate(c,now)
    after,e=evaluate(c,1.,car=False)
    assert e['changed']==0 and not e['state']['stable']
    after,e=evaluate(c,1.1,y=-.1)
    assert e['changed']==0 and not e['state']['stable']
