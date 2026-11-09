"""서로 끊긴 두 관측 면의 분리 및 새로운 면/접근면에 대한 비승계 확인."""
import numpy as np
from near_fragments_candidate import NearbyFragments
from arena_autonomy.bypass_corridor_trial import CorridorPolicy
from arena_autonomy.local_scene import LocalScene,SensorRay
from test_near_face_candidate import CFG,WALL
from test_local_follow import history

def view(now,advance=0.,missing=False):
    pieces=[(10,np.linspace(-.25,-.1,18),-.2)]
    if not missing:pieces.append((50,np.linspace(-.04,.12,18),-.2+advance))
    pieces.append((120,np.array([.9]),.12))
    rays=[]
    for base,xs,y in pieces:
        for i,x in enumerate(xs):
            p=(x,y);d=np.asarray(p)-[.06,0.];d/=np.linalg.norm(d)
            rays.append(SensorRay((.06,0.),p,now-.1,'return','laser_frame',0,base+i,tuple(d)))
    return LocalScene(now,now-.1,now-.1,'laser_frame',0,tuple(rays))

def evaluate(c,now,**kw):
    s=view(now,**kw);padding=np.full(len(s.rays),.13)
    return c.apply(s,CFG,'left',WALL,history(),padding)

def test_each_side_of_missing_rays_has_its_own_observed_window():
    c=NearbyFragments(CorridorPolicy())
    for t in (.5,.6,.7,.8,.9):after,e=evaluate(c,t)
    assert e['changed']==36 and len(e['fragments'])==2
    assert after[-1]==.13
    assert set(e['fragments'][0]['source_indices']).isdisjoint(e['fragments'][1]['source_indices'])

def test_approaching_fragment_does_not_borrow_stationary_fragment_state():
    c=NearbyFragments(CorridorPolicy())
    for i,t in enumerate((.5,.6,.7,.8,.9)):after,e=evaluate(c,t,advance=i*.015)
    assert all(after[:18]<.13) and all(after[18:]==.13)
    assert sorted(f['state']['stable'] for f in e['fragments'])==[False,True]

def test_disappeared_fragment_requires_new_window():
    c=NearbyFragments(CorridorPolicy())
    for t in (.5,.6,.7,.8,.9):evaluate(c,t)
    evaluate(c,1.,missing=True)
    after,e=evaluate(c,1.1)
    assert all(after[:18]<.13) and all(after[18:]==.13)

def test_new_epoch_and_repeated_scan_do_not_preserve_stability():
    c=NearbyFragments(CorridorPolicy())
    for _ in range(8):after,e=evaluate(c,.5)
    assert e['changed']==0
    for t in (.6,.7,.8,.9):evaluate(c,t)
    after,e=evaluate(c,.4)
    assert e['changed']==0
