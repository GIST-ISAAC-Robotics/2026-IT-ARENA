"""서로 끊긴 두 관측 면의 분리 및 새로운 면/접근면에 대한 비승계 확인."""
import numpy as np
from arena_autonomy.nearby_face_motion import NearbyFragments
from arena_autonomy.bypass_corridor_trial import CorridorPolicy
from arena_autonomy.local_scene import LocalScene,SensorRay
from arena_autonomy.local_follow import FollowConfig
CFG=FollowConfig(wall_offset_m=.505)
WALL=dict(slope=0.,intercept_m=-.275)
from test_local_follow import history
TIMES=tuple(.5+.1*i for i in range(9))

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
    for t in TIMES:after,e=evaluate(c,t)
    assert e['changed']==36 and len(e['fragments'])==2
    assert after[-1]==.13
    assert set(e['fragments'][0]['source_indices']).isdisjoint(e['fragments'][1]['source_indices'])

def test_approaching_fragment_does_not_borrow_stationary_fragment_state():
    c=NearbyFragments(CorridorPolicy())
    for i,t in enumerate(TIMES):after,e=evaluate(c,t,advance=i*.015)
    assert all(after[:18]<.13) and all(after[18:]==.13)
    assert sorted(f['state']['stable'] for f in e['fragments'])==[False,True]

def test_disappeared_fragment_requires_new_window():
    c=NearbyFragments(CorridorPolicy())
    for t in TIMES:evaluate(c,t)
    evaluate(c,1.4,missing=True)
    after,e=evaluate(c,1.5)
    assert all(after[:18]<.13) and all(after[18:]==.13)

def test_new_epoch_and_repeated_scan_do_not_preserve_stability():
    c=NearbyFragments(CorridorPolicy())
    for _ in range(8):after,e=evaluate(c,.5)
    assert e['changed']==0
    for t in TIMES[1:]:evaluate(c,t)
    after,e=evaluate(c,.4)
    assert e['changed']==0

def test_inactive_bypass_clears_evidence_and_preserves_padding():
    c=NearbyFragments(CorridorPolicy())
    for t in TIMES:evaluate(c,t)
    s=view(1.4);padding=np.full(len(s.rays),.13)
    after,e=c.apply(s,CFG,'left',WALL,history(),padding,active=False)
    np.testing.assert_array_equal(after,padding)
    assert e['changed']==0
    after,e=evaluate(c,1.5)
    assert e['changed']==0

def test_base_padding_and_current_occupied_points_are_not_removed():
    c=NearbyFragments(CorridorPolicy())
    for t in TIMES:evaluate(c,t)
    s=view(1.4);padding=np.full(len(s.rays),.13)
    points=np.asarray(s.occupied_endpoints_xy_m).copy()
    after,e=c.apply(s,CFG,'left',WALL,history(),padding)
    assert np.all(after>=.025+CFG.measurement_padding_m)
    np.testing.assert_array_equal(s.occupied_endpoints_xy_m,points)
    np.testing.assert_array_equal(padding,np.full(len(s.rays),.13))
    assert e['unknown_filled'] is False and e['static_identity_claimed'] is False
