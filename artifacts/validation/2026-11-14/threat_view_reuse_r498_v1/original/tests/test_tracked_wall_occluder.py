"""우회 기준선의 좁은 가림체 연결은 점유나 미반환 광선을 제거하지 않는다."""
from dataclasses import replace
import math
import numpy as np
import pytest
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import SensorRay,LocalScene
from arena_autonomy.bypass_wall_fit import tracked_straight_wall
from arena_autonomy.bypass_entry_trial import current_outer_wall_padding

def sample(kind='narrow'):
    a,b=-.2,-.3
    origin=np.array([.06,0.]);center=np.array([.315,-.23]);radius=.07 if kind=='wide' else .01
    if kind=='near':center=np.array([.06,-.20]);radius=.015
    rays=[];post_ids=[]
    for i in range(500):
        theta=-math.pi+i*math.tau/500;d=np.array([math.cos(theta),math.sin(theta)])
        denominator=d[1]-a*d[0]
        distance=(a*origin[0]+b)/denominator if abs(denominator)>1e-12 else math.inf
        if distance<=0 or distance>12:distance=math.inf
        toward=center-origin;projection=float(toward@d)
        root=radius**2-float(toward@toward)+projection**2
        pole=projection-math.sqrt(root) if root>=0 else math.inf
        if 0<pole<distance:distance=pole;post_ids.append(i)
        p=tuple(origin+distance*d) if math.isfinite(distance) else None
        rays.append(SensorRay(tuple(origin),p,.5+i*.0002,
            'return' if p is not None else 'unknown','laser_frame',0,i,tuple(d)))
    if kind=='unknown':
        mid=post_ids[len(post_ids)//2]
        rays[mid]=replace(rays[mid],status='unknown',hit_xy_m=None)
    cfg=replace(FollowConfig(),wall_offset_m=.505,narrow_occluder_bridge=True)
    x=np.array([-.3,1.8]);line=np.c_[x,a*x+b+cfg.wall_offset_m*np.hypot(1,a)]
    return LocalScene(.6,.5,.5998,'laser_frame',0,tuple(rays)),cfg,line,post_ids

def test_current_wall_extends_across_observed_small_occluder_only_when_selected():
    scene,cfg,line,post_ids=sample();original=scene.rays
    old,_=tracked_straight_wall(scene,replace(cfg,narrow_occluder_bridge=False),'right',line)
    fit,wall=tracked_straight_wall(scene,cfg,'right',line)
    assert old[3]<.8 and fit[3]>1.8
    assert wall['narrow_occluders_bridged']==1
    assert not set(post_ids).intersection(wall['source_ray_indices'])
    assert scene.rays==original and wall['all_returns_remain_occupied']
    assert wall['wall_reference_only_not_free_space']
    padding,_=current_outer_wall_padding(scene,cfg,'right',wall)
    ids=[r.source_index for r in scene.rays if r.status=='return']
    assert all(padding[ids.index(i)]>.025+cfg.measurement_padding_m for i in post_ids)

@pytest.mark.parametrize('kind',['unknown','wide'])
def test_unknown_gap_or_broad_obstacle_does_not_join_wall_reference(kind):
    scene,cfg,line,_=sample(kind)
    fit,wall=tracked_straight_wall(scene,cfg,'right',line)
    assert fit[3]<.8 and 'narrow_occluders_bridged' not in wall

def test_no_far_wall_returns_means_no_extension():
    scene,cfg,line,post_ids=sample()
    scene=replace(scene,rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.status=='return' and r.hit_xy_m[0]>.35 else r for r in scene.rays))
    fit,wall=tracked_straight_wall(scene,cfg,'right',line)
    assert fit[3]<.8 and 'narrow_occluders_bridged' not in wall


def test_nearby_small_post_remains_a_blocker_when_more_rays_hit_it():
    scene,cfg,line,post_ids=sample('near');before=scene.rays
    assert len(post_ids)>6
    fit,wall=tracked_straight_wall(scene,cfg,'right',line)
    assert wall.get('narrow_occluders_bridged')==1 and fit[3]>1.5
    assert scene.rays==before and not set(post_ids).intersection(wall['source_ray_indices'])
    padding,_=current_outer_wall_padding(scene,cfg,'right',wall)
    ids=[r.source_index for r in scene.rays if r.status=='return']
    assert all(padding[ids.index(i)]>.025+cfg.measurement_padding_m for i in post_ids)
