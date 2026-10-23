"""거리 기반 코너 후보의 상태기계/점유 분리와 실제 첫 반환 반례."""
from dataclasses import replace
import math

import numpy as np
import pytest

import arena_autonomy.local_follow as lf
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import LocalScene, SensorRay
from test_corner_distance_probe import reference,wall
from test_corner_raycast import elbow
from test_local_follow import history,cast_scene,corridor,box,branch


def scene(now,inner=True,block=False,unknown=False,source=None,epoch=0):
    path=reference()
    tangent=np.gradient(path,axis=0)
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    outer=path-.425*np.column_stack((-tangent[:,1],tangent[:,0]))
    p=np.vstack((wall() if inner else np.empty((0,2)),outer))
    if block:p=np.vstack((p,np.column_stack((np.full(20,.27),np.linspace(-.1,.1,20)))))
    first=now-.1 if source is None else source
    rays=[]
    for i,point in enumerate(p):
        direction=point-[.06,0.];direction/=np.linalg.norm(direction)
        rays.append(SensorRay((.06,0.),tuple(point),first,'return','laser_frame',epoch,i,tuple(direction)))
    if unknown:rays.append(SensorRay((.06,0.),None,first,'unknown','laser_frame',epoch,len(rays),(1.,0.)))
    return LocalScene(now,first,first,'laser_frame',epoch,tuple(rays))


@pytest.fixture
def isolated(monkeypatch):
    def fitted(points,side,offset):
        if side=='left':return None,{'reason':'wall_segment_short'}
        return reference(),{'reason':'ok','wall_distance_m':.425,'boundary_observed_ahead_m':1.2}
    monkeypatch.setattr(lf,'local_path',fitted)


def test_new_mode_confirms_two_fresh_scans_and_never_filters_occupancy(isolated):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate'))
    result=[ctl.command(scene(t),history(),0.,'left') for t in (.4,.5,.6,.7)]
    assert result[0][0]==result[1][0]==0
    out=result[-1]
    assert out[0]>0,out[2]
    assert out[2]['boundary_selection']=='bilateral_observed_corner_distance'
    assert out[2]['occupied_points']==139 and out[2]['unfiltered_occupancy']
    assert 'source_indices' not in out[2]['corner_distance_evidence']
    assert 'distance_m' not in out[2]['corner_distance_evidence']


@pytest.mark.parametrize('mode',['connected','fragmented_candidate','corner_anchor_candidate'])
def test_other_modes_do_not_silently_gain_new_corner_support(isolated,mode):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode=mode))
    for t in (.4,.5,.6,.7):assert ctl.command(scene(t),history(),0.,'left')[0]==0


@pytest.mark.parametrize('kwargs',[{'inner':False},{'block':True},{'unknown':True}])
def test_missing_anchor_occupied_or_unknown_is_hold(isolated,kwargs):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate'))
    for t in (.4,.5,.6,.7):assert ctl.command(scene(t,**kwargs),history(),0.,'left')[0]==0


def test_reverse_epoch_same_scan_and_ttl_preserved(isolated):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate'));h=history()
    for t in (.4,.42,.44):assert ctl.command(scene(t,source=.3),h,0.,'left')[0]==0
    assert ctl.pending_count==1
    for t in (.5,.6,.7):out=ctl.command(scene(t),h,0.,'left')
    assert out[0]>0
    assert ctl.command(scene(.8),h,-.01,'left')[0]==0
    assert ctl.command(scene(.9),h,0.,'left')[0]==0
    assert ctl.command(scene(1.),h,0.,'left')[0]>0
    assert ctl.command(scene(1.1,epoch=1),h,0.,'left')[0]==0
    for t in (1.2,1.3,1.4):ctl.command(scene(t,epoch=1),h,0.,'left')
    source=ctl.path_source
    for t in (1.5,1.6,1.7,1.8):out=ctl.command(scene(t,epoch=1,inner=False),h,0.,'left')
    assert out[0]==0 and out[2]['reason']=='follow_boundary_unconfirmed'
    assert ctl.path_source==source and ctl.path is None


@pytest.mark.parametrize('kind',['open','center_block','side_block','missing_inner','short_fake_inner'])
@pytest.mark.parametrize('entry',[.05,.15,.30])
@pytest.mark.parametrize('sign',[1,-1])
def test_first_return_entry_cases_and_occlusion(kind,entry,sign):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate'))
    world=(np.asarray(elbow(entry,kind))*[1,sign]).tolist()
    for t in (.4,.5,.6,.7):
        raw=cast_scene(t,world)
        raw=replace(raw,rays=tuple(replace(r,status='masked',hit_xy_m=None)
            if abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))>math.radians(150) else r for r in raw.rays))
        out=ctl.command(raw,history(),0.,'left' if sign==1 else 'right')
    if kind=='open':
        assert out[0]>0,out[2]
        assert out[2]['corner_distance_evidence']['supported_prefix_m'] >= .65
        assert out[2]['occupied_points']==len(raw.occupied_endpoints_xy_m)
    else:assert out[0]==0,out[2]


@pytest.mark.parametrize('geometry',[corridor(),corridor()+box(.28,0.,.20,.15),branch(1.),branch(-1.)])
def test_existing_non_corner_outputs_equal(geometry):
    old=LocalFollow(FollowConfig(boundary_evidence_mode='fragmented_candidate'))
    new=LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate'))
    for t in (.4,.5,.6,.7):
        raw=cast_scene(t,geometry)
        a,b=old.command(raw,history(),0.,'left'),new.command(raw,history(),0.,'left')
        assert a[:2]==b[:2]
        assert a[2]['reason']==b[2]['reason']


def test_unconfirmed_far_tail_is_removed_from_carried_and_command_path(monkeypatch):
    original=reference()
    full=np.vstack((original,np.column_stack((np.full(25,.425),np.linspace(.45,1.05,25)))))
    def fitted(points,side,offset):
        if side=='left':return None,{'reason':'wall_segment_short'}
        return full,{'reason':'ok','wall_distance_m':.425,'boundary_observed_ahead_m':1.8}
    monkeypatch.setattr(lf,'local_path',fitted)
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate'))
    for now in (.4,.5,.6,.7):out=ctl.command(scene(now),history(),0.,'left')
    assert out[0]>0,out[2]
    evidence=out[2]['corner_distance_evidence']
    end=evidence['path_end_exclusive']
    assert end<len(full) and evidence['supported_prefix_m']>=.65
    np.testing.assert_array_equal(ctl.path,full[:end])
    np.testing.assert_array_equal(out[2]['path_points'],full[:end])
    assert evidence['maximum_offset_error_m']>.08
    assert evidence['maximum_supported_offset_error_m']<=.08
    assert out[2]['observed_path_ahead_m']<evidence['reference_ahead_m']


def test_disagreeing_fresh_corner_does_not_replace_valid_memory(monkeypatch,isolated):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate'))
    h=history()
    for t in (.4,.5,.6,.7):ctl.command(scene(t),h,0.,'left')
    old_path,old_stamp=ctl.path.copy(),ctl.path_source
    monkeypatch.setattr(lf,'path_agrees',lambda *args:False)
    out=ctl.command(scene(.8),h,0.,'left')
    assert not out[2]['corner_distance_agrees_memory']
    assert out[2]['corner_distance_evidence']['supported']
    assert not out[2]['fresh_boundary_accepted']
    assert ctl.path_source==old_stamp
    np.testing.assert_array_equal(ctl.path,old_path)
