"""현재 가림 광선으로 먼 코너 면을 연결한다. 점유나 자유 공간을 생성하지 않는다."""
from dataclasses import replace
from functools import lru_cache
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from arena_autonomy.corner_boundary_support import far_face_occlusion, propose
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, restore_call
from arena_autonomy.scene_motion import motion_scene

@lru_cache(maxsize=1)
def actual():
    p=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/corner_occlusion_connected_r45_v1/first_loss_fixture.json'
    f=json.loads(p.read_bytes());args,scan,hist=restore_call(f['input'],MotionConfig(**f['motion_config']))
    s=motion_scene(scan,hist,f['input']['reference_s'],timing_verified=args['timing_verified'],
                   epoch=args['epoch'],rear_half_angle_deg=args['rear_half_angle_deg'])
    return s,np.asarray(f['reference']),FollowConfig(**f['follow_config']),f['reference_age_s']

def test_observed_far_occlusion_preserves_original_occupancy():
    s,ref,cfg,age=actual();p=np.asarray(s.occupied_endpoints_xy_m).copy();rays=s.rays
    path,e=propose(p,ref,'right',cfg,age,scene=s)
    assert path is not None and e['far_face_start_from_corner_m']>.5
    assert e['far_occlusion_support']['foreground_rays']>=3
    assert not e['free_space_certified'] and not e['driving_authorized'] and e['added_sensor_points']==0
    np.testing.assert_array_equal(p,s.occupied_endpoints_xy_m);assert rays==s.rays

@pytest.mark.parametrize('missing',['rays','foreground','expired','changed_reference','shifted_face'])
def test_current_witness_and_recent_consistent_corner_are_required(missing):
    s,ref,cfg,age=actual();p=np.asarray(s.occupied_endpoints_xy_m).copy()
    if missing=='rays':s=None
    elif missing=='foreground':
        _,e=propose(p,ref,'right',cfg,age,scene=s)
        ids=set(e['far_occlusion_support']['foreground_source_indices'])
        s=replace(s,rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.source_index in ids else r for r in s.rays))
    elif missing=='expired':age=cfg.memory_ttl_s
    elif missing=='changed_reference':ref=ref*[1.,-1.]
    elif missing=='shifted_face':p[p[:,0]>1.,0]+=.35
    assert propose(p,ref,'right',cfg,age,scene=s)[0] is None

def test_reflected_actual_corner_keeps_same_path_and_evidence():
    s,ref,cfg,age=actual();p=np.asarray(s.occupied_endpoints_xy_m)
    path,e=propose(p,ref,'right',cfg,age,scene=s)
    reflect=lambda v:(v[0],-v[1])
    mirrored=replace(s,rays=tuple(replace(r,ray_origin_xy_m=reflect(r.ray_origin_xy_m),direction_xy=reflect(r.direction_xy),
        hit_xy_m=reflect(r.hit_xy_m) if r.hit_xy_m is not None else None) for r in s.rays))
    other,info=propose(p*[1.,-1.],ref*[1.,-1.],'left',cfg,age,scene=mirrored)
    np.testing.assert_allclose(path*[1.,-1.],other,atol=1e-8)
    assert info['far_occlusion_support']['foreground_rays']==e['far_occlusion_support']['foreground_rays']

def scene(kind='block',shift=0.):
    rays=[]
    for i,y in enumerate(np.linspace(.04,.66,50)):
        o=np.array([shift,0.]);end=np.array([1.,y]);v=end-o;v/=np.linalg.norm(v)
        hit=o+.5*(end-o) if kind=='block' else end;status='return'
        if kind in ('unknown','masked'):status=kind;hit=None
        if kind=='far':hit=o+1.4*(end-o)
        rays.append(SimpleNamespace(ray_origin_xy_m=o,direction_xy=v,hit_xy_m=hit,status=status,source_index=i))
    return SimpleNamespace(rays=rays)

@pytest.mark.parametrize('shift',[0.,.06,-.07])
def test_compensated_ray_origins(shift):
    e=far_face_occlusion(scene(shift=shift),[1.,0.],[0.,1.],.7,FollowConfig())
    assert e['accepted'] and e['foreground_rays']==50 and not e['unknown_filled']

@pytest.mark.parametrize('kind',['unknown','masked','far','wall'])
def test_unknown_or_background_is_not_foreground(kind):
    assert not far_face_occlusion(scene(kind),[1.,0.],[0.,1.],.7,FollowConfig())['accepted']

@pytest.mark.parametrize('gap',[.5,1.200001,float('inf'),float('nan')])
def test_extension_has_bounded_extent(gap):
    assert not far_face_occlusion(scene(),[1.,0.],[0.,1.],gap,FollowConfig())['accepted']

def test_observed_background_contradiction_rejects_line():
    s=scene();s.rays[20]=scene('far').rays[20]
    e=far_face_occlusion(s,[1.,0.],[0.,1.],.7,FollowConfig())
    assert not e['accepted'] and e['contradicted_rays']==1
