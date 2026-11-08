"""가림 경로 가설에 필요한 현재 광선 증거. 자유 공간/안전 증명이 아니다."""
from types import SimpleNamespace
import numpy as np
import pytest
from candidate import far_face_occlusion
from arena_autonomy.local_follow import FollowConfig

def scene(kind='block', shift=0.):
    rays=[]
    for i,y in enumerate(np.linspace(.04,.66,50)):
        o=np.array([shift,0.]);end=np.array([1.,y]);v=end-o;v/=np.linalg.norm(v)
        hit=o+.5*(end-o) if kind=='block' else end
        status='return'
        if kind in ('unknown','masked'):status=kind;hit=None
        if kind=='far':hit=o+1.4*(end-o)
        rays.append(SimpleNamespace(ray_origin_xy_m=o,direction_xy=v,hit_xy_m=hit,status=status,source_index=i))
    return SimpleNamespace(rays=rays)

@pytest.mark.parametrize('shift',[0.,.06,-.07])
def test_current_foreground_with_motion_origins(shift):
    s=scene(shift=shift)
    before=np.array([r.hit_xy_m for r in s.rays])
    r=far_face_occlusion(s,[1.,0.],[0.,1.],.7,FollowConfig())
    assert r['accepted'] and r['foreground_rays']==50 and r['foreground_wall_span_m']>.6
    np.testing.assert_array_equal(before,[r.hit_xy_m for r in s.rays])
    assert not r['free_space_certified'] and not r['unknown_filled']

@pytest.mark.parametrize('kind',['unknown','masked','far','wall'])
def test_missing_or_contradictory_foreground_is_not_occlusion(kind):
    assert not far_face_occlusion(scene(kind),[1.,0.],[0.,1.],.7,FollowConfig())['accepted']

@pytest.mark.parametrize('gap',[.5,1.200001,float('inf'),float('nan')])
def test_extension_is_bounded(gap):
    assert not far_face_occlusion(scene(),[1.,0.],[0.,1.],gap,FollowConfig())['accepted']

def test_even_one_observed_background_contradiction_rejects_wall_extension():
    s=scene();s.rays[20]=scene('far').rays[20]
    r=far_face_occlusion(s,[1.,0.],[0.,1.],.7,FollowConfig())
    assert not r['accepted'] and r['contradicted_rays']==1
