"""현재 벽에 맞춘 작은 목표선 보정. 점군/자유 공간을 보정하지 않는다."""
import math
import numpy as np
import pytest
from arena_autonomy.bypass_entry_trial import EntryGuide,reanchor_path_to_wall
from probe_bypass_entry import sensor_scene
from test_bypass_entry_guide import stationary_history


def drift(points,angle=.01,normal=.004):
    c,s=math.cos(angle),math.sin(angle)
    return np.asarray(points)@np.array([[c,s],[-s,c]])+normal*np.array([-s,c])


def test_alignment_preserves_shape_and_does_not_estimate_wall_tangent_position():
    x=np.linspace(-.1,1.5,40);path=np.c_[x,.16*(1-np.exp(-x.clip(0)*4))]
    old_wall=np.array([[-.1,0.],[1.5,0.]])
    corrected,detail=reanchor_path_to_wall(drift(path),drift(old_wall),(0.,0.,-.1,1.5))
    np.testing.assert_allclose(corrected,path,atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(np.diff(corrected,axis=0),axis=1),
                               np.linalg.norm(np.diff(path,axis=0),axis=1),atol=1e-12)
    assert detail['applied'] and not detail['along_wall_translation_observed']
    assert not detail['current_scene_transformed']
    # 긴 직선에서 종방향 위치는 알 수 없으므로 임의의 +0.2 m를 지우지 않는다.
    translated=path+[.2,0.]
    untouched,_=reanchor_path_to_wall(translated,old_wall,(0.,0.,-.1,1.5))
    np.testing.assert_allclose(untouched,translated)


@pytest.mark.parametrize('angle,normal',[(math.radians(2.1),0.),(0.,.031)])
def test_large_wall_change_is_not_silently_aligned(angle,normal):
    wall=np.array([[-.1,0.],[1.5,0.]])
    corrected,detail=reanchor_path_to_wall(drift(wall,angle,normal),drift(wall,angle,normal),(0.,0.,-.1,1.5))
    assert corrected is None and not detail['applied']


def started_reanchor():
    history=stationary_history();guide=EntryGuide(stationary_outer_boundary=True,wall_reanchor=True)
    first,_=sensor_scene(targets=((1.2,0.),));guide.evaluate(first,history,0.,0.,.02)
    second,_=sensor_scene(targets=((1.2,0.),),now=.6,first=.5)
    out=guide.evaluate(second,history,0.,0.,.02)
    assert out['selected'] and guide.path is not None
    return guide,history,out


def test_reanchor_is_explicit_and_only_applies_once_per_fresh_scan():
    assert not EntryGuide().wall_reanchor
    with pytest.raises(ValueError):EntryGuide(wall_reanchor=True)
    guide,history,first=started_reanchor()
    original=guide.path.copy();guide.path=drift(guide.path);guide.wall_line=drift(guide.wall_line)
    scene,_=sensor_scene(targets=((1.2,0.),),now=.7,first=.6)
    rays_before=scene.rays
    out=guide.evaluate(scene,history,0.,first['selected']['steering_rad'],.02)
    assert out['path_reanchor']['applied']
    np.testing.assert_allclose(guide.path,original,atol=1e-10)
    assert scene.rays==rays_before and not out['control_authorized']
    again=guide.evaluate(scene,history,0.,first['selected']['steering_rad'],.02)
    assert not again['path_reanchor']['applied']


def test_aligned_target_line_still_checks_current_obstacles():
    guide,history,first=started_reanchor()
    guide.path=drift(guide.path);guide.wall_line=drift(guide.wall_line)
    near,_=sensor_scene(targets=((.18,0.),),now=.7,first=.6)
    blocked=guide.evaluate(near,history,0.,first['selected']['steering_rad'],.02)
    assert blocked['selected'] is None
    assert not blocked['unknown_filled'] and not blocked['control_authorized']
