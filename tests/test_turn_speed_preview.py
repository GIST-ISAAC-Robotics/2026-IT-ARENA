from dataclasses import replace
import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.turn_preview import preview_speed
from test_local_follow import history, scan, scene, settle


def test_preview_uses_only_beams_crossing_the_requested_path():
    cfg=FollowConfig(open_route_speed_mps=2.,blend_open_speed_by_length=True,
                     occupancy_timing_mode='per_ray_candidate',preview_turn_speed=True)
    recorded=motion_scene(scan(.5,[6.]*500),history(),.6,timing_verified=True,epoch=0)
    path=np.column_stack((np.linspace(-.05,1.7,36),np.zeros(36)))
    def trial(s):
        return preview_speed(s,path,cfg,1.3,0.,0.,.1,1.4,.6,.64,1.5)
    v,label=trial(recorded)
    assert .6<v<=1.4 and label=='observed_turn_clear'
    def unknown(indices):
        return replace(recorded,rays=tuple(replace(r,status='unknown',hit_xy_m=None)
                       if r.source_index in indices else r for r in recorded.rays))
    assert trial(unknown({249,250,251}))==(.6,'current_ray_support_missing')
    assert all(recorded.rays[k].status=='return' for k in (45,46,47))
    assert trial(unknown({45,46,47}))==(v,label)
    front=motion_scene(scan(.5,[.42 if 245<=i<=255 else 6. for i in range(500)]),history(),.6,
                       timing_verified=True,epoch=0)
    assert trial(front)==(.6,'observed_hit_remains')
    # 끊긴 경로나 다음 스캔을 만들어 내지 않는다.
    assert preview_speed(recorded,path,cfg,.8,0.,0.,.1,1.4,.6,.64,.74)==(.6,'observed_path_too_short')


def test_preview_never_releases_the_existing_close_obstacle_hold():
    with pytest.raises(ValueError,match='timed beam'):
        FollowConfig(open_route_speed_mps=2.,blend_open_speed_by_length=True,preview_turn_speed=True)
    cfg=FollowConfig(open_route_speed_mps=2.,blend_open_speed_by_length=True,
                     occupancy_timing_mode='per_ray_candidate',preview_turn_speed=True)
    ctrl=LocalFollow(cfg); h=history()
    assert settle(ctrl,h)[0]>0.
    speed,_,detail=ctrl.command(scene(.8,obstacle_x=.10),h,0.,'left')
    assert speed==0 and detail['behavior']=='HOLD' and 'turn_preview' not in detail
