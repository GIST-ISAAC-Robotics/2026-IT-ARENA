"""본선 폭 이관: 경로·측방 단면 일관성과 기존 기본 사양 유지."""
import numpy as np
import pytest
from arena_autonomy.local_follow import FollowConfig, path_agrees
from arena_autonomy.local_path import local_path
from arena_autonomy.m3_execution_trial import M3ExecutionFollow
from arena_autonomy.pass_release_trial import lateral_config


def test_old_default_section_and_new_lateral_section_are_consistent():
    old=FollowConfig(occupancy_timing_mode='per_ray_candidate')
    new=FollowConfig(wall_offset_m=.505,road_half_m=.305,occupancy_timing_mode='per_ray_candidate')
    for cfg, expected in ((old,.425),(new,.505)):
        lateral=lateral_config(cfg)
        assert lateral.wall_offset_m == pytest.approx(expected)
        follow=M3ExecutionFollow(cfg,allow_simulation_trial=True)
        assert follow.lateral_config == lateral
        assert cfg.measurement_padding_m == .03 and cfg.boundary_agreement_m == .08
    # 값을 일부만 바꾸면 이전과 마찬가지로 거부한다.
    with pytest.raises(ValueError,match='wall-to-road geometry'):
        M3ExecutionFollow(FollowConfig(wall_offset_m=.505,occupancy_timing_mode='per_ray_candidate'),
                          allow_simulation_trial=True)


def test_observed_parallel_walls_produce_agreeing_center_path_with_new_section():
    x=np.linspace(-.4,2.5,180)
    cloud=np.vstack([np.column_stack([x,np.full_like(x,.505)]),
                     np.column_stack([x,np.full_like(x,-.505)])])
    old=[local_path(cloud,side,.425)[0] for side in ('left','right')]
    new=[local_path(cloud,side,.505)[0] for side in ('left','right')]
    assert not path_agrees(*old,.08)
    assert path_agrees(*new,.08)
    assert np.max(np.abs(new[0][:,1])) < 1e-9


def test_missing_wall_still_does_not_create_a_path_from_section_specification():
    x=np.linspace(-.4,2.5,180)
    one=np.column_stack([x,np.full_like(x,.505)])
    path,info=local_path(one,'right',.505)
    assert path is None and info['reason']=='wall_segment_short'
