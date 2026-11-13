"""이미 확인된 반대 벽 경로의 단독 상한과 관측/교통 경계를 대조한다."""
from dataclasses import replace
import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from test_corner_distance_follow import isolated, scene
from test_local_follow import history


def run(config, **kwargs):
    controller=LocalFollow(config)
    result=[controller.command(scene(t,**kwargs),history(),0.,'left') for t in (.4,.5,.6,.7)]
    return controller,result


def test_confirmed_alternative_can_use_selected_solo_speed(isolated):
    cfg=FollowConfig(boundary_evidence_mode='corner_distance_candidate',
        max_speed_mps=.2,open_route_speed_mps=1.5,solo_route_only=True)
    _,result=run(cfg)
    assert result[0][0]==result[1][0]==0  # 첫 스캔/해제 대기는 그대로
    speed,_,detail=result[-1]
    assert detail['path_wall']=='right' and detail['wall_fallback']
    assert detail['open_route_speed_enabled'] and .2<speed<=1.5
    assert detail['unfiltered_occupancy']
    _,traffic=run(replace(cfg,solo_route_only=False))
    assert 0<traffic[-1][0]<=.2 and not traffic[-1][2]['open_route_speed_enabled']


@pytest.mark.parametrize('kwargs',[{'inner':False},{'block':True},{'unknown':True}])
def test_missing_support_close_obstacle_and_unknown_do_not_gain_speed(isolated,kwargs):
    cfg=FollowConfig(boundary_evidence_mode='corner_distance_candidate',
        max_speed_mps=.2,open_route_speed_mps=1.5,solo_route_only=True)
    _,result=run(cfg,**kwargs)
    assert all(row[0]==0 for row in result)


def test_stale_scene_cannot_keep_alternative_cruise(isolated):
    cfg=FollowConfig(boundary_evidence_mode='corner_distance_candidate',
        max_speed_mps=.2,open_route_speed_mps=1.5,solo_route_only=True)
    controller,result=run(cfg)
    assert result[-1][0]>.2
    assert controller.command(scene(1.,source=.6),history(),0.,'left')[0]==0
