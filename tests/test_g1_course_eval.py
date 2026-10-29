"""전체 랩 평가의 경계/반례. 제어 성능 검사가 아니다."""
import math
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
from shapely.geometry import box
import g1_traffic_eval as ev
from g1_course_eval import continuous_course_positions, full_course_progress, wheel_center_surface
import validate_g1_traffic as harness

ROOT=Path(__file__).resolve().parents[1]


def circle():
    angle=np.arange(1000)*math.tau/1000
    return ev.Centerline(2*np.cos(angle),2*np.sin(angle),2*angle,4*math.pi)


def poses(line,start,distance,dt=.02):
    return [(i*dt,*line.pose(start+travel)) for i,travel in enumerate(np.arange(0.,distance,.01))]


def test_continuous_reference_survives_half_lap_and_start_crossing():
    line=circle()
    own=poses(line,line.lap-.8,line.lap+1.)
    lead=poses(line,line.lap-.2,line.lap+1.)
    result=full_course_progress(line,own,lead,line.lap-.2,.2)
    assert result['valid'] and result['ego_stays_behind']
    assert result['progress_m']>line.lap
    assert result['ego_end_s_m']>2*line.lap
    assert not full_course_progress(line,lead,own,line.lap-.2,.2)['ego_stays_behind']


def test_projection_jump_missing_pair_and_reversed_time_fail():
    line=circle();own=poses(line,1.,2.);lead=poses(line,1.6,2.)
    broken=list(own);broken[20]=(broken[20][0],*line.pose(7.))
    assert not full_course_progress(line,broken,lead,1.6,.2)['valid']
    assert not full_course_progress(line,own,lead[4:],1.6,.2)['valid']
    assert continuous_course_positions(line,own[::-1],1.6)[1]>0


def test_two_wheel_centers_are_separate_from_full_body_road():
    road=box(-5,-.225,5,.225)
    for y,expected in ((0.,0),(.155,0),(.20,2),(.31,4)):
        row=[(0.,0.,y,0.),(.02,.01,y,0.)]
        result=wheel_center_surface(road,row)
        assert result['valid'] and result['max_off_wheels']==expected
        assert result['three_plus_samples']==(2 if expected>=3 else 0)
    assert ev.rectangle(0.,.155,0.,.2,.15).difference(road).area>0
    assert not wheel_center_surface(None,row)['valid']


def test_full_course_gate_connection_preserves_body_metric_and_rejects_short_run():
    from test_g1_traffic_eval import synthetic_run
    cfg=harness.load_config(ROOT/'config/tests/g1_traffic_v1.json')
    cfg['acceptance']['lap_probe']={**cfg['acceptance']['stopped_center'],'minimum_laps':1}
    case={'name':'lap_probe','behavior_kind':'stopped_center','route_kind':'full_course',
          'duration_s':10.,'schedule':[]}
    # 차체 끝은 도로를 조금 벗어나지만 네 바퀴 중심은 안에 있는 평가 전용 자세열이다.
    result=synthetic_run(3.2,config=cfg,case=case,road=box(-.225,-10,.225,100),
                         ego=lambda rows:[(t,.155,y,yaw) for t,x,y,yaw in rows])
    assert not result['gates']['full_lap']
    assert result['gates']['two_wheel_surface']
    assert result['historical_road_only_gate'] is False
    assert 'inside_road' not in result['gates']
    assert result['gates']['hold'] and result['gates']['ego_stays_behind']
    missing=synthetic_run(3.2,config=cfg,case=case,road=None)
    assert not missing['gates']['two_wheel_surface']


def test_full_course_is_explicit_and_registered_route_stays_inside_track():
    cfg=harness.load_config(ROOT/'config/tests/g1_traffic_full_course_v1.json')
    line,scene=harness.load_centerline();case=cfg['cases'][0]
    start=harness.actor_start_s(cfg,scene,0,case)
    road=harness.road_polygon(harness.DEMO_WORLD/'world.sdf')
    walls=harness.obstacle_polygons(harness.DEMO_WORLD/'world.sdf')
    result=harness.check_route(cfg,line,case,start,road,walls)
    assert result['actor_end_s_m']-start>line.lap
    assert result['actor_road_wall_samples']>=5200
    with pytest.raises(ValueError):harness.check_straight(cfg,line,case,start)
    with pytest.raises(ValueError):harness.check_route(cfg,line,case,start,None,walls)
    with pytest.raises(ValueError,match='wall'):
        harness.check_route(cfg,line,case,start,road,[box(-100,-100,100,100)])
    bad=deepcopy(cfg);bad['acceptance'][case['name']]['maximum_actor_laps']=1.01
    with pytest.raises(ValueError):harness.check_route(bad,line,case,start,road,walls)
    assert 'scripts/g1_course_eval.py' in harness.source_paths(ROOT/'config/tests/g1_traffic_full_course_v1.json')
