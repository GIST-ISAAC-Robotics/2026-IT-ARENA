"""실제 샘플 경계의 코너 소실과 실제 제3면·굴곡을 구분한다."""
import json
import math
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.piecewise_wall import corner_path, _vertices, _short_bevel_vertices
from arena_autonomy.local_path import local_path

ROOT=Path(__file__).resolve().parents[1]
FIX=json.loads((ROOT/'artifacts/validation/2026-11-11/async_corner_r105_v1/fixture.json').read_text('utf8'))


@pytest.mark.parametrize('mirror',[1.,-1.])
def test_resampled_single_corner_keeps_turn_and_all_source_points(mirror):
    samples=np.asarray(FIX['samples'])*[1,mirror];before=samples.copy()
    assert len(_vertices(samples))==4
    path=corner_path(samples,'right' if mirror==1 else 'left',.505,.347)
    assert path is not None
    tangent=np.diff(path,axis=0)
    # 첫 관측 창은 출구 면 중간에서 끝난다. 90도 전체를 외삽하지 않고
    # 실제 창 안에서 이미 시작된 코너만 보존해야 한다.
    assert mirror*math.atan2(tangent[-1,1],tangent[-1,0]) > .5
    assert np.max(mirror*path[:,1]) > .10
    np.testing.assert_array_equal(samples,before)
    # 최소 회전 반경보다 작은 평행 경로를 만들어 해결하지 않는다.
    assert corner_path(samples,'right' if mirror==1 else 'left',.505,.6) is None


def test_whole_current_cloud_returns_corner_not_false_straight_and_off_unchanged():
    cloud=np.asarray(FIX['points']);before=cloud.copy()
    path,info=local_path(cloud,'right',.505,piecewise_corner=True,minimum_radius=.347,narrow_occluder_bridge=True)
    assert info['wall_model']=='piecewise_corner' and path[-1,1]>.7
    assert info['path_anchor_error_m']<.03
    default,old_info=local_path(cloud,'right',.505,narrow_occluder_bridge=True)
    assert 'wall_model' not in old_info and default[-1,1]<.1
    np.testing.assert_array_equal(cloud,before)


def polyline(vertices):
    v=np.asarray(vertices,dtype=float)
    return np.vstack([np.linspace(a,b,max(6,int(np.linalg.norm(b-a)/.01)+1))[:-1]
                      for a,b in zip(v,v[1:])]+[v[-1:]])


@pytest.mark.parametrize('vertices',[
    [[-.3,-.5],[1.,-.5],[1.13,-.37],[1.13,.7]], # 실제 긴 세 번째 면
    [[-.3,-.5],[1.,-.5],[1.03,-.53],[1.03,.7]], # 진행 방향이 반대로 꺾이는 돌출
    [[-.3,-.5],[1.,-.5],[1.,.2],[.5,.2],[.5,.8]], # 두 코너
])
def test_real_third_face_and_reverse_bend_are_not_merged(vertices):
    points=polyline(vertices)
    assert _short_bevel_vertices(points,_vertices(points)) is None
    assert corner_path(points,'right',.505,.347) is None


def test_existing_line_residual_budget_is_not_relaxed():
    samples=np.asarray(FIX['samples']);vertices=_vertices(samples)
    samples[20,1]+=.12
    # 제안 구간이 같더라도 원 점 전체의 3 cm 잔차 조건을 넘으면 거절한다.
    assert _short_bevel_vertices(samples,vertices) is None
