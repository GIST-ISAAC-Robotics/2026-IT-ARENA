"""r227 원 함수 보존본과 묶음 계산의 모든 외곽 좌표를 비교한다."""
import ast
import math
from pathlib import Path
import zipfile
import numpy as np
import pytest
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan, history

ROOT=Path(__file__).resolve().parents[1]
with zipfile.ZipFile(ROOT/'artifacts/validation/2026-11-12/motion_query_batch_connected_r227_v1/before_sources.zip') as archive:
    source=archive.read('src/arena_autonomy/arena_autonomy/pass_release_trial.py').decode()
function=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='motion_queries')
namespace=vars(ps).copy()
exec(compile(ast.Module(body=[function],type_ignores=[]),'r227_original_motion_queries','exec'),namespace)
reference=namespace['motion_queries']


@pytest.mark.parametrize('missing',['none','all','thin','mixed'])
@pytest.mark.parametrize('angles',[(0.,0.),(.21,.21),(-.27,.19),(-1e-9,1e-9),(.4,0.),(0.,-.4)])
@pytest.mark.parametrize('speed,target',[(0.,.2),(-.2,.3),(.9,0.)])
def test_exact_query_set(missing,angles,speed,target):
    rng=np.random.default_rng(226)
    ranges=rng.uniform(.2,6.,500)
    if missing=='all':ranges[:]=math.inf
    elif missing=='thin':ranges[[0,1,249,250,498,499]]=math.inf
    elif missing=='mixed':ranges[rng.choice(500,110,replace=False)]=math.inf
    scene=motion_scene(scan(.5,ranges.tolist()),history(speed,.6),.635,
        timing_verified=True,rear_half_angle_deg=20.)
    cfg=FollowConfig(wheelbase_m=.135)
    args=(scene,speed,target,*angles,.02,cfg,ps.lateral_config(cfg))
    distance,expected=reference(*args)
    other,actual=ps.motion_queries(*args)
    assert distance==other
    np.testing.assert_array_equal(actual,expected)
