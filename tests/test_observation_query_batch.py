import importlib.util
import math
from pathlib import Path
import types
import zipfile

import numpy as np
import pytest
from arena_autonomy.observation_support import sweep_queries
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import history,scan

PATH=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-10-21/g1_query_batch_candidate_v1/candidate.py'
spec=importlib.util.spec_from_file_location('query_batch_candidate',PATH)
candidate=importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)
with zipfile.ZipFile(PATH.parent/'run/source_snapshot.zip') as archive:
    source=archive.read('src/arena_autonomy/arena_autonomy/observation_support.py').decode()
reference=types.ModuleType('before_query_batch_runtime')
exec(compile(source,'before_query_batch_runtime','exec'),reference.__dict__)


@pytest.mark.parametrize('seed',range(6))
@pytest.mark.parametrize('angle',[0.,-9.6137665e-9,9.6137665e-9,-.001,.001,-.175,.175,-.37,.37])
@pytest.mark.parametrize('distance',[.12,.27,.55,.9,2.5])
def test_every_query_and_thin_count_exactly_equal(seed,angle,distance):
    rng=np.random.default_rng(seed)
    ranges=rng.uniform(.2,12.,500)
    ranges[rng.choice(500,seed*12,replace=False)]=math.inf
    scene=motion_scene(scan(.5,ranges.tolist()),history(.15*seed,.4*(seed-2)),.635,
                       timing_verified=True,rear_half_angle_deg=30.)
    cfg=FollowConfig(wheelbase_m=.135)
    expected,thin=reference.sweep_queries(scene,angle,distance,cfg)
    actual,other=candidate.sweep_queries(scene,angle,distance,cfg)
    assert thin==other
    np.testing.assert_array_equal(actual,expected)
    runtime,n=sweep_queries(scene,angle,distance,cfg)
    assert n==thin
    np.testing.assert_array_equal(runtime,expected)


@pytest.mark.parametrize('missing',[False,True])
def test_all_or_no_missing_rays(missing):
    scene=motion_scene(scan(.5,[math.inf if missing else 3.]*500),history(.2,.5),.635,
                       timing_verified=True,rear_half_angle_deg=0.)
    cfg=FollowConfig(wheelbase_m=.135)
    expected,n=reference.sweep_queries(scene,.21,.3,cfg)
    actual,m=candidate.sweep_queries(scene,.21,.3,cfg)
    assert m==n
    np.testing.assert_array_equal(actual,expected)
    runtime,k=sweep_queries(scene,.21,.3,cfg)
    assert k==n
    np.testing.assert_array_equal(runtime,expected)
