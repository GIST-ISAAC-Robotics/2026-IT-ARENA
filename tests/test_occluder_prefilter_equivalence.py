"""r87 원 함수와 같은 제거 순서/점 마스크/연결. 계산 후보 선별만 비교한다."""
import ast
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np
import pytest

from arena_autonomy.local_path import break_adjacent_occluders

ROOT=Path(__file__).resolve().parents[1]
A=ROOT/'artifacts/validation/2026-11-11/occluder_cpu_r89_v1'
NAME='src/arena_autonomy/arena_autonomy/local_path.py'
with zipfile.ZipFile(A/'before.zip') as archive:
    source=archive.read(NAME)
assert hashlib.sha256(source).hexdigest()==json.loads((A/'before_manifest.json').read_bytes())[NAME]
node=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='break_adjacent_occluders')
namespace={'np':np}
exec(compile(ast.Module(body=[node],type_ignores=[]),'frozen_r87_occluder','exec'),namespace)
reference=namespace['break_adjacent_occluders']


@pytest.mark.parametrize('seed',range(8))
@pytest.mark.parametrize('max_points',[1,6,17])
@pytest.mark.parametrize('reverse',[False,True])
def test_noisy_walls_multiple_occluders_and_interrupted_indices(seed,max_points,reverse):
    rng=np.random.default_rng(89+seed)
    angles=np.linspace(.15,2.8,220)
    radius=.5/np.sin(angles)+rng.uniform(-.03,.03,len(angles))
    for first in (23,37,45,97,105,166,174):
        radius[first:first+int(rng.integers(1,8))]-=rng.uniform(.07,.18)
    points=radius[:,None]*np.c_[np.cos(angles),np.sin(angles)]
    if seed%2:points[:,1]*=-1
    if reverse:points=points[::-1].copy()
    ranges=np.linalg.norm(points,axis=1)
    broken=np.linalg.norm(np.diff(points,axis=0),axis=1)>.10+.035*np.minimum(ranges[:-1],ranges[1:])
    broken|=rng.random(len(broken))<.03
    original_points,original_breaks=points.copy(),broken.copy()
    expected=reference(points,broken,max_points=max_points)
    actual=break_adjacent_occluders(points,broken,max_points=max_points)
    np.testing.assert_array_equal(actual[0],expected[0])
    assert actual[1]==expected[1]
    np.testing.assert_array_equal(points,original_points)
    np.testing.assert_array_equal(broken,original_breaks)


@pytest.mark.parametrize('depth',[.06-1e-14,.06,.06+1e-14])
@pytest.mark.parametrize('limit',[0,1,6,17])
def test_radial_boundary_and_candidate_order(depth,limit):
    points=np.array([[.4,.5],[.45,.5],[.45,.36],[.47,.52],
                     [.55,.52],[.60,.53],[.60,.38],[.62,.55],[.67,.55]])
    broken=np.ones(len(points)-1,dtype=bool)
    expected=reference(points,broken,max_points=limit,min_depth=depth)
    got=break_adjacent_occluders(points,broken,max_points=limit,min_depth=depth)
    np.testing.assert_array_equal(got[0],expected[0])
    assert got[1]==expected[1]


@pytest.mark.parametrize('n',[0,1,2,3])
def test_short_input(n):
    points=np.zeros((n,2));broken=np.ones(max(0,n-1),dtype=bool)
    expected=reference(points,broken)
    got=break_adjacent_occluders(points,broken)
    np.testing.assert_array_equal(got[0],expected[0]);assert got[1]==expected[1]


@pytest.mark.parametrize('points,broken',[
    ([[1.,2.],[3.,float('nan')]], [True]),
    ([[1.,2.],[3.,4.]], []),
    ([[1.,2.,3.]], []),
])
def test_invalid_input_still_rejected(points,broken):
    with pytest.raises(ValueError):reference(points,broken)
    with pytest.raises(ValueError):break_adjacent_occluders(points,broken)
