"""동결 원 판정과 비교: 가림/무반환·취득 순서·동률·광선 원점·반환 경계."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import zipfile

import numpy as np
import pytest
from arena_autonomy import numeric_kernels

from arena_autonomy.m3_execution_trial import masked_only
from arena_autonomy.local_scene import SensorRay, scene_from_measurements

ROOT = Path(__file__).resolve().parents[1]
with zipfile.ZipFile(ROOT/'artifacts/validation/2026-11-11/traffic_cpu_r85_v1/before_r85.zip') as archive:
    source = archive.read('src/arena_autonomy/arena_autonomy/m3_execution_trial.py').decode('utf-8')
node = next(n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name == 'masked_only')
scope = dict(np=np, math=math)
exec(compile(ast.Module(body=[node], type_ignores=[]), '<frozen_masked_only>', 'exec'), scope)
reference = scope['masked_only']


@pytest.fixture(autouse=True, params=['python','native_v1'])
def numeric_mode(request):
    if request.param=='native_v1':
        module=pytest.importorskip('arena_autonomy._numeric_kernels')
        assert hasattr(module,'masked'), 'native regression requires rebuilt masked kernel'
    numeric_kernels.configure(request.param)
    yield
    numeric_kernels.configure('python')


@pytest.mark.parametrize('count,sign', [(504,1), (508,-1), (512,1), (500,-1)])
@pytest.mark.parametrize('travel', [0., .3])
def test_native_motion_and_multiple_sectors_match_frozen_rule(count, sign, travel):
    rng = np.random.default_rng(count+int(travel*100))
    angles = sign*(-math.pi+np.arange(count)*math.tau/count+rng.uniform(-.001,.001,count))
    ranges = rng.uniform(.15,5.,count);ranges[::23] = np.inf;ranges[::71] = np.nan
    transforms = np.column_stack((np.linspace(-travel,0,count),np.linspace(.03,0,count),np.linspace(-.2,0,count)))
    scene = scene_from_measurements(ranges, angles_rad=angles,
        acquired_at_s=np.linspace(.8,.9,count),t_ref_s=1.,range_min=.05,range_max=12.,
        acquisition_to_ref_se2=transforms,rear_half_angle_deg={'sectors_deg':[[90,6],[-90,5],[170,12]]},
        preserve_sampling=True)
    pad = rng.uniform(.025,.2,count)
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    q = np.vstack((rng.uniform(-1.,1.,(1400,2)), origins[::10],
                   origins[::10]+[1e-10,0.], origins[::10]+[-1e-9,1e-9]))
    np.testing.assert_array_equal(masked_only(scene,q,pad),reference(scene,q,pad))


def small_scene(angles,statuses,indices=None):
    rays=[]
    for i,(angle,status) in enumerate(zip(angles,statuses)):
        d=(math.cos(angle),math.sin(angle))
        rays.append(SensorRay((0.,0.),d if status=='return' else None,.8+i*.001,
            status,'laser_frame',0,i if indices is None else indices[i],d))
    return SimpleNamespace(rays=tuple(rays))


@pytest.mark.parametrize('angle',[.0125-1e-12,.0125,.0125+1e-12])
@pytest.mark.parametrize('radius',[0.,1e-10,.8,.9,.90000000001,1.])
def test_angle_radius_and_padding_boundaries_preserve_strict_rule(angle,radius):
    scene=small_scene([-angle,angle],['masked','return'])
    q=np.array([[radius,0.],[-radius,0.]])
    np.testing.assert_array_equal(masked_only(scene,q,[.1,.1]),reference(scene,q,[.1,.1]))


@pytest.mark.parametrize('statuses', [ ['masked','masked','return'], ['unknown','masked','return'],
                                      ['return','masked','masked'], ['return','return','return'] ])
def test_duplicate_direction_ties_follow_original_array_order(statuses):
    scene=small_scene([0.,0.,0.],statuses)
    q=np.array([[.2,0.],[0.,0.],[.2,1e-11]])
    np.testing.assert_array_equal(masked_only(scene,q,[.02]*3),reference(scene,q,[.02]*3))


def test_source_index_gap_and_unknown_are_not_relabelled_masked():
    q=np.array([[.2,0.]])
    missing=small_scene([-.005,.005],['masked','unknown'])
    gap=small_scene([-.005,.005],['masked','return'],[0,499])
    for scene in (missing,gap):
        assert not masked_only(scene,q,[.02,.02])[0]
        np.testing.assert_array_equal(masked_only(scene,q,[.02,.02]),reference(scene,q,[.02,.02]))
    assert masked_only(missing,[],[.02,.02]).shape==(0,)


def test_sparse_bearings_and_nonfinite_queries_keep_original_rejection():
    scene=small_scene([1.,2.],['masked','masked'])
    q=np.array([[1.,0.],[np.inf,1.],[np.nan,0.]])
    with np.errstate(invalid='ignore'):
        np.testing.assert_array_equal(masked_only(scene,q,[.02,.02]),reference(scene,q,[.02,.02]))
