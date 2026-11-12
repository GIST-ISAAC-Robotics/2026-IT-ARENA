"""공유 각도 탐색을 기존 두 독립 판정과 대조한다. 관측 조건을 합치지 않는다."""
import math
import numpy as np
import pytest
from arena_autonomy import numeric_kernels as backend
from arena_autonomy.observation_support import depth_support,depth_and_masked_support
from arena_autonomy.local_scene import scene_from_measurements
from test_masked_only_batch import small_scene,reference


@pytest.fixture(autouse=True)
def reset():
    module=pytest.importorskip('arena_autonomy._numeric_kernels')
    assert hasattr(module,'classes'),'rebuild native classification kernel'
    backend.configure('python');yield;backend.configure('python')


def compare(scene,q,padding,cycle=None):
    backend.configure('python')
    with np.errstate(invalid='ignore'):
        free=depth_support(scene,q,padding,cycle=cycle)
        masked=reference(scene,q,padding)
        backend.configure('native_v1')
        actual=depth_and_masked_support(scene,q,padding,cycle=cycle)
    np.testing.assert_array_equal(actual[0],free)
    np.testing.assert_array_equal(actual[1],masked)


def test_default_python_path_is_unchanged():
    assert depth_and_masked_support(None,None,None) is None


@pytest.mark.parametrize('count,sign',[(500,-1),(504,1),(508,-1),(512,1)])
@pytest.mark.parametrize('travel',[0.,.3])
def test_moving_irregular_rays_and_multiple_self_sectors(count,sign,travel):
    rng=np.random.default_rng(count+int(travel*100))
    angle=sign*(-math.pi+np.arange(count)*math.tau/count+rng.uniform(-.001,.001,count))
    ranges=rng.uniform(.1,8.,count);ranges[::23]=np.inf;ranges[::71]=np.nan
    transforms=np.column_stack((np.linspace(-travel,0,count),np.linspace(.03,0,count),np.linspace(-.2,0,count)))
    scene=scene_from_measurements(ranges,angles_rad=angle,acquired_at_s=np.linspace(.8,.9,count),t_ref_s=1.,
        range_min=.05,range_max=12.,acquisition_to_ref_se2=transforms,
        rear_half_angle_deg={'sectors_deg':[[90,6],[-90,5],[170,12]]},preserve_sampling=True)
    origins=np.array([r.ray_origin_xy_m for r in scene.rays])
    q=np.vstack((rng.uniform(-1.,1.,(1400,2)),origins[::10],origins[::10]+[1e-10,0.],
                 origins[::10]+[-1e-9,1e-9]))
    compare(scene,q,rng.uniform(.025,.2,count))


@pytest.mark.parametrize('angle',[0.,1e-10-1e-12,1e-10+1e-12,.0125-1e-12,.0125,.0125+1e-12])
@pytest.mark.parametrize('status',[['masked','return'],['masked','masked'],['unknown','masked'],['return','return']])
def test_boundaries_origin_ties_and_returns(angle,status):
    scene=small_scene([-angle,angle],status)
    q=np.array([[r,0.] for r in (0.,-0.,1e-10,.8,.9,.90000000001,1.,-1.)]+[[np.inf,1.],[np.nan,0.]])
    compare(scene,q,[.1,.1])


@pytest.mark.parametrize('statuses',[['return','return'],['masked','return'],['unknown','masked']])
def test_cyclic_free_and_noncyclic_mask_rules_stay_distinct(statuses):
    scene=small_scene([-.005,.005],statuses,[0,499])
    compare(scene,np.array([[.2,0.]]),[.02,.02],cycle=500)


def test_sentinel_and_invalid_native_input():
    backend.configure('native_v1')
    q=np.array([[.2,0.]]);origins=np.zeros((2,2));directions=np.array([[1.,0.],[1.,0.]])
    indices=np.array([0,1]);status=np.array([2,1]);depth=np.ones(2);padding=np.zeros(2)
    assert backend.support_classes(q,np.array([[2,2]]),origins,directions,indices,status,depth,padding,None)[0]==0
    with pytest.raises(ValueError,match='index outside'):
        backend.support_classes(q,np.array([[-1,1]]),origins,directions,indices,status,depth,padding,None)
