from dataclasses import replace
import math
import numpy as np
import pytest

from arena_autonomy.local_scene import scene_from_measurements
from arena_autonomy.lateral_intent import LateralIntentConfig
from arena_autonomy import m3_execution_trial as mx
from probe import QueryReuse


def inputs():
    n=508;angle=math.pi-np.arange(n)*math.tau/n
    ranges=np.full(n,2.);ranges[33:37]=np.inf
    scene=scene_from_measurements(ranges,angles_rad=angle,
        acquired_at_s=np.linspace(.8,.8998,n),t_ref_s=.92,range_min=.05,range_max=12.,
        rear_half_angle_deg={'sectors_deg':[[90,5],[-100,6]]},preserve_sampling=True)
    poses=np.array([[0.,0.,0.],[.1,.01,.06],[.2,.02,.12]])
    return scene,poses,np.full(3,.1),LateralIntentConfig()


def exact(a,b):
    assert a[1]==b[1] and a[0].dtype==b[0].dtype and a[0].shape==b[0].shape
    assert a[0].tobytes()==b[0].tobytes()


def test_return_mutation_cannot_poison_first_or_later_hit():
    args=inputs();original=mx.body_queries(*args)
    with QueryReuse() as reuse:
        first=mx.body_queries(*args);exact(first,original);first[0][:]=999.
        second=mx.body_queries(*args);exact(second,original);second[0][:]=-999.
        exact(mx.body_queries(*args),original)
        assert reuse.hits==2 and reuse.misses==1


@pytest.mark.parametrize('field',['scene','poses','steering','config'])
def test_changed_dependency_never_reuses_result(field):
    scene,poses,steer,cfg=inputs();changed=[scene,poses.copy(),steer.copy(),cfg]
    if field=='scene':changed[0]=replace(scene,t_ref_s=.94)
    elif field=='poses':changed[1][0,1]+=.001
    elif field=='steering':changed[2][0]+=.001
    else:changed[3]=replace(cfg,wheel_radius_m=.031)
    expected=mx.body_queries(*changed)
    with QueryReuse() as reuse:
        mx.body_queries(scene,poses,steer,cfg)
        exact(mx.body_queries(*changed),expected)
        assert reuse.hits==0 and reuse.misses==2


def test_inplace_input_change_is_keyed_by_current_bytes():
    scene,poses,steer,cfg=inputs()
    with QueryReuse() as reuse:
        mx.body_queries(scene,poses,steer,cfg)
        poses[0,1]=.001
        actual=mx.body_queries(scene,poses,steer,cfg)
        exact(actual,reuse.original(scene,poses,steer,cfg))
        assert reuse.hits==0


def test_eight_entry_bound_and_scene_release():
    scene,poses,steer,cfg=inputs()
    with QueryReuse() as reuse:
        for i in range(10):mx.body_queries(scene,poses+i*.001,steer,cfg)
        assert len(reuse.cache)==8
        mx.body_queries(replace(scene,t_ref_s=.93),poses,steer,cfg)
        assert len(reuse.cache)==1
