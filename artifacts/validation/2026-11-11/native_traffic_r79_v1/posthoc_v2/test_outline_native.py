import importlib.util
from pathlib import Path
import numpy as np
import pytest
spec=importlib.util.spec_from_file_location('outline_native',Path(__file__).with_name('outline_native.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def raw():
    return {'representation':'native_measurements_v1','measurements':dict(
        ranges_m=[1.,float('inf'),2.],angles_rad=[3.10,3.08,3.01],acquired_at_s=[2.,2.006,2.019])}

def test_native_keeps_acquisition_order_angles_times_and_missing_returns():
    stamp,ranges,angles,times,lo,hi=m.scan_geometry(raw())
    assert stamp==2. and np.array_equal(angles,[3.10,3.08,3.01])
    assert np.array_equal(times,[2.,2.006,2.019]) and len(ranges)==3
    assert np.isposinf(ranges[1]) and lo==.05 and hi==12.

def test_uniform_geometry_matches_original_equations():
    r=dict(sec=2,nanosec=12345,ranges=[1.,2.,float('inf')],angle_min=-3.14,
        angle_increment=.1,time_increment=.0002,range_min=.05,range_max=12.)
    stamp,ranges,angles,times,lo,hi=m.scan_geometry(r)
    idx=np.arange(3)
    assert stamp==r['sec']+r['nanosec']*1e-9
    assert np.array_equal(angles,r['angle_min']+idx*r['angle_increment'])
    assert np.array_equal(times,stamp+idx*r['time_increment'])

@pytest.mark.parametrize('kind',['length','unordered','nan_angle','unknown_representation'])
def test_invalid_native_is_not_fabricated(kind):
    r=raw()
    if kind=='length':r['measurements']['angles_rad']=[1.,2.]
    if kind=='unordered':r['measurements']['acquired_at_s']=[2.,2.019,2.006]
    if kind=='nan_angle':r['measurements']['angles_rad'][1]=float('nan')
    if kind=='unknown_representation':r['representation']='guess'
    with pytest.raises(ValueError):m.scan_geometry(r)
