"""동일 광선의 native 전송 전후 결측 분류·기존 제한 정책이 같아야 한다."""
import json,math
import numpy as np
import pytest
from arena_vehicle_interface.native_lidar_sampling import encode_measurements
from arena_autonomy.native_lidar_input import decode_measurements
from arena_autonomy.scan_input import scan_angles,scan_times
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.rear_fringe_trial import bounded_rear_fringe
from test_rear_fringe_trial import case
from test_local_follow import history

@pytest.mark.parametrize('value',[math.inf,math.nan,-math.inf,.02,-1.])
def test_same_rear_query_before_and_after_native_wire(value):
    expected,raw,_=case(value=value)
    sample=dict(ranges_m=raw.ranges,angles_rad=scan_angles(raw).tolist(),
        acquired_at_s=scan_times(raw).tolist(),source_epoch=0)
    wire=json.dumps(encode_measurements(sample,range_min=raw.range_min,range_max=raw.range_max,
        frame_id=raw.header.frame_id),allow_nan=False)
    decoded=decode_measurements(wire,expected_time_basis='simulation_acquisition')
    scene=motion_scene(decoded,history(),.6,timing_verified=True)
    actual=bounded_rear_fringe(scene,decoded,[(-.047,.0753)],np.full(500,.10),.20,.15)
    assert actual['accepted']==expected['accepted']
    assert actual['reason']==expected['reason']
    assert scene.rays[451].status=='unknown' and scene.rays[451].hit_xy_m is None
    assert actual['unknown_filled'] is False
    if value==math.inf:
        assert decoded.ranges[451]==math.inf
        assert actual['gaps']==expected['gaps']
    elif math.isnan(value):assert math.isnan(decoded.ranges[451])
    else:assert decoded.ranges[451]==value

def test_native_wire_preserves_finite_zero_and_each_nonfinite_kind():
    sample=dict(ranges_m=[1.,0.,math.inf,-math.inf,math.nan],angles_rad=[1.,.9,.8,.7,.6],
        acquired_at_s=[1.,1.02,1.04,1.06,1.08],source_epoch=0)
    encoded=encode_measurements(sample,range_min=.05,range_max=12.,frame_id='laser_frame')
    assert encoded['ranges_m']==[1.,0.,'+inf','-inf',None]
    decoded=decode_measurements(json.dumps(encoded,allow_nan=False),expected_time_basis='simulation_acquisition')
    assert decoded.ranges[:4]==(1.,0.,math.inf,-math.inf) and math.isnan(decoded.ranges[4])
    assert decoded.acquired_at_s==tuple(sample['acquired_at_s'])
    assert decoded.angles_rad==tuple(sample['angles_rad'])

def test_legacy_null_remains_invalid_and_is_not_recovered_as_infinity():
    sample=dict(ranges_m=[math.nan,1.],angles_rad=[1.,.9],acquired_at_s=[1.,1.05],source_epoch=0)
    encoded=encode_measurements(sample,range_min=.05,range_max=12.,frame_id='laser_frame')
    decoded=decode_measurements(json.dumps(encoded,allow_nan=False),expected_time_basis='simulation_acquisition')
    assert math.isnan(decoded.ranges[0])

@pytest.mark.parametrize('bad',['NaN','Infinity','+Infinity',float('inf'),True])
def test_unrecognized_encoding_is_not_accepted(bad):
    sample=dict(ranges_m=[1.,1.],angles_rad=[1.,.9],acquired_at_s=[1.,1.05],source_epoch=0)
    encoded=encode_measurements(sample,range_min=.05,range_max=12.,frame_id='laser_frame')
    encoded['ranges_m'][0]=bad
    with pytest.raises(ValueError):decode_measurements(json.dumps(encoded),expected_time_basis='simulation_acquisition')
