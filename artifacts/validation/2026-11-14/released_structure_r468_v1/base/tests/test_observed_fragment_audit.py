import copy
from dataclasses import asdict
import math
from types import SimpleNamespace

import pytest

from arena_autonomy.follow_trace import snapshot_call
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from audit_observed_fragments import audit


def events():
    config=MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06)
    history=MotionHistory(config)
    for i in range(41):
        history.add_mean_wheel(i*.01,0.)
        history.add_gyro(i*.01,0.)
    scan=SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame',
        stamp=SimpleNamespace(sec=0,nanosec=200000000)),ranges=[1.]*500,
        angle_min=-math.pi,angle_increment=math.tau/500,range_min=.05,range_max=12.,time_increment=.0002)
    data=snapshot_call(scan,history,.32,0,0.,'left',0.,.02,True,30.)
    header=dict(schema=1,scope='g1_decision_stage_only',motion_config=asdict(config))
    return [dict(seq=0,kind='header',data=header),dict(seq=1,kind='call',data={'input':data}),
        dict(seq=2,kind='call',data={'input':copy.deepcopy(data)}),
        dict(seq=3,kind='footer',data=dict(capture_errors=0,failed=0,dropped=0,pending=0,
                                        worker_alive=False,written=3,accepted=3))]


def test_reused_scan_not_new_observation():
    result=audit(events())
    assert result['calls']==2 and result['unique_scans']==result['transformed_scans']==1
    assert result['reused_calls']==1
    assert result['totals']['returns']+result['totals']['masked']==500
    assert result['partition_conservation_passed'] and result['all_unique_scans_projected']
    assert not result['driving_authorized'] and not result['free_area_certified']


def test_same_source_with_modified_scan_rejected():
    data=events()
    data[2]['data']['input']['scan']['ranges'][0]=2.
    with pytest.raises(ValueError,match='same source'): audit(data)


@pytest.mark.parametrize('fault',['gap','missing_footer','dropped','trailing'])
def test_broken_capture_rejected(fault):
    data=events()
    if fault=='gap': data[1]['seq']=99
    elif fault=='missing_footer': data.pop()
    elif fault=='dropped': data[-1]['data']['dropped']=1
    else: data.append(dict(seq=4,kind='reset',data={}))
    with pytest.raises(ValueError): audit(data)


def test_projection_error_is_not_conservation_success():
    data=events()
    for e in data[1:3]: e['data']['input']['timing_verified']=False
    result=audit(data)
    assert not result['all_unique_scans_projected'] and not result['partition_conservation_passed']
    assert result['projection_error_counts']=={'scan_timing_unverified':1}
    assert result['capture_integrity']
