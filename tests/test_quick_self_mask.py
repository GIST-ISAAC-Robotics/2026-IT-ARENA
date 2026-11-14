"""임시 26×21cm 자체 반환 영역의 PC 계약 검증. 실물 보정 증거가 아니다."""
import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import pickle
from types import SimpleNamespace

import numpy as np
import pytest

from arena_autonomy.quick_self_mask import QuickSelfMask, configure_node
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_path import mask_scan
from arena_autonomy.local_scene import scene_from_measurements
from arena_autonomy.native_geometry import native_motion_scene
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_input import project_scan
from arena_autonomy.follow_trace import snapshot_call, restore_call
from arena_autonomy.observation_support import depth_support


ROOT = Path(__file__).resolve().parents[1]
PROFILE = json.loads((ROOT/'config/sensors/quick_self_mask_2026_11_14.json').read_text())
CONFIG = MotionConfig(lidar_x_m=.0754, lidar_y_m=0., rear_axle_x_m=-.064)


def native_points(points, config=CONFIG):
    distances = [math.hypot(x-config.lidar_x_m, y-config.lidar_y_m) for x, y in points]
    angles = [math.atan2(y-config.lidar_y_m, x-config.lidar_x_m)-config.lidar_yaw_rad for x, y in points]
    return MeasurementScan(tuple(distances), tuple(angles), tuple(.5+i*.0002 for i in range(len(points))),
                           time_basis='simulation_acquisition')


def mask(config=CONFIG, **changes):
    return QuickSelfMask(json.dumps(dict(PROFILE, **changes)), config)


def history(config=CONFIG):
    h = MotionHistory(config)
    for t in np.arange(.4, .71, .01):
        h.add_mean_wheel(float(t), 0.)
        h.add_gyro(float(t), 0.)
    return h


def scene(scan, config=CONFIG):
    return native_motion_scene(scan, np.zeros((len(scan.ranges), 3)), .62, config, 0, 0.)


@pytest.mark.parametrize('sensor', [CONFIG, replace(CONFIG, lidar_x_m=-.03, lidar_y_m=.012, lidar_yaw_rad=.73)])
def test_exact_boundaries_and_outside_close_objects_with_rotated_sensor(sensor):
    box = mask(sensor)
    a,b,c,d = box.bounds
    assert [a,b,c,d] == pytest.approx([-.12635,.13365,-.105,.105])
    inside = [(a,0.),(b,0.),(0.,c),(0.,d),(a,c),(b,d)]
    outside = [(a-1e-7,0.),(b+1e-7,0.),(0.,c-1e-7),(0.,d+1e-7)]
    raw = native_points(inside+outside,sensor)
    frozen = pickle.dumps(raw)
    out = box.apply(raw)
    assert pickle.dumps(raw) == frozen and box.last_raw_scan is raw
    assert out.self_masked_indices == tuple(range(len(inside)))
    assert all(math.isnan(v) for v in out.ranges[:len(inside)])
    assert out.ranges[len(inside):] == raw.ranges[len(inside):]
    assert out.acquired_at_s == raw.acquired_at_s and out.angles_rad == raw.angles_rad
    actual = scene(out,sensor)
    assert [r.status for r in actual.rays] == ['masked']*6+['return']*4
    np.testing.assert_allclose(actual.occupied_endpoints_xy_m,outside,atol=1e-14)
    assert box.snapshot()['masked_returns'] == 6


def test_invalid_and_unknown_rays_are_not_upgraded_or_filled():
    raw = MeasurementScan((.08,math.inf,math.nan,-math.inf,.01,13.), (math.pi,)*6,
                          tuple(.5+i*.001 for i in range(6)), time_basis='simulation_acquisition')
    out = mask().apply(raw)
    assert out.self_masked_indices == (0,)
    assert out.ranges[1] == math.inf and out.ranges[3:] == raw.ranges[3:]
    actual = scene(out)
    assert [r.status for r in actual.rays] == ['masked']+['unknown']*5
    assert not actual.occupied_endpoints_xy_m
    assert not depth_support(actual,[[-.3,0.]],np.zeros(6))[0]


def test_native_and_general_scene_and_cad_composition_keep_same_classification():
    raw = native_points([(-.05,0.),(.14,0.),(.3,.2)])
    out = mask_scan(mask().apply(raw), {'sectors_deg':[[150.,1.]],'range_limits_m':[[.1,.2]]})
    poses = np.array([[.1,-.1,.12],[.2,0.,.2],[.3,.1,.3]])
    fast = native_motion_scene(out,poses,.62,CONFIG,0,0.)
    general = scene_from_measurements(out.ranges, angles_rad=out.angles_rad, range_min=out.range_min,
        range_max=out.range_max, acquired_at_s=out.acquired_at_s, t_ref_s=.62,
        sensor_xy_m=(CONFIG.lidar_x_m,CONFIG.lidar_y_m),sensor_yaw_rad=CONFIG.lidar_yaw_rad,
        acquisition_to_ref_se2=poses.tolist(),rear_half_angle_deg=0.,preserve_sampling=True,
        self_masked_indices=out.self_masked_indices)
    assert fast == general and fast.rays[0].status == 'masked'
    assert fast.rays[1].status == 'return'


def test_disabled_is_same_object_and_empty_node_config_is_original_callback():
    raw = native_points([(-.05,0.),(.14,0.)])
    assert mask(enabled=False).apply(raw) is raw
    callback = lambda scan:scan
    node = SimpleNamespace(declare_parameter=lambda *a:SimpleNamespace(value=''))
    assert configure_node(node,callback) is callback


@pytest.mark.parametrize('change', [dict(frame_id='laser_frame'),dict(center_xy_m=[0.]),
    dict(center_xy_m=[True,0.]),dict(body_length_m=0.),dict(body_width_m=-.1),
    dict(margin_each_side_m=-.01),dict(enabled=1),dict(status='calibrated'),
    dict(body_length_m=math.inf),dict(center_xy_m=[math.nan,0.]),dict(extra=1)])
def test_invalid_profile_rejected(change):
    with pytest.raises(ValueError): mask(**change)


def test_frame_mismatch_and_forged_classification_rejected():
    raw = native_points([(-.05,0.),(.14,0.)])
    with pytest.raises(ValueError,match='frame'):
        mask().apply(replace(raw,source_frame_id='other'))
    for indices in ((0,), (2,), (True,), (0,0)):
        with pytest.raises(ValueError): replace(raw,self_masked_indices=indices)


@pytest.mark.parametrize('native', [False,True])
def test_snapshot_ipc_json_restore_keeps_masks_raw_indices_times_and_close_obstacle(native):
    h = history()
    if native:
        raw = native_points([(-.05,0.),(.14,0.),(.3,.2)])
    else:
        raw = SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame',
            stamp=SimpleNamespace(sec=0,nanosec=500000000)),ranges=[.08]*500,
            angle_min=-math.pi,angle_increment=math.tau/500,time_increment=.0002,range_min=.05,range_max=12.)
        raw.ranges[250] = .14-CONFIG.lidar_x_m # 6.46cm external front return outside box
    filtered = mask_scan(mask().apply(raw),0.)
    captured = snapshot_call(filtered,h,.62,0,0.,'left',0.,.1,True,0.)
    decoded = json.loads(json.dumps(captured,allow_nan=False))
    _, restored, restored_history = restore_call(decoded,CONFIG)
    assert tuple(restored.self_masked_indices) == tuple(filtered.self_masked_indices)
    assert motion_scene(restored,restored_history,.62,timing_verified=True,rear_half_angle_deg=0.) == \
           motion_scene(filtered,h,.62,timing_verified=True,rear_half_angle_deg=0.)
    points, ids, _ = project_scan(restored,restored_history,.62,timing_verified=True)
    assert not set(ids).intersection(filtered.self_masked_indices)
    assert (1 if native else 250) in ids
    assert copy.deepcopy(filtered).self_masked_indices == filtered.self_masked_indices


def test_independent_consumers_use_identical_profile_with_own_diagnostics():
    received = [[],[]]
    nodes=[];callbacks=[]
    for i in range(2):
        n = SimpleNamespace(motion=SimpleNamespace(history=history()),
            declare_parameter=lambda *a:SimpleNamespace(value=json.dumps(PROFILE)))
        callbacks.append(configure_node(n,received[i].append));nodes.append(n)
    raw=native_points([(-.05,0.),(.14,0.)])
    for callback in callbacks:callback(raw)
    assert nodes[0].quick_self_mask is not nodes[1].quick_self_mask
    assert received[0][0].self_masked_indices == received[1][0].self_masked_indices == (0,)
    assert nodes[0].quick_self_mask.snapshot() == nodes[1].quick_self_mask.snapshot()


class MaskTransportEvaluator:
    """실제 plan/current IPC 슬롯을 통과한 입력에서 공통 투영 경계를 검사한다."""
    def __init__(self, stage, header):
        self.stage = stage

    def reset(self, reason):
        pass

    def __call__(self, snapshot):
        _, scan, h = restore_call(snapshot, CONFIG)
        result = motion_scene(scan,h,.62,timing_verified=True,rear_half_angle_deg=0.)
        return dict(stage=self.stage,indices=list(scan.self_masked_indices),
                    states=[r.status for r in result.rays],
                    acquired_at_s=list(scan.acquired_at_s),
                    occupied=list(result.occupied_endpoints_xy_m))


@pytest.mark.parametrize('stage',['plan','current'])
def test_real_dual_runtime_ipc_preserves_mask_metadata(stage):
    from arena_autonomy.runtime_dual_process import Lane
    raw = native_points([(-.05,0.),(.14,0.),(.3,.2)])
    out = mask().apply(raw)
    snapshot = snapshot_call(out,history(),.62,0,0.,'left',0.,.1,True,0.)
    lane = Lane(stage,{},capacity=1_000_000,timeout=10.,evaluator_factory=MaskTransportEvaluator)
    try:
        lane.offer(snapshot,1,1)
        assert lane.connection.poll(10.)
        result = lane.collect()['output']
        assert result['stage'] == stage
        assert result['indices'] == [0]
        assert result['states'] == ['masked','return','return']
        assert result['acquired_at_s'] == list(raw.acquired_at_s)
        np.testing.assert_allclose(result['occupied'],[[.14,0.],[.3,.2]],atol=1e-14)
    finally:
        cleanup = lane.close()
    assert not lane.process.is_alive() and lane.process.exitcode == 0
