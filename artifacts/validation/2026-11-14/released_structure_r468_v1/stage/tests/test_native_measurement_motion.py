"""비균일/CW 원천 입력의 점별 운동 보정과 unknown 셀. 실제 시각 교정 시험은 아니다."""
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.local_scene import scene_from_measurements, scene_from_scan
from arena_autonomy.measurement_scan import MeasurementScan, angular_cell_widths, unknown_cell_directions
from arena_autonomy.observation_support import source_valid, transport_scene, depth_support
from arena_autonomy.scene_motion import measurement_motion_scene, relative_poses


def history(v=1., w=.8):
    h = MotionHistory(MotionConfig(lidar_x_m=.06, wheel_radius_m=.0325))
    for t in np.arange(.9, 1.31, .005):
        h.add_mean_wheel(float(t), v/.0325)
        h.add_gyro(float(t), w)
    return h


def scan(n=508, clockwise=True, **changes):
    direction = -1 if clockwise else 1
    angles = tuple(direction*(i*math.tau/n+.0003*math.sin(i*.9)) for i in range(n))
    stamps = tuple(1.+i*.1/n for i in range(n))
    params = dict(ranges_m=(2.,)*n, angles_rad=angles, acquired_at_s=stamps,
                  time_basis='simulation_acquisition')
    params.update(changes)
    return MeasurementScan(**params)


@pytest.mark.parametrize('n', [412,424,500,508,524,640])
@pytest.mark.parametrize('clockwise', [False,True])
def test_native_motion_preserves_each_source_and_admits_variable_sampling(n, clockwise):
    s=scan(n,clockwise); h=history(); now=1.12
    scene=measurement_motion_scene(s,h,now,timing_verified=True,rear_half_angle_deg=0.)
    assert source_valid(scene)
    assert scene.source_angles_rad == s.angles_rad
    poses=relative_poses(h,s.acquired_at_s,now)
    for i,(ray,(dx,dy,yaw)) in enumerate(zip(scene.rays,poses)):
        # 별도 SE(2) 행렬로 원점/끝점 대조. 배열 정렬이나 점 복제 없음.
        rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        source=np.array([.06+2*math.cos(s.angles_rad[i]),2*math.sin(s.angles_rad[i])])
        assert ray.hit_xy_m == pytest.approx(rotation@source+[dx,dy],abs=1e-12)
        assert ray.acquired_at_s == s.acquired_at_s[i] and ray.source_index == i
    carried=transport_scene(scene,h,1.15)
    assert carried.source_angles_rad == s.angles_rad and source_valid(carried)
    assert tuple(r.acquired_at_s for r in carried.rays) == s.acquired_at_s


@pytest.mark.parametrize('basis,verified', [('unverified_estimate',True),('simulation_acquisition',False)])
def test_estimated_or_unverified_timing_never_bypasses_motion_gate(basis, verified):
    with pytest.raises(MotionUnavailable,match='scan_timing_unverified'):
        measurement_motion_scene(scan(time_basis=basis),history(),1.12,timing_verified=verified)


def test_sensor_and_motion_restarts_cannot_reuse_scene_epoch():
    ids=set()
    for sensor in (0,1,2):
        for motion in (0,1,2):
            result=measurement_motion_scene(scan(source_epoch=sensor),history(),1.12,
                                            timing_verified=True,motion_epoch=motion)
            assert result.source_epoch not in ids
            ids.add(result.source_epoch)


@pytest.mark.parametrize('kind', ['future','stale','frame','gap'])
def test_native_keeps_motion_faults(kind):
    s=scan();h=history();now=1.12
    if kind=='future': now=1.09
    if kind=='stale': now=1.4
    if kind=='frame': s=replace(s,source_frame_id='unknown_frame')
    if kind=='gap': h=MotionHistory()
    with pytest.raises(MotionUnavailable):
        measurement_motion_scene(s,h,now,timing_verified=True)


def test_unknown_cells_use_neighbor_angles_in_clockwise_order():
    angles=(.03,.01,-.02,-.03)
    scene=scene_from_measurements([1.,math.nan,1.,1.],angles_rad=angles,
        acquired_at_s=[1.,1.01,1.02,1.03],t_ref_s=1.04,range_min=.05,range_max=12.,
        rear_half_angle_deg=0.,preserve_sampling=True)
    assert angular_cell_widths(scene,[1])[0] == pytest.approx((.015,.01))
    low,high=unknown_cell_directions(scene,[scene.rays[1]])
    assert math.atan2(low[0][1],low[0][0]) == pytest.approx(-.005)
    assert math.atan2(high[0][1],high[0][0]) == pytest.approx(.02)
    assert scene.rays[1].status=='unknown' and scene.rays[1].hit_xy_m is None


@pytest.mark.parametrize('kind', ['partial','duplicate','reverse','missing_large','times_equal'])
def test_structural_invalid_sampling_does_not_become_recent_support(kind):
    s=scan();a=list(s.angles_rad);t=list(s.acquired_at_s)
    if kind=='partial': a=[v/2 for v in a]
    if kind=='duplicate': a[20]=a[19]
    if kind=='reverse': a[20]=a[19]+.001
    if kind=='missing_large': a[20]=a[19]-.1
    if kind=='times_equal': t[20]=t[19]
    scene=scene_from_measurements(s.ranges_m,angles_rad=a,acquired_at_s=t,t_ref_s=1.12,
        range_min=.05,range_max=12.,preserve_sampling=True)
    assert not source_valid(scene)


def test_source_span_overlap_is_preserved_but_no_cyclic_support_is_invented():
    n=508;s=scan(n,False,angles_rad=tuple(i*math.radians(362)/(n-1) for i in range(n)))
    scene=measurement_motion_scene(s,history(0.,0.),1.12,timing_verified=True,rear_half_angle_deg=0.)
    assert source_valid(scene) and len(scene.rays)==n
    assert scene.source_angles_rad[-1] > math.tau
    # 회전 끝/처음 인접성의 별도 허가를 발급하지 않는다.
    from arena_autonomy.scan_topology import cyclic_count
    assert cyclic_count(scene,None) is None


def test_uniform_legacy_geometry_and_support_remain_identical():
    n=500;step=math.tau/n;a=[-math.pi+i*step for i in range(n)]
    values=[2. if i%17 else math.nan for i in range(n)]
    params=dict(range_min=.05,range_max=12.,acquired_at_s=[1.+i*.0002 for i in range(n)],t_ref_s=1.12)
    old=scene_from_scan(values,angle_min=-math.pi,angle_increment=math.tau/n,**params)
    default=scene_from_measurements(values,angles_rad=a,**params)
    native=scene_from_measurements(values,angles_rad=a,preserve_sampling=True,**params)
    assert old==default and native.rays==old.rays
    queries=np.random.default_rng(7).uniform(-1.,1.,(200,2))
    assert np.array_equal(depth_support(old,queries,np.full(n,.03)),depth_support(native,queries,np.full(n,.03)))


@pytest.mark.parametrize('changes', [dict(acquired_at_s=(1.,1.)),dict(acquired_at_s=(1.,.9)),
    dict(angles_rad=(0.,math.nan)),dict(ranges_m=(1.,True)),dict(time_basis='sdk_verified')])
def test_batch_rejects_malformed_contract(changes):
    params=dict(ranges_m=(1.,2.),angles_rad=(0.,.01),acquired_at_s=(1.,1.01))
    params.update(changes)
    with pytest.raises(ValueError): MeasurementScan(**params)
