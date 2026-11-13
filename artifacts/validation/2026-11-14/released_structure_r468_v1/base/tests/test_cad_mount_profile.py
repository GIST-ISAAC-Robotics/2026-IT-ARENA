"""11/12 CAD 단면·좌표·방향 배선·선택형 거리 한정 자기 반환. 실물/주행 증명 아님."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from arena_autonomy.local_scene import blind_parameter,blind_spec,scene_from_measurements
from arena_autonomy.local_path import mask_scan
from arena_autonomy.native_geometry import native_motion_scene
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.known_structure import validate
from test_low_speed_arc_reacquire import launch_module
from test_jr60_adoption_profile import MODEL

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'src/arena_description/config'


def profile(label='nowire'):
    launch=launch_module()
    return launch._local_sensor_profile(ROOT/'src/arena_description',str(CONFIG/f'b0390_c1_cad_20261112_{label}.yaml'),True)


def mounted(label='nowire',marker_z=.18):
    p=profile(label)
    xml,text=launch_module()._mount_trial(MODEL,p,.0325,.06,[-.1005,0.,marker_z],.05,True)
    return p,ET.fromstring(xml),text


def test_ground_frame_mesh_and_sensor_coordinates_preserve_other_vehicle_values():
    p,model,text=mounted()
    assert p['lidar']['xyz_m']==[.0754,0.,.1346]
    assert p['camera']['xyz_m']==[.055,0.,.198363]
    assert p['camera']['model']=='Arducam_B0390_IMX219'
    old=yaml.safe_load((CONFIG/'b0390_c1_jr60p20_sim.yaml').read_text(encoding='utf8'))
    assert p['imu']==old['imu']
    for k in ('width_px','height_px','rate_hz','horizontal_fov_deg','mass_kg','proxy_size_m'):
        assert p['camera'][k]==old['camera'][k]
    chassis=model.find('model/link');before=ET.fromstring(MODEL).find('model/link')
    for key in ('collision','inertial'):
        assert ET.tostring(chassis.find(key))==ET.tostring(before.find(key))
    visuals=chassis.findall('visual')
    assert len(visuals)==3
    for visual in visuals[1:]:
        assert math.isclose(float(visual.find('pose').text.split()[2]),-.0625)
        uri=visual.find('geometry/mesh/uri').text
        assert Path(uri).is_absolute() and Path(uri).is_file() and '%20' not in uri
    assert len(blind_spec(blind_parameter(0.,text))[1])==4


def test_exact_section_occlusion_and_original_U_not_filled():
    p,_,text=mounted();spec=blind_parameter(0.,text)
    sectors=blind_spec(spec)[1]
    # 보고서 B-rep 0.02도 표본 합11.92와 연속각 범위의 차이만 허용.
    actual=sum(2*(half-.5) for _,half in sectors)
    assert abs(actual-11.92)<.08
    rear=[(c,h-.5) for c,h in sectors if abs(c)>150]
    opening=360-sum(abs(c)+h for c,h in rear)
    assert 20.8<opening<21.2
    for r in p['lidar']['scan_plane_sections']:
        q=np.asarray(r['polygon_xy_m']);assert q.shape==(8,2)
        cross=np.cross(np.roll(q,-1,axis=0)-q,np.roll(q,-2,axis=0)-np.roll(q,-1,axis=0))
        assert np.any(cross>0) and np.any(cross<0) # U 홈/트러스의 오목 경계 보존


@pytest.mark.parametrize('label,axis,thickness',[('frontwire_2mm',0,.002),('frontwire_4mm',0,.004),('frontwire_6mm',0,.006),('inboardwire_4mm',1,.004)])
def test_wiring_only_grows_toward_the_chosen_face(label,axis,thickness):
    p,_,_=mounted(label)
    for row in p['lidar']['scan_plane_sections']:
        q=np.asarray(row['polygon_xy_m'])
        wires=row['wire_polygons_xy_m'];assert wires
        for wire in wires:
            w=np.asarray(wire)
            np.testing.assert_allclose(w[3]-w[0],w[2]-w[1],rtol=0,atol=1e-14)
            if axis==0:
                np.testing.assert_allclose(w[3]-w[0],[thickness,0.],rtol=0,atol=1e-14)
                assert w[:,0].min()>=q[:,0].min()-1e-12
                assert w[:,1].min()>=q[:,1].min()-1e-12 and w[:,1].max()<=q[:,1].max()+1e-12
            else:
                np.testing.assert_allclose(w[3]-w[0],[0.,-np.sign(q[:,1].mean())*thickness],rtol=0,atol=1e-14)
                assert np.max(np.abs(w[:,1]))<=np.max(np.abs(q[:,1]))+1e-12


def test_existing_ID49_height_is_not_silently_hidden():
    _,_,low=mounted(marker_z=.14);_,_,high=mounted(marker_z=.18)
    a=json.loads(low);b=json.loads(high)
    assert len(a['sectors_deg'])==5 and len(b['sectors_deg'])==4
    assert abs(abs(a['sectors_deg'][-1][0])-180)<1e-10
    assert .17<a['range_limits_m'][-1][0]<a['range_limits_m'][-1][1]<.19


def make_scan(ranges,angles):
    return MeasurementScan(tuple(ranges),tuple(angles),tuple(.5+i*.001 for i in range(len(ranges))),time_basis='simulation_acquisition')


def test_near_self_return_removed_but_foreign_return_and_unknown_are_not_free():
    _,_,text=mounted();spec=blind_parameter(0.,text)
    c=math.radians(spec['sectors_deg'][0][0]);lo,hi=spec['range_limits_m'][0]
    scan=make_scan([(lo+hi)/2,lo/2,hi+.1,math.inf,math.nan],[c]*5)
    out=mask_scan(scan,spec)
    assert math.isnan(out.ranges[0]) and out.ranges[1:3]==scan.ranges[1:3]
    assert all(math.isnan(x) for x in out.ranges[3:])
    cfg=SimpleNamespace(lidar_x_m=.0754,lidar_y_m=0.,lidar_yaw_rad=0.)
    poses=np.zeros((5,3));scene=native_motion_scene(scan,poses,.51,cfg,0,spec)
    assert [r.status for r in scene.rays]==['masked','return','return','masked','masked']
    assert len(scene.occupied_endpoints_xy_m)==2
    assert scene.source_first_s==scan.acquired_at_s[0]
    assert scene.source_angles_rad==scan.angles_rad
    assert validate('structure_aware_v1',text)==tuple(map(tuple,spec['sectors_deg']))


@pytest.mark.parametrize('clockwise',[False,True])
def test_native_and_general_motion_paths_have_identical_distance_masks(clockwise):
    _,_,text=mounted('frontwire_4mm');spec=blind_parameter(0.,text)
    angles=np.linspace(-math.pi,math.pi,508,endpoint=False)
    if clockwise:angles=angles[::-1]
    rng=np.random.default_rng(20261112);scan=make_scan(rng.uniform(.05,.3,508),angles)
    pose=rng.normal(0,.02,(508,3));cfg=SimpleNamespace(lidar_x_m=.0754,lidar_y_m=0.,lidar_yaw_rad=.01)
    general=scene_from_measurements(scan.ranges_m,angles_rad=scan.angles_rad,range_min=scan.range_min_m,
        range_max=scan.range_max_m,acquired_at_s=scan.acquired_at_s,t_ref_s=1.1,
        sensor_xy_m=(cfg.lidar_x_m,0.),sensor_yaw_rad=.01,acquisition_to_ref_se2=pose.tolist(),
        rear_half_angle_deg=spec,preserve_sampling=True)
    assert native_motion_scene(scan,pose,1.1,cfg,0,spec)==general


@pytest.mark.parametrize('bands',[[],[[1.,0.]],[[0.,math.inf]],[[True,1.]],[[0.,1.,2.]]])
def test_invalid_ranged_contract_rejected(bands):
    with pytest.raises(ValueError):blind_parameter(0.,json.dumps({'sectors_deg':[[0.,2.]],'range_limits_m':bands}))
