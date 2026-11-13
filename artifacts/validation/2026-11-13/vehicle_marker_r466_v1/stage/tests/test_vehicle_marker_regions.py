"""근사 geometry/실제 pixel 검출/기존 pipeline/점유 제한의 Windows 단위 검증."""
import copy
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from arena_autonomy import vehicle_marker_geometry as g
from arena_autonomy.vehicle_markers import VehicleRegionBuffer
from arena_autonomy.vehicle_marker_occupancy import current_regions,constrain_command
from arena_autonomy.vehicle_marker_calibration import adjust_profile
from arena_autonomy.vision_process import VisionPipeline,recognize
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.local_follow import FollowConfig
from test_vision_process import WorkerDouble,message

STAGE=Path(__file__).resolve().parents[1]


@pytest.fixture
def profile():
    p=json.loads((STAGE/'config/perception/vehicle_marker_approx_b0390.json').read_text(encoding='utf-8'))
    p['targets']=p['targets'][:1]
    p['camera']['fx_px']=p['camera']['fy_px']=800.
    return p


def projected(profile,depth=.65,heading=.15,x=.04):
    # 후방 marker x=차량 우측, y=위쪽, z=뒤쪽; optical y는 아래쪽.
    a=heading;c,s=math.cos(a),math.sin(a)
    rotation=np.array([[c,0.,s],[0.,-1.,0.],[s,0.,-c]])
    half=profile['targets'][0]['black_code_size_m']/2
    obj=np.array([[-half,half,0.],[half,half,0.],[half,-half,0.],[-half,-half,0.]])
    q=cv2.projectPoints(obj,cv2.Rodrigues(rotation)[0],np.array([x,.04,depth]),
        g.camera_matrix(profile['camera'],1280,720),None)[0].reshape(4,2)
    return q


def packet(profile,stamp=1.,epoch=0,depth=.65):
    return g.estimate_regions(np.zeros((720,1280,3),np.uint8),[(1,projected(profile,depth))],profile,
        source_stamp_s=stamp,frame_id='camera_color_optical_frame',epoch=epoch)


@pytest.mark.parametrize('heading,x',[(0.,0.),(.3,.04),(-.3,-.04)])
def test_metric_pose_polygon_and_frame(profile,heading,x):
    region,reason=g.pose_region(projected(profile,.65,heading,x),profile['targets'][0],profile,1280,720)
    assert reason is None
    assert region['rear_center_xy_m']==pytest.approx([.705,-x],abs=1e-7)
    assert region['heading_rad']==pytest.approx(heading,abs=1e-6)
    poly=np.asarray(region['polygon_xy_m'])
    assert np.linalg.norm(poly[1]-poly[0])==pytest.approx(.20)
    assert np.linalg.norm(poly[3]-poly[0])==pytest.approx(.15)
    assert not region['absolute_accuracy_verified']
    assert region['uncertainty']['range_bound_m'] is None


def test_whole_board_size_is_not_black_code_side(profile):
    q=projected(profile)
    correct,_=g.pose_region(q,profile['targets'][0],profile,1280,720)
    wrong=copy.deepcopy(profile['targets'][0]);wrong['black_code_size_m']=.05
    larger,_=g.pose_region(q,wrong,profile,1280,720)
    assert (larger['rear_center_xy_m'][0]-.055)/(correct['rear_center_xy_m'][0]-.055)==pytest.approx(.05/.0348)


@pytest.mark.parametrize('identity',[0,20,30,45,49])
def test_course_and_self_cannot_be_occupancy_targets(profile,identity):
    profile['targets'][0]['id']=identity
    with pytest.raises(ValueError):g.validate_profile(profile)


def test_official_image_profiles_preserve_conflict_and_dictionary():
    p=json.loads((STAGE/'config/perception/vehicle_marker_approx_b0390.json').read_text(encoding='utf-8'))
    g.validate_profile(p)
    assert [(t['dictionary'],t['id']) for t in p['targets']]==[
        ('DICT_4X4_50',1),('DICT_4X4_50',7),('DICT_4X4_50',11),('DICT_7X7_100',91)]
    assert p['targets'][2]['black_code_size_m']==pytest.approx(.05*590/710)
    assert p['targets'][3]['black_code_size_m']==.05


def test_missing_frame_crop_duplicate_no_usable_region(profile):
    rgb=np.zeros((720,1280,3),np.uint8);q=projected(profile)
    args=dict(source_stamp_s=1.,epoch=0,frame_id='camera_color_optical_frame')
    assert g.estimate_regions(rgb,[],profile,**args)['regions']==[]
    assert not g.estimate_regions(rgb,[],profile,**args)['no_detection_is_clear']
    assert not g.estimate_regions(rgb,[(1,q),(1,q+40)],profile,**args)['regions']
    assert g.estimate_regions(rgb,[(1,q)],profile,**dict(args,frame_id='other'))['reason']=='image_frame_mismatch'
    assert g.estimate_regions(rgb[:600],[(1,q)],profile,**args)['reason']=='image_profile_shape_mismatch'


def test_ttl_empty_epoch_and_timestamp_boundary(profile):
    buffer=VehicleRegionBuffer(profile);p=packet(profile)
    assert buffer.update(p,stamp=1.,epoch=0)
    assert buffer.snapshot(1.1,0)['regions']
    for now,epoch in [(.99,0),(1.25,0),(1.1,1)]:assert not buffer.snapshot(now,epoch)['fresh']
    p=packet(profile,1.1);p['regions']=[]
    assert buffer.update(p,stamp=1.1,epoch=0)
    assert not buffer.snapshot(1.11,0)['regions']
    assert not buffer.update(p,stamp=1.2,epoch=0)


def test_pipeline_metric_uses_same_detector_and_rejects_old_epoch(profile,monkeypatch):
    from arena_autonomy import vision_process as v
    calls=[]
    # Match fixture image shape; profile expresses matching resize, not unknown crop.
    profile['camera'].update(width_px=848,height_px=480,fx_px=800.,fy_px=800.)
    q=np.array([[400.,280.],[440.,280.],[440.,320.],[400.,320.]])
    def detect(rgb,threshold):calls.append(1);return [(1,q)]
    monkeypatch.setattr(v,'marker_observations',detect)
    worker=WorkerDouble();pipe=VisionPipeline(worker,vehicle_region_profile=profile)
    msg=message();msg.header.frame_id='camera_color_optical_frame'
    pipe.offer(msg,10.);pipe.pump(1.,10.);worker.finish();assert len(calls)==1
    assert pipe.pump(1.1,10.1) and pipe.vehicle_regions.snapshot(1.1,0)['regions']
    assert not pipe.fault
    pipe.begin_input_boundary(1.15)
    assert not pipe.vehicle_regions.snapshot(1.16,pipe.epoch)['regions']


@pytest.mark.parametrize('dictionary,identity',[('DICT_4X4_50',1),('DICT_7X7_100',91)])
def test_real_pixels_detect_explicit_dictionaries(profile,dictionary,identity):
    profile['targets'][0].update(dictionary=dictionary,id=identity)
    rgb=np.full((720,1280,3),255,np.uint8)
    d=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,dictionary))
    draw=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
    rgb[400:480,600:680]=draw(d,identity,80)[:,:,None]
    from arena_autonomy.core import StartSignal
    request=dict(epoch=0,sequence=1,stamp=1.,signal=vars(StartSignal()),signal_detection_enabled=False,
        image=dict(height=720,width=1280,step=1280*3,encoding='rgb8'),
        image_frame_id='camera_color_optical_frame',vehicle_region_profile=profile)
    result=recognize(request,rgb.tobytes())
    assert [(r['dictionary'],r['id']) for r in result['vehicle_regions']['regions']]==[(dictionary,identity)]


def history(speed=.1):
    cfg=MotionConfig();h=MotionHistory(cfg)
    for t in np.arange(.8,1.31,.05):
        h.add_mean_wheel(round(float(t),6),speed/cfg.wheel_radius_m);h.add_gyro(round(float(t),6),0.)
    return h


def test_occupancy_actual_existing_sweep_and_absence_exact(profile):
    h=history();cfg=FollowConfig();out=(.75,0.,{'reason':'local_follow','behavior':'FOLLOW'})
    p=packet(profile,depth=.25);p['fresh']=True
    regions,_=current_regions(p,h,1.1,0)
    original=np.asarray(p['regions'][0]['polygon_xy_m'])
    assert min(regions[0][1][:,0])==pytest.approx(min(original[:,0])-.01,abs=1e-8)
    limited=constrain_command(out,p,h,1.1,0,0.,cfg)
    assert 0<=limited[0]<out[0] and limited[1]==out[1]
    for other in [None,dict(p,regions=[]),dict(p,source_stamp_s=.5),dict(p,vision_epoch=1),dict(p,purpose='calibration_only')]:
        assert constrain_command(out,other,h,1.1,0,0.,cfg) is out
    side=copy.deepcopy(p)
    side['regions'][0]['polygon_xy_m']=(original+[0.,1.]).tolist()
    assert constrain_command(out,side,h,1.1,0,0.,cfg) is out


def test_simple_focal_one_two_distance_and_lidar_offset(profile):
    samples=[dict(frontoparallel=True,pixel_width=800*.0348/z,black_code_size_m=.0348,
        measured_distance_m=z,image_width_px=1280) for z in (.5,1.)]
    for count in (1,2):
        adjusted=adjust_profile(profile,samples[:count])
        assert adjusted['camera']['fx_px']==pytest.approx(800)
        assert not adjusted['simple_adjustment']['full_intrinsic_calibration']
    sample=dict(samples[0],measured_distance_m=.5-(.0754-.055),distance_basis='lidar_forward_same_plane',
        lidar_base_x_m=.0754,same_frontoparallel_plane=True)
    assert adjust_profile(profile,[sample])['camera']['fx_px']==pytest.approx(800)
    sample['same_frontoparallel_plane']=False
    with pytest.raises(ValueError):adjust_profile(profile,[sample])


def test_range_heading_adjustment_is_explicit_approximate(profile):
    sample=dict(frontoparallel=True,measured_distance_m=1.,estimated_camera_depth_m=.8,
        estimated_heading_rad=.1,reference_heading_rad=0.)
    result=adjust_profile(profile,[sample],'range_scale')
    assert result['range_scale']==1.25 and result['heading_offset_rad']==pytest.approx(-.1)
    assert result['mode']=='approximate_v1' and not result['accuracy_verified']


def test_optional_geometry_error_does_not_abort_existing_vision(profile,monkeypatch):
    import arena_autonomy.vehicle_marker_geometry as geometry
    from arena_autonomy.core import StartSignal
    def error(*a,**k):raise cv2.error('synthetic optional pose error')
    monkeypatch.setattr(geometry,'estimate_regions',error)
    rgb=np.full((720,1280,3),128,np.uint8)
    request=dict(epoch=0,sequence=1,stamp=1.,signal=vars(StartSignal()),signal_detection_enabled=False,
        image=dict(height=720,width=1280,step=1280*3,encoding='rgb8'),
        image_frame_id='camera_color_optical_frame',vehicle_region_profile=profile)
    output=recognize(request,rgb.tobytes())
    assert not output['vehicle_regions']['regions'] and 'signal' in output and 'ids' in output
    assert output['vehicle_regions']['reason']=='optional_geometry_error:error'


def test_offline_image_and_adjustment_cli_files(profile,tmp_path,monkeypatch):
    import runpy,sys
    p=tmp_path/'profile.json';p.write_text(json.dumps(profile),encoding='utf-8')
    rgb=np.full((720,1280,3),255,np.uint8)
    d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
    rgb[400:480,600:680]=draw(d,1,80)[:,:,None]
    image=tmp_path/'synthetic.png';image.write_bytes(cv2.imencode('.png',rgb)[1].tobytes())
    report=tmp_path/'image.json'
    monkeypatch.setattr(sys,'argv',['inspect','--profile',str(p),'--image',str(image),'--output',str(report)])
    runpy.run_path(str(STAGE/'scripts/inspect_vehicle_marker_image.py'),run_name='__main__')
    result=json.loads(report.read_text(encoding='utf-8'))
    assert result['runtime_usable'] is False and result['black_code_pixels'][0]['id']==1
    samples=tmp_path/'samples.json'
    samples.write_text(json.dumps([dict(frontoparallel=True,measured_distance_m=.348,
        pixel_width=80.,black_code_size_m=.0348,image_width_px=1280)]),encoding='utf-8')
    output=tmp_path/'adjusted.json'
    monkeypatch.setattr(sys,'argv',['adjust','--profile',str(p),'--samples',str(samples),'--output',str(output)])
    runpy.run_path(str(STAGE/'scripts/adjust_vehicle_marker_profile.py'),run_name='__main__')
    assert json.loads(output.read_text())['camera']['fx_px']==pytest.approx(800.)
    with pytest.raises(FileExistsError):
        runpy.run_path(str(STAGE/'scripts/adjust_vehicle_marker_profile.py'),run_name='__main__')
