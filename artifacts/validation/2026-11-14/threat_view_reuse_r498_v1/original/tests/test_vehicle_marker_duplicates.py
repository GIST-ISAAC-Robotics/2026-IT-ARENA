"""원 OpenCV 4.6 중첩 quad와 배경판/테두리 반례. 인쇄 정확도 검사가 아니다."""
import copy
import cv2
import numpy as np
import pytest

from arena_autonomy import vehicle_marker_geometry as geometry
from arena_autonomy.vehicle_marker_calibration import adjust_profile
from test_vehicle_marker_regions import profile

OUTER=np.array([[600.,400.],[679.,400.],[679.,479.],[600.,479.]])
INNER=np.array([[605.,406.],[673.,405.],[674.,473.],[606.,474.]])


def image(identity=1,frame=False):
    rgb=np.full((720,1280,3),255,np.uint8)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
    rgb[400:480,600:680]=draw(dictionary,identity,80)[:,:,None]
    if frame:cv2.rectangle(rgb,(595,395),(684,484),(0,0,0),2)
    return rgb


def estimate(rgb,quads,profile):
    return geometry.estimate_regions(rgb,[(1,q) for q in quads],profile,source_stamp_s=1.,
        frame_id='camera_color_optical_frame',epoch=0)


@pytest.mark.parametrize('reverse',[False,True])
def test_saved_linux_nested_quad_uses_actual_black_border(profile,reverse):
    quads=[OUTER,INNER]
    if reverse:quads.reverse()
    selected,evidence=geometry.representative_quad(image(),quads,profile['targets'][0])
    np.testing.assert_array_equal(selected,OUTER)
    assert evidence['min_iou']==pytest.approx(4625/6241)
    assert evidence['max_center_fraction']==0.
    assert evidence['selected_black_code_evidence']['supported_edges']==4
    result=estimate(image(),quads,profile)['regions'][0]
    baseline,_=geometry.pose_region(OUTER,profile['targets'][0],profile,1280,720)
    np.testing.assert_array_equal(result['polygon_xy_m'],baseline['polygon_xy_m'])
    assert result['rear_center_xy_m']==baseline['rear_center_xy_m']
    assert not result['absolute_accuracy_verified']


@pytest.mark.parametrize('frame',[False,True])
def test_larger_white_board_or_black_frame_is_not_automatically_selected(profile,frame):
    # OpenCV thickness=2 frame의 실제 바깥 검은 pixel은 594..685다.
    bigger=OUTER.mean(axis=0)+(OUTER-OUTER.mean(axis=0))*((91 if frame else 89)/79)
    rgb=image(frame=frame)
    selected,evidence=geometry.representative_quad(rgb,[bigger,OUTER],profile['targets'][0])
    np.testing.assert_array_equal(selected,OUTER)
    assert evidence['supported_candidates']==1
    wrong,_=geometry.pose_region(bigger,profile['targets'][0],profile,1280,720)
    correct=estimate(rgb,[bigger,OUTER],profile)['regions'][0]
    assert correct['rear_center_xy_m'][0]>wrong['rear_center_xy_m'][0]


@pytest.mark.parametrize('other',[
    OUTER+[40.,0.],
    OUTER.mean(axis=0)+(OUTER-OUTER.mean(axis=0))*.6,
    np.roll(OUTER,1,axis=0),
    OUTER[[0,2,1,3]],
])
def test_separate_scale_orientation_or_invalid_duplicate_not_merged(profile,other):
    result=estimate(image(),[OUTER,other],profile)
    assert not result['regions'] and result['rejected'][0]['reason']=='ambiguous_duplicate'


def test_false_id_on_different_code_or_blank_image_has_no_supported_representative(profile):
    for rgb in (image(identity=7),np.full((720,1280,3),255,np.uint8)):
        assert not estimate(rgb,[OUTER,INNER],profile)['regions']


def test_single_detection_retains_original_pose_without_new_image_gate(profile):
    # Single detection already carries the existing detector's ID; only duplicate resolution changed.
    blank=np.zeros((720,1280,3),np.uint8)
    result=estimate(blank,[OUTER],profile)['regions'][0]
    expected,_=geometry.pose_region(OUTER,profile['targets'][0],profile,1280,720)
    assert result['rear_center_xy_m']==expected['rear_center_xy_m']


@pytest.mark.parametrize('distance',[.2,.5])
def test_existing_one_frontal_sample_accepts_short_box_range(profile,distance):
    sample=dict(frontoparallel=True,measured_distance_m=distance,
        pixel_width=800*.0348/distance,black_code_size_m=.0348,image_width_px=1280)
    result=adjust_profile(profile,[sample])
    assert result['camera']['fx_px']==pytest.approx(800.)
    assert result['simple_adjustment']['absolute_accuracy_verified'] is False
