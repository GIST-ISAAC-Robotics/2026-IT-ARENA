"""영상 앵커 늦은 출발 후보의 대표 기능/오출발/상태 회귀."""
import cv2
import numpy as np
import pytest
from arena_autonomy.race_start_marker import MarkerAnchoredStartSignal, late_green
from arena_autonomy.core import marker_observations

def detections(rgb):
    return marker_observations(rgb,.02)


def frame(kind='green'):
    image=np.full((300,650,3),160,np.uint8)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for identity,cx in ((45,180),(0,480)):
        code=(cv2.aruco.generateImageMarker(dictionary,identity,70) if hasattr(cv2.aruco,'generateImageMarker')
              else cv2.aruco.drawMarker(dictionary,identity,70))
        image[90:190,cx-50:cx+50]=255
        image[105:175,cx-35:cx+35]=code[:,:,None]
    cv2.rectangle(image,(235,110),(420,165),(35,35,35),-1)
    for col in range(5):
        for y in (128,148):
            color=(0,255,0) if col==4 and kind in ('green','both') else ((255,0,0) if col<4 and kind in ('red','both') else (0,0,0))
            if col==4 and kind=='unlit':color=(0,110,0)
            cv2.circle(image,(255+36*col,y),6,color,-1)
    if kind=='single':cv2.circle(image,(399,128),6,(0,255,0),-1)
    if kind=='shirt':cv2.rectangle(image,(382,113),(412,160),(0,255,0),-1)
    return image


def observe(image,n=4):
    s=MarkerAnchoredStartSignal()
    return s,[s.update(image,1.+i*.1) for i in range(n)]


def test_current_two_markers_and_two_green_lamps_start_without_red_history():
    s,values=observe(frame())
    assert values==[False,False,True,True] and s.observed=='late_green_confirmed'
    assert s.update(frame('off'),2.)


@pytest.mark.parametrize('kind',['red','both','off','unlit','single','shirt'])
def test_not_enough_positive_green_evidence(kind):
    assert not any(observe(frame(kind))[1])


def test_background_people_colours_outside_panel_do_not_change_the_decision():
    image=frame()
    cv2.rectangle(image,(20,195),(500,299),(0,255,0),-1)
    cv2.rectangle(image,(510,20),(630,100),(255,0,0),-1)
    assert observe(image)[1]==[False,False,True,True]


@pytest.mark.parametrize('kind',['missing','one','swapped','separate_duplicate'])
def test_missing_or_ambiguous_marker_pair_cannot_anchor(kind):
    rgb=frame();m=detections(rgb)
    if kind=='missing':m=[]
    if kind=='one':m=[a for a in m if a[0]==0]
    if kind=='swapped':m=[(45 if i==0 else 0,c) for i,c in m]
    if kind=='separate_duplicate':m=m+[(45,next(c for i,c in m if i==45)+[0,90])]
    s=MarkerAnchoredStartSignal()
    assert not any(s.update(rgb,1.+i*.1,m) for i in range(5))


def test_nested_same_marker_detection_is_one_physical_anchor():
    rgb=frame();m=detections(rgb);c=next(c for i,c in m if i==45)
    m.append((45,(c-c.mean(axis=0))*.9+c.mean(axis=0)))
    s=MarkerAnchoredStartSignal()
    assert [s.update(rgb,1.+i*.1,m) for i in range(3)]==[False,False,True]


@pytest.mark.parametrize('interruption',['gap','invalid_image','missing_marker','clock_reset','invalid_time'])
def test_confirmation_restarts_after_interruption(interruption):
    rgb=frame();s=MarkerAnchoredStartSignal()
    assert not s.update(rgb,1.)
    t=1.2
    if interruption=='gap':t=1.5
    if interruption=='invalid_image':assert not s.update(np.zeros((5,5)),1.1)
    if interruption=='missing_marker':assert not s.update(rgb,1.1,[])
    if interruption=='clock_reset':assert not s.update(rgb,.5)
    if interruption=='invalid_time':assert not s.update(rgb,float('nan'))
    assert not s.update(rgb,t)
    assert not s.update(rgb,t+.1)
    assert s.update(rgb,t+.2)


def test_duplicate_frame_does_not_accumulate_confirmation():
    rgb=frame();s=MarkerAnchoredStartSignal()
    for _ in range(8):assert not s.update(rgb,1.)
    assert not s.update(rgb,1.1)
    assert s.update(rgb,1.2)


def test_old_red_green_path_remains_available_without_markers():
    from audit_start_signal_layout import light_image
    s=MarkerAnchoredStartSignal()
    assert not s.update(light_image(4,paired=True),1.,[])
    assert not s.update(light_image(4,paired=True),1.1,[])
    assert not s.update(light_image(0,True,paired=True),1.2,[])
    assert not s.update(light_image(0,True,paired=True),1.3,[])
    assert s.update(light_image(0,True,paired=True),1.4,[])
    assert s.observed=='green_confirmed'


@pytest.mark.parametrize('scale,angle',[(.65,0.),(1.4,0.),(1.,15.),(1.,-15.)])
def test_image_scale_and_moderate_roll_are_measured_from_markers(scale,angle):
    image=frame();image=cv2.resize(image,None,fx=scale,fy=scale,interpolation=cv2.INTER_LINEAR)
    h,w=image.shape[:2]
    transform=cv2.getRotationMatrix2D((w/2,h/2),angle,1.)
    image=cv2.warpAffine(image,transform,(w,h),borderValue=(160,160,160))
    assert observe(image)[1]==[False,False,True,True]
