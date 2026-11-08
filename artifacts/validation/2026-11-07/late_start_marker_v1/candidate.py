"""저장 RGB 분석용 후보. ROS/주행 연결 전."""
import math
import cv2
import numpy as np
from arena_autonomy.race_start_local import LocalOfficialStartSignal
from arena_autonomy.race_start import _components
from arena_autonomy.core import red_signal_mask


def detections(rgb):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters=(cv2.aruco.DetectorParameters() if hasattr(cv2.aruco,'ArucoDetector')
                else cv2.aruco.DetectorParameters_create())
    parameters.minMarkerDistanceRate=.02
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
    if hasattr(cv2.aruco,'ArucoDetector'):
        corners,ids,_=cv2.aruco.ArucoDetector(dictionary,parameters).detectMarkers(gray)
    else:
        corners,ids,_=cv2.aruco.detectMarkers(gray,dictionary,parameters=parameters)
    return [] if ids is None else [(int(i),c.reshape(4,2)) for i,c in zip(ids.flatten(),corners)]


def marker_geometry(markers):
    anchors=[]
    for wanted in (45,0):
        found=[]
        for identity,corners in markers:
            c=np.asarray(corners,dtype=float)
            if identity!=wanted or c.shape!=(4,2) or not np.isfinite(c).all():continue
            size=float(np.mean(np.linalg.norm(c-np.roll(c,1,axis=0),axis=1)))
            if size>=8:found.append((c.mean(axis=0),size))
        if not found:return None
        center,size=max(found,key=lambda f:f[1])
        if any(np.linalg.norm(pos-center)>.4*max(size,s) for pos,s in found):return None
        anchors.append((center,size))
    left,right=anchors;vec=right[0]-left[0];span=float(np.linalg.norm(vec))
    size=(left[1]+right[1])/2
    if (vec[0]<=0 or abs(vec[1])>.4*vec[0] or not 2.5*size<=span<=8*size
            or max(left[1],right[1])>1.8*min(left[1],right[1])):return None
    return dict(origin=left[0].tolist(),axis=(vec/span).tolist(),span=span,size=size)


def late_green(rgb,markers):
    """현재 두 마커 사이의 두 광원을 확인한다. 좌표는 영상에서 얻는다."""
    bank=marker_geometry(markers)
    if bank is None:return None,'late_markers_unavailable'
    origin=np.asarray(bank['origin']);axis=np.asarray(bank['axis']);normal=np.array([-axis[1],axis[0]])
    span=bank['span'];h,w=rgb.shape[:2]
    # 두 마커 사이 패널 영역. 수직 여유는 설치 높이 차를 허용한다.
    vertices=np.array([origin+span*(u*axis+v*normal) for u,v in ((.15,-.23),(.85,-.23),(.85,.12),(.15,.12))])
    if np.any(vertices[:,0]<0) or np.any(vertices[:,0]>=w) or np.any(vertices[:,1]<0) or np.any(vertices[:,1]>=h):
        return None,'late_panel_outside'
    region=np.zeros((h,w),np.uint8);cv2.fillConvexPoly(region,np.rint(vertices).astype(np.int32),255)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    red=red_signal_mask(hsv);red[region==0]=0
    if np.count_nonzero(red)>10:return None,'late_red_present'
    green=cv2.inRange(hsv,(41,131,176),(89,255,255));green[region==0]=0
    blobs,other=_components(green)
    if other or len(blobs)!=2:return None,'late_green_pair_missing'
    uv=[]
    for x,y,diameter in blobs:
        rel=(np.array([x,y])-origin)/span
        u,v=float(rel@axis),float(rel@normal)
        if not (.50<u<.85 and -.20<v<.10 and .025<diameter/span<.12):
            return None,'late_green_location_mismatch'
        uv.append((u,v,diameter/span))
    a,b=uv;diam=(a[2]+b[2])/2
    if abs(a[0]-b[0])>.6*diam or not .8*diam<=abs(a[1]-b[1])<=2.5*diam or max(a[2],b[2])>1.8*min(a[2],b[2]):
        return None,'late_green_pair_mismatch'
    return bank,'late_green_pair'


class MarkerAnchoredStartSignal(LocalOfficialStartSignal):
    def reset(self):
        super().reset();self.late_frames=0;self.late_bank=None

    def update(self,rgb,stamp_s,markers=None):
        previous=self.last_stamp
        result=super().update(rgb,stamp_s)
        if result:return True
        if self.observed in ('invalid_stamp','clock_reset','invalid_image'):
            self.late_frames=0;self.late_bank=None
            return False
        if previous is not None and stamp_s<=previous:return False
        if previous is not None and stamp_s-previous>self.config.max_frame_gap_s:
            self.late_frames=0;self.late_bank=None
        bank,reason=late_green(rgb,detections(rgb) if markers is None else markers)
        if bank is None:
            self.late_frames=0;self.late_bank=None
            if self.red_frames==0:self.observed=reason
            return False
        old=self.late_bank
        consistent=(old is not None and math.dist(bank['origin'],old['origin'])<=.5*bank['size']
                    and abs(bank['span']-old['span'])<=.2*bank['span'])
        self.late_frames=self.late_frames+1 if consistent else 1
        self.late_bank=bank
        self.started=self.late_frames>=self.config.green_confirmations
        self.observed='late_green_confirmed' if self.started else 'late_green_candidate'
        return self.started
