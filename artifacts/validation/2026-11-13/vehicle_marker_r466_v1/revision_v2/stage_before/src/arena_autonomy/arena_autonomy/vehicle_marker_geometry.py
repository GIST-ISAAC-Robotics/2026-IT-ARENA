"""후방 마커의 근사 평면 차량 영역. LiDAR 반환/자유 공간/차량 정답이 아니다."""
import math

import cv2
import numpy as np

COURSE_IDS = frozenset((0, 20, 30, 45))
DICTIONARIES = ('DICT_4X4_50', 'DICT_7X7_100')
WIDTH_M, LENGTH_M = .15, .20


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name+' must be finite numeric')
    return float(value)


def validate_profile(profile):
    if profile.get('schema') != 1 or profile.get('mode') != 'approximate_v1':
        raise ValueError('explicit approximate marker profile required')
    camera = profile['camera']
    if not isinstance(profile.get('profile_id'), str) or not profile['profile_id']:
        raise ValueError('explicit profile identity required')
    for key in ('width_px', 'height_px'):
        if type(camera[key]) is not int or camera[key] <= 0:
            raise ValueError('positive camera image dimensions required')
    if not camera.get('optical_frame_id'):
        raise ValueError('explicit image optical frame required')
    if 'fx_px' in camera:
        if min(finite(camera['fx_px'],'fx'), finite(camera.get('fy_px',camera['fx_px']),'fy')) <= 0:
            raise ValueError('positive focal pixels required')
    elif not 1 < finite(camera['horizontal_fov_deg'],'FOV') < 179:
        raise ValueError('finite horizontal FOV required')
    for key in ('base_xyz_m','base_rpy_rad'):
        if len(camera[key]) != 3 or not all(math.isfinite(finite(v,key)) for v in camera[key]):
            raise ValueError('camera base transform needs three values')
    if len(camera.get('distortion',[])) not in (0,4,5,8,12,14):
        raise ValueError('unsupported OpenCV distortion length')
    for v in camera.get('distortion',[]):finite(v,'distortion')
    for key in ('cx_px','cy_px'):
        if key in camera:finite(camera[key],key)
    for key,default in (('max_range_m',5.),('max_reprojection_error_px',4.)):
        if finite(profile.get(key,default),key)<=0:raise ValueError(key+' must be positive')
    if finite(profile.get('range_scale',1.),'range scale') <= 0:
        raise ValueError('positive range scale required')
    finite(profile.get('heading_offset_rad',0.),'heading offset')
    if not 0 < finite(profile.get('max_age_s',.25),'max age') <= .5:
        raise ValueError('short explicit marker observation lifetime required')
    own = profile.get('own_marker',{'dictionary':'DICT_4X4_50','id':49})
    seen=set()
    if profile.get('purpose','occupancy_assist') not in ('occupancy_assist','calibration_only'):
        raise ValueError('unknown marker profile purpose')
    for item in profile['targets']:
        d,i=item['dictionary'],item['id']
        if d not in DICTIONARIES or type(i) is not int or not 0 <= i < (50 if d==DICTIONARIES[0] else 100):
            raise ValueError('explicit supported dictionary/ID required')
        if d==DICTIONARIES[0] and i in COURSE_IDS:
            raise ValueError('course marker cannot be a vehicle')
        if (d,i) in seen:raise ValueError('duplicate vehicle marker profile')
        seen.add((d,i))
        if (d,i)==(own['dictionary'],own['id']) and profile.get('purpose')!='calibration_only':
            raise ValueError('own marker requires calibration-only profile')
        black=finite(item['black_code_size_m'],'black code size')
        board=finite(item['printed_image_size_m'],'printed image size')
        if not 0 < black <= board or not item.get('size_basis'):
            raise ValueError('black code and complete printed image sizes must be explicit')
    return profile


def camera_matrix(camera, width, height):
    sx,sy=width/camera['width_px'],height/camera['height_px']
    if abs(sx-sy)>1e-6:
        raise ValueError('unknown crop/aspect change; explicit matching profile required')
    fx=camera.get('fx_px')
    if fx is None:fx=camera['width_px']/(2*math.tan(math.radians(camera['horizontal_fov_deg'])/2))
    fy=camera.get('fy_px',fx)
    cx,cy=camera.get('cx_px',camera['width_px']/2),camera.get('cy_px',camera['height_px']/2)
    return np.array([[fx*sx,0.,cx*sx],[0.,fy*sy,cy*sy],[0.,0.,1.]],dtype=float)


def optical_to_base(camera):
    """RPY는 x전방/y좌측/z위 camera body 기준. optical은 x우/y아래/z전방."""
    r,p,y=camera['base_rpy_rad']
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    rotation=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
        [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    return rotation @ np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])


def rectangle(rear_xy, heading):
    forward=np.array([math.cos(heading),math.sin(heading)])
    left=np.array([-forward[1],forward[0]])
    rear=np.asarray(rear_xy,dtype=float)
    return np.asarray([rear-WIDTH_M/2*left,rear+LENGTH_M*forward-WIDTH_M/2*left,
                       rear+LENGTH_M*forward+WIDTH_M/2*left,rear+WIDTH_M/2*left])


def valid_quad(corners,width,height):
    q=np.asarray(corners,dtype=float)
    if q.shape!=(4,2) or not np.isfinite(q).all() or (q<0).any() or (q[:,0]>=width).any() or (q[:,1]>=height).any():
        return False
    edges=np.roll(q,-1,axis=0)-q
    lengths=np.linalg.norm(edges,axis=1)
    cross=edges[:,0]*np.roll(edges[:,1],-1)-edges[:,1]*np.roll(edges[:,0],-1)
    return bool(lengths.min()>=6. and (np.all(cross>0) or np.all(cross<0)))


def pose_region(corners, target, profile, width, height):
    """검출한 검은 코드 네 꼭짓점으로만 추정한다. 상대 마커 높이를 가정하지 않는다."""
    if not valid_quad(corners,width,height):return None,'invalid_or_small_quad'
    camera=profile['camera'];matrix=camera_matrix(camera,width,height)
    distortion=np.asarray(camera.get('distortion',[]),dtype=float)
    half=target['black_code_size_m']/2
    obj=np.array([[-half,half,0.],[half,half,0.],[half,-half,0.],[-half,-half,0.]],dtype=float)
    q=np.asarray(corners,dtype=float)
    # 네 평면 점의 일반 반복 PnP. IPPE의 180도 rear-face 특이 수치 사례를
    # 보존한 r466 진단 뒤 단일 solver로 선택했다. 여러 solver fallback은 없다.
    ok,rvec,tvec=cv2.solvePnP(obj,q,matrix,distortion,flags=cv2.SOLVEPNP_ITERATIVE)
    solved=(ok,[rvec],[tvec])
    candidates=[];to_base=optical_to_base(camera)
    if solved[0]:
        for rvec,tvec in zip(solved[1],solved[2]):
            rotation=cv2.Rodrigues(rvec)[0];translation=np.asarray(tvec).reshape(3)
            if not np.isfinite(rotation).all() or not np.isfinite(translation).all():continue
            if np.any((obj @ rotation.T+translation)[:,2]<=0) or rotation[:,2] @ translation >= 0:continue
            projected=cv2.projectPoints(obj,rvec,tvec,matrix,distortion)[0].reshape(4,2)
            error=float(np.sqrt(np.mean(np.sum((projected-q)**2,axis=1))))
            forward=-(to_base @ rotation[:,2])
            if abs(forward[2])>.5 or np.linalg.norm(forward[:2])<.5:continue
            candidates.append((error,translation,forward))
    if not candidates:return None,'pose_unavailable'
    error,translation,forward=min(candidates,key=lambda item:item[0])
    if error>profile.get('max_reprojection_error_px',4.):return None,'reprojection_inconsistent'
    base=to_base @ (translation*profile.get('range_scale',1.))+np.asarray(camera['base_xyz_m'])
    if not 0 < base[0] or np.linalg.norm(base[:2])>profile.get('max_range_m',5.):return None,'outside_optional_range'
    heading=math.atan2(forward[1],forward[0])+profile.get('heading_offset_rad',0.)
    heading=math.atan2(math.sin(heading),math.cos(heading))
    return dict(dictionary=target['dictionary'],id=target['id'],rear_center_xy_m=base[:2].tolist(),
        observed_marker_center_z_m=float(base[2]),heading_rad=heading,
        polygon_xy_m=rectangle(base[:2],heading).tolist(),width_m=WIDTH_M,length_m=LENGTH_M,
        reprojection_error_px=error,black_code_size_m=target['black_code_size_m'],
        printed_image_size_m=target['printed_image_size_m'],size_basis=target['size_basis'],
        pose_method='planar_PnP_with_approximate_intrinsics',absolute_accuracy_verified=False,
        uncertainty=dict(intrinsics='nominal_FOV_or_simple_focal_adjustment',
            extrinsics='nominal_CAD_or_explicit_profile',black_code_size_basis=target['size_basis'],
            range_bound_m=None,heading_bound_rad=None,statistical_confidence_calibrated=False),
        vehicle_extent_is_assumed=True,relative_object_motion_corrected=False),None


def empty_packet(profile, *, source_stamp_s, frame_id, epoch, reason='no_usable_detection'):
    packet=dict(source_stamp_s=float(source_stamp_s),reference_s=float(source_stamp_s),
        source_frame_id=frame_id,frame_id='base_link',vision_epoch=epoch,
        profile_id=profile['profile_id'],mode='approximate_v1',purpose=profile.get('purpose','occupancy_assist'),
        max_age_s=profile.get('max_age_s',.25),regions=[],rejected=[],
        is_lidar_return=False,no_detection_is_clear=False,free_space_added=False,
        metric_accuracy_verified=False,timing_verified=False,time_basis='image_header_unverified_exposure')
    packet['nominal_camera_to_base']={key:profile['camera'][key] for key in ('base_xyz_m','base_rpy_rad')}
    packet['reason']=reason
    return packet


def estimate_regions(rgb, observations_4x4, profile, *, source_stamp_s, frame_id, epoch):
    """기존 4x4 검출을 재사용하고 필요한 명시 다른 사전만 추가 검출한다."""
    validate_profile(profile)
    height,width=rgb.shape[:2]
    packet=empty_packet(profile,source_stamp_s=source_stamp_s,frame_id=frame_id,epoch=epoch)
    if frame_id!=profile['camera']['optical_frame_id']:
        packet['reason']='image_frame_mismatch';return packet
    try:camera_matrix(profile['camera'],width,height)
    except ValueError:
        packet['reason']='image_profile_shape_mismatch';return packet
    observations={'DICT_4X4_50':observations_4x4}
    if any(t['dictionary']=='DICT_7X7_100' for t in profile['targets']):
        gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
        if hasattr(cv2.aruco,'ArucoDetector'):corners,ids,_=cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
        else:corners,ids,_=cv2.aruco.detectMarkers(gray,dictionary)
        observations['DICT_7X7_100']=[] if ids is None else [(int(i),q.reshape(4,2)) for i,q in zip(ids.flatten(),corners)]
    for target in profile['targets']:
        quads=[np.asarray(q,dtype=float) for identity,q in observations[target['dictionary']] if identity==target['id']]
        if not quads:continue
        # 동일 코드의 중복 사각형만 병합. 서로 다른 위치의 같은 ID는 그 프레임만 제외한다.
        ref=quads[0]
        equivalent=all(q.shape==ref.shape==(4,2) and np.max(np.linalg.norm(q-ref,axis=1))<=2. for q in quads)
        if not equivalent:region,reason=None,'ambiguous_duplicate'
        else:
            try:region,reason=pose_region(ref,target,profile,width,height)
            except (cv2.error,ValueError):region,reason=None,'pose_input_unusable'
        if region is None:packet['rejected'].append(dict(dictionary=target['dictionary'],id=target['id'],reason=reason))
        else:packet['regions'].append(region)
    packet['reason']='observed' if packet['regions'] else 'no_usable_detection'
    return packet
