"""정면 동일 평면 1~2거리 표본의 간이 조정. 전체 왜곡/외부파라미터 교정이 아니다."""
import copy
import math
import statistics

from arena_autonomy.vehicle_marker_geometry import finite,validate_profile


def optical_depth(sample,camera):
    value=finite(sample['measured_distance_m'],'measured distance')
    basis=sample.get('distance_basis','camera_optical_depth')
    if basis=='lidar_forward_same_plane':
        if sample.get('same_frontoparallel_plane') is not True or any(abs(x)>1e-9 for x in camera['base_rpy_rad']):
            raise ValueError('simple offset requires same frontal plane and zero camera body RPY')
        value+=finite(sample['lidar_base_x_m'],'lidar base x')-camera['base_xyz_m'][0]
    elif basis!='camera_optical_depth':
        raise ValueError('bumper gap/slant range is not camera optical depth')
    if value<=0:raise ValueError('positive optical depth required')
    return value


def adjust_profile(profile,samples,method='focal_pixels'):
    validate_profile(profile)
    if not 1<=len(samples)<=2:raise ValueError('one or two frontal samples required')
    result=copy.deepcopy(profile);camera=result['camera'];estimates=[]
    for sample in samples:
        if sample.get('frontoparallel') is not True:raise ValueError('explicit frontal sample required')
        depth=optical_depth(sample,camera)
        if method=='focal_pixels':
            pixel=finite(sample['pixel_width'],'pixel width');black=finite(sample['black_code_size_m'],'black code')
            width=finite(sample.get('image_width_px',camera['width_px']),'image width')
            if min(pixel,black,width)<=0:raise ValueError('positive size required')
            estimates.append(pixel*depth/black*camera['width_px']/width)
        elif method=='range_scale':
            estimated=finite(sample['estimated_camera_depth_m'],'estimated depth')
            if estimated<=0:raise ValueError('positive estimate required')
            estimates.append(depth/estimated)
        else:raise ValueError('unknown adjustment method')
    center=statistics.median(estimates)
    if method=='focal_pixels':
        camera['fx_px']=camera['fy_px']=center
        result['range_scale']=1.
    else:result['range_scale']=profile.get('range_scale',1.)*center
    offsets=[finite(s.get('reference_heading_rad',0.),'reference heading')-
        finite(s['estimated_heading_rad'],'estimated heading') for s in samples if 'estimated_heading_rad' in s]
    if offsets:
        correction=math.atan2(sum(math.sin(x) for x in offsets),sum(math.cos(x) for x in offsets))
        heading=profile.get('heading_offset_rad',0.)+correction
        result['heading_offset_rad']=math.atan2(math.sin(heading),math.cos(heading))
    result['profile_id']=profile['profile_id']+'_simple_adjustment'
    result['simple_adjustment']=dict(method=method,samples=copy.deepcopy(samples),derived_values=estimates,
        spread_fraction=(max(estimates)-min(estimates))/center,
        formula='fx_px = black_pixel_width * optical_depth_m / black_code_size_m',
        full_intrinsic_calibration=False,absolute_accuracy_verified=False)
    validate_profile(result)
    return result
