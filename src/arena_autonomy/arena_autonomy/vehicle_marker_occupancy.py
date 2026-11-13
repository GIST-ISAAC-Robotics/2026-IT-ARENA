"""영상 차량 영역만 기존 swept/제동 함수로 검사한다. 원시 ray/벽/자유 공간은 그대로다."""
import math
from contextlib import contextmanager
from contextvars import ContextVar
import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.local_path import swept_limit, braking_speed

_call_cloud = ContextVar('vehicle_marker_call_cloud',default=None)


@contextmanager
def marker_scope(data,history,config,profile,clock_domain):
    """현재 한 plan 호출 전용. 예외/중첩 후 원 context를 복구하고 오래된 cloud를 캐시하지 않는다."""
    from arena_autonomy.vehicle_marker_channel import checked_packet
    packet=checked_packet(data.get('vehicle_marker_regions'),profile,data['reference_s'],clock_domain)
    regions=[] if packet is None else current_regions(packet,history,data['reference_s'],packet['vision_epoch'])[0]
    value=None
    if regions:
        age=data['reference_s']-packet['source_stamp_s']
        value=dict(points=np.vstack([p for _,p in regions]),
            padding=.025+config.measurement_padding_m+config.object_motion_bound_mps*age+math.sqrt(2)*.01/2,
            source_stamp_s=packet['source_stamp_s'],checks=0,limited_checks=0)
    token=_call_cloud.set(value)
    try:yield value
    finally:_call_cloud.reset(token)


def additional_clearance(clearance,steering,previous_steering,config,*,horizon=2.5):
    """벽/관측 점은 받지 않는다. 기존 후보의 점유 결과에 별도 점유 sweep 최솟값만 합성."""
    current=_call_cloud.get()
    if current is None:return clearance
    marker=min(swept_limit(current['points'],angle,config.wheelbase_m,horizon=horizon,
        margin=current['padding'],length=config.length_m,width=config.width_m)
        for angle in (steering,previous_steering))
    current['checks']+=1;current['limited_checks']+=marker<clearance
    return min(clearance,marker)


def occupied_samples(polygon, spacing=.01):
    """15x20 cm 사각형 내부의 유한 점유 샘플. LiDAR 측정으로 기록하지 않는다."""
    p=np.asarray(polygon,dtype=float)
    if p.shape!=(4,2) or not np.isfinite(p).all():raise ValueError('finite four-corner footprint required')
    a,b=p[1]-p[0],p[3]-p[0]
    if not np.allclose(p[2],p[0]+a+b,atol=1e-8):raise ValueError('rectangular footprint required')
    la,lb=np.linalg.norm(a),np.linalg.norm(b)
    if not np.allclose([la,lb],[.20,.15],atol=1e-8) or abs(a@b)>1e-8:
        raise ValueError('explicit assumed 15x20 cm extent required')
    u=np.linspace(0,1,int(math.ceil(la/spacing))+1)
    v=np.linspace(0,1,int(math.ceil(lb/spacing))+1)
    return (p[0]+u[:,None,None]*a+v[None,:,None]*b).reshape(-1,2)


def current_regions(packet, history, now, vision_epoch):
    """자차 이력만 이동 보정. 상대 자체 운동/영상 노출 시각 검증을 가정하지 않는다."""
    if (not isinstance(packet,dict) or not packet.get('fresh') or packet.get('vision_epoch')!=vision_epoch
            or packet.get('purpose')!='occupancy_assist' or packet.get('frame_id')!='base_link'):
        return [],'no_current_optional_region'
    stamp=packet.get('source_stamp_s')
    if type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0<=now-stamp<packet.get('max_age_s',.25):
        return [],'optional_region_expired'
    result=[]
    try:
        for region in packet['regions']:
            points=occupied_samples(region['polygon_xy_m'])
            # 취득 시각/epoch는 바꾸지 않고 base_link(image_stamp)→base_link(now)만 수행한다.
            points=transport_points(points,history,stamp,now)
            result.append((region,points))
    except (MotionUnavailable,ValueError,KeyError,TypeError):
        return [],'optional_region_transport_unavailable'
    return result,'current_optional_region' if result else 'no_detection'


def constrain_command(output, packet, history, now, vision_epoch, previous_steering, config):
    """기존 양수 후보만 추가 점유로 제한. 미검출/만료/보정불가는 원 output 그대로 반환한다.

    추월을 새로 시작하거나 경로를 만들지 않는다. 기존 후보가 옆으로 지나면 같은 함수로
    점유 충돌을 대조할 수 있다. current 검사/독립 보호층의 후속 허가를 대체하지 않는다.
    """
    if output is None or output[0]<=0:return output
    regions,reason=current_regions(packet,history,now,vision_epoch)
    if not regions:return output
    speed,steering,detail=output
    age=now-packet['source_stamp_s']
    padding=(.025+config.measurement_padding_m+config.object_motion_bound_mps*age
             +math.sqrt(2)*.01/2)  # 1 cm 내부 샘플의 최대 양자화 거리만 추가.
    points=np.vstack([points for _,points in regions])
    clearance=min(swept_limit(points,angle,config.wheelbase_m,horizon=2.5,margin=padding,
        length=config.length_m,width=config.width_m) for angle in (steering,previous_steering))
    available=max(0.,clearance-config.standstill_gap_m)
    cap=min(braking_speed(available,age),available/config.headway_s)
    if clearance<=config.hold_clearance_m:cap=0.
    selected=min(float(speed),float(cap))
    evidence=dict(source_stamp_s=packet['source_stamp_s'],reference_s=now,age_s=age,
        targets=[{'dictionary':r['dictionary'],'id':r['id']} for r,_ in regions],
        extent_m={'width':.15,'length':.20},swept_clearance_m=clearance,speed_cap_mps=cap,
        applied=selected<speed,unmodified_speed_mps=speed,profile_id=packet['profile_id'],
        method='existing_swept_limit_braking_headway',approximate=True,
        raw_lidar_modified=False,free_space_added=False,relative_object_motion_corrected=False)
    # 보조가 후보를 바꾸지 않으면 원 메타까지 exact. 빈 관측의 정지/대기 latch가 없다.
    if selected==speed:return output
    return selected,steering,{**detail,'vehicle_marker_occupancy':evidence,
        'reason':'vehicle_marker_occupied_hold' if selected==0 else detail.get('reason','vehicle_marker_follow'),
        'underlying_reason':detail.get('reason'),'behavior':'HOLD' if selected==0 else detail.get('behavior','FOLLOW')}


def apply_plan(output,data,history,config,profile,clock_domain):
    """실제 evaluator의 tuple 출력에만 추가 제한. OFF/optional 입력 오류는 exact."""
    if profile is None:return output
    from arena_autonomy.vehicle_marker_channel import checked_packet
    packet=checked_packet(data.get('vehicle_marker_regions'),profile,data['reference_s'],clock_domain)
    if packet is None:return output
    result=constrain_command(output,packet,history,data['reference_s'],packet['vision_epoch'],
        data['previous_steering_rad'],config)
    scope=_call_cloud.get()
    if result is not None and scope is not None and scope['limited_checks']:
        result=(result[0],result[1],{**result[2],'vehicle_marker_sweep':{
            key:scope[key] for key in ('source_stamp_s','checks','limited_checks')}})
    return result


def apply_current(result,data,history,config,profile,clock_domain):
    if profile is None or result is None or result['speed_mps']<=0:return result
    candidate=(result['speed_mps'],result['steering_rad'],result)
    output=apply_plan(candidate,data,history,config,profile,clock_domain)
    if output is candidate:return result
    return {**output[2],'speed_mps':output[0],'steering_rad':output[1]}
