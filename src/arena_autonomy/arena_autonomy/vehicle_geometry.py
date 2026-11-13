"""차량 YAML의 명시 기하를 계획/preview 매개변수로 옮기는 순수 변환."""
import math
import struct


def float32_limit_not_above(limit):
    """Ackermann float32 반올림으로 물리/MCU 상한을 넘지 않는 송신 상한."""
    if not math.isfinite(limit) or limit <= 0:
        raise ValueError('Command limit must be finite and positive')
    try:
        packed = struct.pack('<f', limit)
    except OverflowError as error:
        raise ValueError('Command limit cannot be represented as float32') from error
    rounded = struct.unpack('<f', packed)[0]
    if rounded > limit:
        bits = struct.unpack('<I', packed)[0]
        rounded = struct.unpack('<f', struct.pack('<I', bits-1))[0]
    if rounded <= 0:
        raise ValueError('Command limit is below positive float32 range')
    return rounded


def positive(value,name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
        raise ValueError(name+' must be a finite positive number')
    return float(value)


def planning_parameters(vehicle):
    """명시 YAML에서 누락/부정합 시 실패한다. legacy 기본값으로 조용히 대체하지 않는다."""
    drive=vehicle['drivetrain'];body=vehicle['body'];footprint=vehicle['footprint']
    length=positive(footprint['length_m'],'footprint.length_m')
    width=positive(footprint['width_m'],'footprint.width_m')
    wheelbase=positive(drive['wheelbase_m'],'wheelbase_m')
    track=positive(drive['track_width_m'],'track_width_m')
    radius=positive(drive['wheel_radius_m'],'wheel_radius_m')
    wheel_width=positive(drive['wheel_width_m'],'wheel_width_m')
    body_length=positive(body['length_m'],'body.length_m')
    body_width=positive(body['width_m'],'body.width_m')
    offset=body.get('geometry_center_x_m',0.)
    if isinstance(offset,bool) or not isinstance(offset,(int,float)) or not math.isfinite(offset):
        raise ValueError('body geometry offset must be finite')
    if (2*abs(offset)+body_length>length+1e-9 or body_width>width+1e-9
            or wheelbase+2*radius>length+1e-9 or track+wheel_width>width+1e-9):
        raise ValueError('planning envelope must contain body and straight wheels')
    return dict(vehicle_length_m=length,vehicle_width_m=width,
        vehicle_track_width_m=track,vehicle_wheel_radius_m=radius,vehicle_wheel_width_m=wheel_width)


def equivalent_steering_limit(drivetrain):
    """기존 simulation.launch의 중앙 등가각 변환을 그대로 공유한다."""
    model=drivetrain.get('simulation_steering_model','ideal_ackermann')
    wheelbase,track,limit=(positive(drivetrain[key],key) for key in
        ('wheelbase_m','track_width_m','max_steering_angle_rad'))
    if limit>=1.4:raise ValueError('invalid steering dimensions or limit')
    if model=='fusion_cad_2026_11_13':
        if (drivetrain.get('simulation_drive_mode')!='single_motor'
                or abs(wheelbase-.128)>1e-6 or abs(track-.1239)>1e-6):
            raise ValueError('CAD steering requires final geometry and single_motor mode')
        from arena_vehicle_interface.steering_calibration import CadMapping
        return CadMapping.default().command_limit(wheelbase=wheelbase,track=track,wheel_limit=limit)
    if model!='ideal_ackermann':raise ValueError('unknown steering model')
    return math.atan(wheelbase/(wheelbase/math.tan(limit)+track/2))


def preview_parameters(vehicle):
    """선택한 차량의 계획/운동 기하만 전달한다. 속도·기어비·출력/시계 정책은 바꾸지 않는다."""
    result=planning_parameters(vehicle);drive=vehicle['drivetrain']
    wheelbase=positive(drive['wheelbase_m'],'wheelbase_m')
    return {**result,'wheelbase_m':wheelbase,'motion_rear_axle_x_m':-wheelbase/2,
        'motion_wheel_radius_m':result['vehicle_wheel_radius_m'],
        'max_steering_angle_rad':float32_limit_not_above(equivalent_steering_limit(drive))}
