"""명시 선택형 원천 LiDAR ROS 입력. 메시지가 스스로 실물 시각 검증을 승인할 수 없다."""
import json
import math

from arena_autonomy.measurement_scan import MeasurementScan


def decode_measurements(text, *, expected_time_basis):
    if not isinstance(text,str) or len(text)>2_000_000:
        raise ValueError('native scan payload too large')
    if expected_time_basis not in ('simulation_acquisition','unverified_estimate'):
        raise ValueError('native hardware acquisition timing is not approved')
    raw=json.loads(text)
    expected={'schema','ranges_m','angles_rad','acquired_at_s','source_frame_id','source_epoch',
              'range_min_m','range_max_m','time_basis'}
    if not isinstance(raw,dict) or set(raw)!=expected or raw.pop('schema')!='arena_native_lidar_v1':
        raise ValueError('invalid native scan schema')
    if raw['time_basis']!=expected_time_basis:
        raise ValueError('native timing basis differs from selected input')
    ranges=[]
    for v in raw['ranges_m']:
        if v is None: ranges.append(math.nan)
        elif v=='+inf': ranges.append(math.inf)
        elif v=='-inf': ranges.append(-math.inf)
        elif isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v): ranges.append(float(v))
        else: raise ValueError('invalid native range encoding')
    raw['ranges_m']=tuple(ranges)
    return MeasurementScan(**raw)


def input_representation(node):
    """먼저 선택을 검사한 노드에서도 선언된 원값/기존 기본 표현을 유지한다."""
    name = 'lidar_input_representation'
    parameter = node.get_parameter(name) if node.has_parameter(name) else node.declare_parameter(name, 'laser_scan')
    return str(parameter.value)


def subscribe_scan(node,callback,qos):
    """한 표현/한 토픽만 구독한다. 시뮬 native 모드는 기본 OFF다."""
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from arena_autonomy.quick_self_mask import configure_node
    callback = configure_node(node, callback)
    mode=input_representation(node)
    if (getattr(getattr(node,'motion',None),'timing_policy','verified_only') == 'host_estimated_v1'
            and mode != 'native_estimated_v1'):
        raise ValueError('host estimate policy requires the native estimated representation')
    if mode=='laser_scan':
        return node.create_subscription(LaserScan,'/scan',callback,qos)
    if mode not in ('native_simulation_v1','native_estimated_v1'):
        raise ValueError('unknown LiDAR input representation')
    if mode=='native_simulation_v1' and not node.get_parameter('use_sim_time').value:
        raise ValueError('native simulation time requires use_sim_time')
    if node.motion is None or node.motion.mode!='both':
        raise ValueError('native input requires both compensation')
    basis='simulation_acquisition' if mode=='native_simulation_v1' else 'unverified_estimate'
    def receive(message):
        try:
            scan=decode_measurements(message.data,expected_time_basis=basis)
        except (ValueError,TypeError,KeyError,OverflowError) as error:
            node.get_logger().warning('Invalid native LiDAR input: '+str(error),throttle_duration_sec=5.)
            return
        callback(scan)
    return node.create_subscription(String,'/scan/native',receive,qos)
