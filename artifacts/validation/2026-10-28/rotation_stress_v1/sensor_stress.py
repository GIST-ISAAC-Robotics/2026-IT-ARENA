"""시험 전용 물리 센서 yaw/gyro 오차. 제어기 보정값과 차량 물성은 바꾸지 않는다."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

SIGMA=.02
BIAS=math.radians(1.)
LIDAR_YAW=-math.radians(.5)


def sensor(root,name):
    values=root.findall(f".//sensor[@name='{name}']")
    if len(values)!=1:raise ValueError('expected exactly one sensor: '+name)
    return values[0]


def noise_nodes(root):
    imu=sensor(root,'lsm6dsox_imu')
    nodes={a:imu.find('imu/angular_velocity/'+a+'/noise') for a in ('x','y','z')}
    if any(n is None or n.get('type')!='gaussian' for n in nodes.values()):
        raise ValueError('expected Gaussian gyro axes')
    return nodes


def apply_stress(xml):
    root=ET.fromstring(xml)
    lidar=sensor(root,'rplidar_c1')
    if lidar.find('pose') is not None:raise ValueError('baseline sensor pose changed')
    ET.SubElement(lidar,'pose').text=f'0 0 0 0 0 {LIDAR_YAW}'
    for axis,node in noise_nodes(root).items():
        if float(node.findtext('mean'))!=0. or float(node.findtext('stddev'))!=.001:
            raise ValueError('baseline gyro noise changed')
        node.find('stddev').text=str(SIGMA)
        if axis=='z':node.find('mean').text=str(BIAS)
    return ET.tostring(root,encoding='unicode')


def verify_runtime_stress(output):
    try:
        root=ET.parse(Path(output)/'runtime_vehicle.sdf').getroot()
        pose=list(map(float,sensor(root,'rplidar_c1').findtext('pose').split()))
        actual={a:dict(mean=float(n.findtext('mean')),stddev=float(n.findtext('stddev')))
                for a,n in noise_nodes(root).items()}
        with (Path(output)/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
            motion=json.loads(next(stream))['data']['motion_config']
        checks=dict(sensor_pose=pose==[0.,0.,0.,0.,0.,LIDAR_YAW],
                    gyro_sigma=all(v['stddev']==SIGMA for v in actual.values()),
                    z_bias=actual['z']['mean']==BIAS,
                    other_bias_zero=actual['x']['mean']==actual['y']['mean']==0.,
                    bias_not_corrected_by_oracle=motion['gyro_bias_rad_s']==0.,
                    mounting_error_not_corrected_by_oracle=motion['lidar_yaw_rad']==0.)
        return dict(passed=all(checks.values()),checks=checks,actual_gyro=actual,actual_lidar_pose=pose,
                    scope='native sensor model errors and unchanged estimator calibration; not hardware measurement')
    except (OSError,ValueError,TypeError,AttributeError,KeyError,StopIteration,ET.ParseError) as error:
        return dict(passed=False,error=f'{type(error).__name__}: {error}')
