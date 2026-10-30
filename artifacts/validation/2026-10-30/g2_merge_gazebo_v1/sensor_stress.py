"""시험용 센서 오차. 모델 변화와 실제 취득 기록을 검증하며 제어 보정값은 그대로 둔다."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parent
SIGMA = .02
BIAS = math.radians(1.)
YAW = -math.radians(.5)


def sensor(root, name):
    matches = root.findall(f".//sensor[@name='{name}']")
    if len(matches) != 1:
        raise ValueError('expected exactly one sensor: '+name)
    return matches[0]


def apply_stress(xml):
    root = ET.fromstring(xml)
    lidar = sensor(root, 'rplidar_c1')
    if lidar.find('pose') is not None:
        raise ValueError('baseline lidar local pose changed')
    ET.SubElement(lidar, 'pose').text = f'0 0 0 0 0 {YAW}'
    imu = sensor(root, 'lsm6dsox_imu')
    for axis in ('x', 'y', 'z'):
        noise = imu.find('imu/angular_velocity/'+axis+'/noise')
        if (noise is None or noise.get('type') != 'gaussian'
                or float(noise.findtext('mean')) != 0.
                or float(noise.findtext('stddev')) != .001):
            raise ValueError('baseline gyro changed')
        noise.find('stddev').text = str(SIGMA)
        noise.find('mean').text = str(BIAS if axis == 'z' else 0.)
    return ET.tostring(root, encoding='unicode')


def verify_stress(output):
    from g1_lidar_stress import audit_injection
    output = Path(output)
    root = ET.parse(output/'runtime_vehicle.sdf').getroot()
    lidar = sensor(root, 'rplidar_c1')
    pose = list(map(float, lidar.findtext('pose').split()))
    imu = sensor(root, 'lsm6dsox_imu')
    actual = {}
    for axis in ('x', 'y', 'z'):
        noise = imu.find('imu/angular_velocity/'+axis+'/noise')
        actual[axis] = dict(mean=float(noise.findtext('mean')), stddev=float(noise.findtext('stddev')))
    with (output/'g2_decisions.jsonl').open(encoding='utf-8') as stream:
        motion = json.loads(next(stream))['data']['motion_config']
    acquired = json.loads((output/'acquisition.json').read_text())
    with (output/'scans_received.jsonl').open(encoding='utf-8') as stream:
        received = [json.loads(line) for line in stream]
    requested = json.loads((BASE/'range_profile.json').read_text())
    radial = audit_injection(acquired, received, requested)
    checks = dict(lidar_local_yaw=pose == [0., 0., 0., 0., 0., YAW],
        gyro_sigma=all(v['stddev'] == SIGMA for v in actual.values()),
        gyro_z_bias=actual['z']['mean'] == BIAS,
        other_gyro_bias_zero=actual['x']['mean'] == actual['y']['mean'] == 0.,
        estimator_gyro_bias_unchanged=motion['gyro_bias_rad_s'] == 0.,
        estimator_lidar_yaw_unchanged=motion['lidar_yaw_rad'] == 0.,
        radial_acquisition_audit=radial['verified'])
    return dict(passed=all(checks.values()), checks=checks, native_gyro=actual,
        radial=radial, scope='synthetic error model and acquisition audit; not a measured C1 distribution')

