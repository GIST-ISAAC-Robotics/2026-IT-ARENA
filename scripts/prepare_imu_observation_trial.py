#!/usr/bin/env python3
"""11/11 IMU 저장 기록의 입력 검사와 선택형 시뮬 민감도 프로필 생성.

기울어진 미장착 센서의 평균/산포를 최종 교정이나 백색잡음 분포로 확정하지 않는다.
ROS 통신/하드웨어를 시작하지 않는다. 현재 운동 입력 콜백만 메모리에서 호출한다.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml
from sensor_msgs.msg import Imu
from arena_autonomy.lidar_motion_ros import MotionInput

ROOT = Path(__file__).resolve().parents[1]
SUMMARY = ROOT/'artifacts/validation/2026-11-11/esp32_imu_input_v1/no_gc_summary.json'
SENSOR = ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'


class OfflineNode:
    """네트워크 없는 ROS 매개변수/시각 최소 대역. source clock 정확성 인증 아님."""
    def __init__(self):
        self.parameters = {'motion_imu_frame_id': 'imu_probe'}
        self.time = 0.

    def declare_parameter(self, name, default):
        self.parameters.setdefault(name, default)
        return self.get_parameter(name)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def has_parameter(self, name):
        return name in self.parameters

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.time*1e9)))

    def create_subscription(self, *_):
        pass


def audit_recording(path):
    node = OfflineNode()
    receiver = MotionInput(node)
    origin = boot = None
    samples = []
    for line in path.open(encoding='utf-8'):
        row = json.loads(line)
        if row.get('phase') != 'measure' or row.get('sensor_tag') != 1 or row.get('type') != 1:
            continue
        if boot is None:
            boot, origin = row['boot'], row['read_begin_us']
        if row['boot'] != boot:
            raise ValueError('multiple MCU boots require separate histories')
        t = 1.+(row['read_begin_us']-origin)*1e-6
        values = [int(v)*.0175*math.pi/180 for v in row['raw_xyz']]
        if not np.allclose(values, row['gyro_rad_s'], rtol=0, atol=1e-15):
            raise ValueError('raw gyro and recorded SI conversion differ')
        msg = Imu()
        msg.header.frame_id = 'imu_probe'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(t*1e9), 1_000_000_000)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = values
        node.time = t
        receiver.on_imu(msg)
        samples.append((t, *values))
    if not samples:
        raise ValueError('no measured gyroscope samples')
    data = np.asarray(samples)
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError('MCU read time not strictly increasing')
    return dict(count=len(data), accepted=receiver.counts['gyro'], rejected=receiver.counts['rejected'],
                rejection_reasons=receiver.gyro_rejections,
                gyro_mean_rad_s=data[:, 1:].mean(axis=0).tolist(),
                gyro_stddev_rad_s=data[:, 1:].std(axis=0).tolist(),
                max_read_interval_ms=float(np.diff(data[:, 0]).max()*1000),
                input_time_basis='MCU read_begin_us relative; no sensor acquisition/host-clock claim',
                frame_assumption='raw sensor z treated as body yaw for sensitivity only',
                firmware_or_ros_live=False, timing_verified=False, calibration_adopted=False,
                recording_sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def profiles(summary, base):
    gyro = summary['streams']['gyro_rad_s']
    result = copy.deepcopy(base)
    # 표현한 것은 이번 5분 수신 평균과 관측 산포의 근사다. 차체 진동/온도 모델 아님.
    result['imu'].update(status='observed_unmounted_2026_11_11_sensitivity_only',
        nominal_rate_hz=base['imu'].get('nominal_rate_hz', base['imu']['rate_hz']), rate_hz=gyro['host_window_rate_hz'],
        simulated_gyro_stddev_rad_s=.0011, simulated_accel_stddev_mps2=.0103)
    fault = dict(imu={'bias': gyro['provisional_only']['apparent_bias_rad_s'][2]}, seed=1111)
    return result, fault


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='new directory')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists; preserve earlier evidence')
    summary = json.loads(SUMMARY.read_text(encoding='utf-8'))
    base = yaml.safe_load(SENSOR.read_text(encoding='utf-8'))
    audit = audit_recording(args.recording)
    gyro = summary['streams']['gyro_rad_s']
    if audit['count'] != gyro['count'] or audit['accepted'] != audit['count'] or audit['rejected']:
        raise ValueError('recording/callback mismatch; do not generate trial')
    profile, fault = profiles(summary, base)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'sensor.yaml').write_text('# 선택형 IMU 민감도 조건. 실물 교정/시각 검증 아님.\n'+
        yaml.safe_dump(profile, allow_unicode=True, sort_keys=False), encoding='utf-8')
    (args.output/'motion.json').write_text(json.dumps(fault, indent=2)+'\n', encoding='utf-8')
    bias = fault['imu']['bias']
    evidence = dict(input_audit=audit,
        summary_sha256=hashlib.sha256(SUMMARY.read_bytes()).hexdigest(),
        base_profile_sha256=hashlib.sha256(SENSOR.read_bytes()).hexdigest(),
        changed_sensor_keys=['imu.'+key for key,value in profile['imu'].items()
                             if value != base['imu'].get(key)],
        gyro_bias_trial_rad_s=bias, gyro_bias_trial_dps=math.degrees(bias),
        gyro_bias_error_deg_over_100ms=math.degrees(bias*.1),
        gyro_bias_error_deg_over_300ms=math.degrees(bias*.3),
        range_2m_lateral_error_100ms_m=2*abs(math.sin(bias*.1)),
        assumptions=['gyro mean is an error hypothesis, not proven zero bias',
                     'three-axis scatter approximated as equal independent Gaussian sigma',
                     'Gazebo rate must be checked from actual source stamps',
                     'no measured latency claim from host receipt gap',
                     'baseline profile, control/STOP rules and time verification unchanged'])
    (args.output/'preflight.json').write_text(json.dumps(evidence, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'samples': audit['count'], 'accepted': audit['accepted'], 'rejected': audit['rejected'],
                      'requested_rate_hz': profile['imu']['rate_hz'], 'bias_rad_s': bias}))


if __name__ == '__main__':
    main()
