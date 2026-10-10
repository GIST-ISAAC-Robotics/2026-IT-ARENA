#!/usr/bin/env python3
"""기록한 실행 설정/구독/MCU 상태에서 단일 모터축 계약을 확인한다. 주행 재시험 아님."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import yaml


def audit(folder):
    report = json.loads((folder/'report.json').read_text())
    with zipfile.ZipFile(folder/'source_snapshot.zip') as bundle:
        raw = bundle.read('src/arena_description/config/vehicle.yaml')
        config = yaml.safe_load(raw)['vehicle']
        source_matches = hashlib.sha256(raw).hexdigest() == report['source_sha256']['src/arena_description/config/vehicle.yaml']
    encoder = config['sensors']['drive_feedback']
    spec = encoder['drive_motor_encoder']
    ratio = config['drivetrain']['motor']['gear_ratio']
    radius = config['drivetrain']['wheel_radius_m']
    runtime = ET.parse(folder/'runtime_vehicle.sdf').find("model/plugin[@name='arena::SingleMotorDrive']")
    states = json.loads((folder/'actuation_trace.json').read_text())
    subscriptions = {name: report.get(name, []) for name in ('autonomy_subscriptions', 'safety_subscriptions')}
    checks = {
        'snapshot_config_hash_matches': source_matches,
        'single_motor_mode': encoder['mode'] == 'drive_motor_shaft',
        'resolution_28': spec['counts_per_motor_revolution'] == 28,
        'runtime_ratio_15': float(runtime.findtext('gear_ratio')) == ratio == 15.,
        'runtime_radius_32_5_mm': float(runtime.findtext('wheel_radius')) == radius == .0325,
        'mcu_all_motor_mode': bool(states) and all(s['feedback_mode'] == 'drive_motor_shaft' for s in states),
        'mcu_no_left_right_encoder_values': bool(states) and all(s.get('left_ticks') is None and s.get('right_ticks') is None
            and s.get('left_wheel_mps') is None and s.get('right_wheel_mps') is None for s in states),
        'motor_counts_present': any(s.get('motor_counts') is not None for s in states),
        'controllers_subscribe_motor_not_wheel_pair': all('/drive_motor/encoder' in topics and '/wheel_states' not in topics
                                                        for topics in subscriptions.values()),
    }
    return {'run': str(folder), 'passed': all(checks.values()), 'checks': checks,
            'report_sha256': hashlib.sha256((folder/'report.json').read_bytes()).hexdigest(),
            'subscriptions': subscriptions, 'mcu_samples': len(states),
            'counts_per_mean_wheel_revolution': spec['counts_per_motor_revolution']*ratio,
            'travel_mm_per_count': math.tau*radius*1000/(spec['counts_per_motor_revolution']*ratio),
            'limits': 'Gazebo inner motor PI uses ideal wheel-joint mean. Quantized single motor encoder feeds controller/deskew/virtual MCU; not full physical MCU speed-loop emulation.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.run)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
