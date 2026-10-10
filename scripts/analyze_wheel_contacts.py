#!/usr/bin/env python3
"""평가 접촉 메시지와 구동 정답의 시간 대조. 미수신을 0하중으로 보간하지 않는다."""
import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

WHEELS = ('front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel')


def contact_rows(message):
    stamp = message['header']['stamp']
    t = float(stamp.get('sec', 0))+float(stamp.get('nsec', 0))*1e-9
    grouped = {}
    for contact in message.get('contact', []):
        names = [contact[f'collision{i}']['name'] for i in (1, 2)]
        for i, name in enumerate(names, 1):
            wheel = next((w for w in WHEELS if name == f'arena_car::{w}::collision'), None)
            if wheel is None:
                continue
            row = grouped.setdefault(wheel, {'t': t, 'wheel': wheel, 'force_world_xyz_n': [0., 0., 0.],
                                            'force_available': True, 'points': 0, 'other_collisions': []})
            row['other_collisions'].append(names[2-i])
            positions, wrenches = contact.get('position', []), contact.get('wrench', [])
            row['points'] += len(positions)
            if not positions or len(wrenches) != len(positions):
                row['force_available'] = False
            for wrench in wrenches:
                force = wrench.get(f'body{i}Wrench', {}).get('force')
                if force is None:
                    row['force_available'] = False
                    continue
                vector = [float(force.get(axis, 0.)) for axis in ('x', 'y', 'z')]
                if not all(math.isfinite(v) for v in vector):
                    raise ValueError('nonfinite force')
                row['force_world_xyz_n'] = [a+b for a, b in zip(row['force_world_xyz_n'], vector)]
    if not math.isfinite(t):
        raise ValueError('nonfinite contact timestamp')
    return list(grouped.values())


def nearest(rows, t, max_age=.006):
    times = [r['t'] for r in rows]
    at = bisect_left(times, t)
    candidates = [i for i in (at-1, at) if 0 <= i < len(rows)]
    if not candidates:
        return None
    row = min((rows[i] for i in candidates), key=lambda r: abs(r['t']-t))
    return row if abs(row['t']-t) <= max_age else None


def analyze(folder, onset=None):
    report = json.loads((folder/'report.json').read_text())
    for filename, key in [('wheel_contacts.jsonl', 'wheel_contacts'), ('braking_diagnostics.jsonl', 'braking_diagnostics')]:
        actual = hashlib.sha256((folder/filename).read_bytes()).hexdigest()
        if actual != report[key]['sha256']:
            raise ValueError(f'raw hash mismatch: {filename}')
    dynamics = [json.loads(line) for line in (folder/'braking_diagnostics.jsonl').read_text().splitlines()]
    radius = float(ET.parse(folder/'runtime_vehicle.sdf').findtext("model/plugin[@name='arena::SingleMotorDrive']/wheel_radius"))
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('invalid recorded wheel radius')
    if onset is None:
        onset = report['command_start_sim_s']+2.
    end = onset+.5
    rows = {w: [] for w in WHEELS}
    duplicates = 0
    seen = set()
    with (folder/'wheel_contacts.jsonl').open() as stream:
        for line in stream:
            for row in contact_rows(json.loads(line)):
                identity = json.dumps(row, sort_keys=True)
                if identity in seen:
                    duplicates += 1
                    continue
                seen.add(identity)
                rows[row['wheel']].append(row)
    for records in rows.values():
        records.sort(key=lambda r: r['t'])
    valid_dynamics = [r for r in dynamics if onset <= r['sim_time_s'] <= end]
    if not valid_dynamics:
        raise ValueError('missing braking window')
    peak = max(valid_dynamics, key=lambda r: abs(r['wheel_surface_speed_mps']-r['truth_longitudinal_mps']))
    stats = {}
    for wheel, records in rows.items():
        window = [r for r in records if onset <= r['t'] <= end]
        loads = [r['force_world_xyz_n'][2] for r in window if r['force_available']]
        stats[wheel] = {
            'observed_samples': len(window), 'missing_force_samples': sum(not r['force_available'] for r in window),
            'observed_fz_n_min_p05_median_p95_max': np.percentile(loads, [0, 5, 50, 95, 100]).tolist() if loads else None,
            'max_observed_gap_s': max((b['t']-a['t'] for a, b in zip(window, window[1:])), default=None),
            'observed_load_below_0_1_n_fraction': sum(v < .1 for v in loads)/len(loads) if loads else None,
            'at_peak_velocity_mismatch': nearest(records, peak['sim_time_s']),
        }
    # 초기 낙하 충격을 제외하고, 정지 구간의 총 하중이 약 mg인지 확인할 참고값.
    reference_t = min(r['sim_time_s'] for r in dynamics)+1.
    reference = {w: nearest(records, reference_t) for w, records in rows.items()}
    return {'run': str(folder), 'report_run_valid': report.get('run_valid', report.get('input_valid')),
            'window_s': [onset, end], 'max_mean_wheel_body_speed_difference_mps':
                abs(peak['wheel_surface_speed_mps']-peak['truth_longitudinal_mps']),
            'max_left_right_surface_speed_difference_mps': max(abs(r['left_speed_rad_s']-r['right_speed_rad_s'])*radius for r in valid_dynamics),
            'wheel_radius_m': radius,
            'input_sha256': {name: hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in
                             ('report.json', 'runtime_vehicle.sdf', 'wheel_contacts.jsonl', 'braking_diagnostics.jsonl')},
            'peak_dynamics': peak, 'wheel_contact_stats': stats,
            'initial_reference_t_s': reference_t, 'initial_reference_contacts': reference,
            'identical_duplicate_rows_removed': duplicates,
            'limits': 'event-only force on each wheel in world frame. Missing/stale rows remain null, never assumed zero load. Diagnostic window does not override whole-run coverage or safety verdict.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--onset', type=float)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = analyze(args.run, args.onset)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('run', 'window_s', 'max_mean_wheel_body_speed_difference_mps', 'max_left_right_surface_speed_difference_mps')}, indent=2))
