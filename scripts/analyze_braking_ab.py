#!/usr/bin/env python3
"""고정 요청 제동 대조의 입력/모델 불변성과 운동 차이. 원시 실행 판정을 바꾸지 않는다."""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from analyze_wheel_contacts import contact_rows, nearest, WHEELS

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def floor_collision(name):
    return name == 'ground_plane_link__ground_collision' or name.startswith(('surface_', 'grass_'))


def normalized_world(path):
    root = ET.parse(path).getroot()
    world = root.find('world')
    floor_count = 0
    for model in list(world.findall('model')):
        if model.get('name') == 'eval_flat_support':
            world.remove(model)
            continue
        for link in model.findall('link'):
            for collision in list(link.findall('collision')):
                if floor_collision(collision.get('name', '')):
                    link.remove(collision)
                    floor_count += 1
    # 같은 정적 시각 메시를 실행 폴더별 절대 경로로 생성하므로 내용 해시로 비교.
    for node in root.findall('.//uri'):
        if node.text and Path(node.text).name.startswith('batched_'):
            value = Path(node.text.removeprefix('file://'))
            if not value.is_absolute():
                value = path.parent/value
            if not value.is_file():
                value = path.parent/value.name  # launch가 보존한 실행 메시 사본
            node.text = 'batch-content-sha256:'+sha(value)
    return ET.canonicalize(ET.tostring(root, encoding='unicode'), strip_text=True), floor_count


def summarize_run(run):
    report = json.loads((run/'report.json').read_text())
    hashes = {name: sha(run/(name+'.jsonl')) == report[name]['sha256'] for name in
              ('braking_diagnostics', 'wheel_contacts', 'requested_commands', 'safe_commands')}
    zip_checks, current = {}, {}
    with zipfile.ZipFile(run/'source_snapshot.zip') as bundle:
        for name, expected in report['source_sha256'].items():
            zip_checks[name] = hashlib.sha256(bundle.read(name)).hexdigest() == expected
            current[name] = sha(ROOT/name) == expected
    data = rows(run/'braking_diagnostics.jsonl')
    start = report['command_start_sim_s']
    begin, end = [start+t for t in report['command_profile']['brake_window_s']]
    window = [r for r in data if begin <= r['sim_time_s'] <= end]
    active = [r for r in data if start <= r['sim_time_s'] <= report['stop_request_sim_s']]
    radius = float(ET.parse(run/'runtime_vehicle.sdf').findtext("model/plugin[@name='arena::SingleMotorDrive']/wheel_radius"))
    peak = max(window, key=lambda r: abs(r['wheel_surface_speed_mps']-r['truth_longitudinal_mps']))
    initial = min(data, key=lambda r: abs(r['sim_time_s']-start))
    contacts = {w: [] for w in WHEELS}
    for message in rows(run/'wheel_contacts.jsonl'):
        for row in contact_rows(message):
            if begin-.01 <= row['t'] <= end+.01:
                contacts[row['wheel']].append(row)
    stats = {}
    for wheel, records in contacts.items():
        records.sort(key=lambda r: r['t'])
        # 동일 공유 토픽에 센서가 중복한 완전 동일 행만 제거.
        unique = {json.dumps(r, sort_keys=True): r for r in records}
        records = sorted(unique.values(), key=lambda r: r['t'])
        loads = [r['force_world_xyz_n'][2] for r in records if r['force_available'] and begin <= r['t'] <= end]
        stats[wheel] = {'samples': len(records), 'missing_force_samples': sum(not r['force_available'] for r in records),
                       'fz_min_p05_median_p95_max_n': np.percentile(loads, [0, 5, 50, 95, 100]).tolist() if loads else None,
                       'max_observed_gap_s': max((b['t']-a['t'] for a, b in zip(records, records[1:])), default=None),
                       'at_peak': nearest(records, peak['sim_time_s'])}
    safety = {}
    for row in active:
        reason = (row.get('safety') or {}).get('reason', 'not_yet_received')
        safety[reason] = safety.get(reason, 0)+1
    checks = {'report_passed': report['passed'], 'raw_hashes': all(hashes.values()),
              'source_zip': bool(zip_checks) and all(zip_checks.values()),
              'source_still_current': all(current.values()),
              'single_motor_feedback': all((r.get('mcu') or {}).get('feedback_mode') == 'drive_motor_shaft' for r in active),
              'no_two_wheel_sensor': all((r.get('mcu') or {}).get('left_ticks') is None and
                                         (r.get('mcu') or {}).get('right_ticks') is None for r in active)}
    result = {'run': str(run.relative_to(ROOT)), 'surface': report['surface'], 'checks': checks,
              'audit_passed': all(checks.values()), 'command_start_sim_s': start,
              'initial_pose': {k: initial[k] for k in ('truth_x_m', 'truth_y_m', 'truth_z_m', 'truth_yaw_rad')},
              'initial_body_speed_mps': initial['truth_longitudinal_mps'],
              'brake_window_sim_s': [begin, end],
              'peak_mean_wheel_body_difference_mps': abs(peak['wheel_surface_speed_mps']-peak['truth_longitudinal_mps']),
              'full_active_peak_difference_mps': max(abs(r['wheel_surface_speed_mps']-r['truth_longitudinal_mps']) for r in active),
              'max_left_right_surface_difference_mps': max(abs(r['left_speed_rad_s']-r['right_speed_rad_s'])*radius for r in window),
              'body_forward_reverse_wheel_samples': sum(r['truth_longitudinal_mps'] > .1 and
                  min(r['left_speed_rad_s'], r['right_speed_rad_s'])*radius < -.03 for r in window),
              'peak_dynamics': {k: v for k, v in peak.items() if k not in ('mcu','request','safe','safety')},
              'peak_mcu_state': peak.get('mcu'), 'safety_reason_samples': safety, 'wheel_contact_stats': stats,
              'input_sha256': {name: sha(run/name) for name in ('report.json', 'source_snapshot.zip', 'runtime_vehicle.sdf',
                  'runtime_world.sdf', 'braking_diagnostics.jsonl', 'wheel_contacts.jsonl', 'requested_commands.jsonl', 'safe_commands.jsonl')}}
    return result, report, data


def sampled_difference(a, b, key_a, key_b=None):
    """정규화 시각의 유지값 대조. 보간으로 명령 차이를 숨기지 않는다."""
    key_b = key_b or key_a
    if not a or not b:
        return {'samples': 0, 'max_abs': None, 'p95_abs': None, 'mean_abs': None}
    last = min(a[-1]['t'], b[-1]['t'])
    first = max(a[0]['t'], b[0]['t'], 0.)
    ta, tb = [r['t'] for r in a], [r['t'] for r in b]
    values = []
    for t in np.arange(first, last, .02):
        ia, ib = bisect_right(ta, t)-1, bisect_right(tb, t)-1
        if t-ta[ia] > .05 or t-tb[ib] > .05:
            continue
        values.append(abs(a[ia][key_a]-b[ib][key_b]))
    return {'samples': len(values), 'max_abs': max(values) if values else None,
            'p95_abs': float(np.percentile(values, 95)) if values else None,
            'mean_abs': float(np.mean(values)) if values else None}


def compare(a, b):
    summaries = [summarize_run(p) for p in (a, b)]
    (_, ra, da), (_, rb, db) = summaries
    wa, ca = normalized_world(a/'runtime_world.sdf')
    wb, cb = normalized_world(b/'runtime_world.sdf')
    diffs = {}
    for name in ('requested_commands', 'safe_commands'):
        streams = [[{**r, 't': r['sim_s']-rep['command_start_sim_s']} for r in rows(run/(name+'.jsonl'))]
                   for run, rep in ((a, ra), (b, rb))]
        for key in ('speed_mps', 'steering_rad'):
            diffs[name+'_'+key] = sampled_difference(*streams, key)
    mcu_streams = []
    for data, rep in ((da, ra), (db, rb)):
        unique = {r['mcu']['now_us']: r['mcu'] for r in data if r.get('mcu')}
        mcu_streams.append([{**v, 't': t*1e-6-rep['command_start_sim_s']} for t, v in sorted(unique.items())])
    for key in ('target_speed_mps', 'steering_rad'):
        diffs['mcu_'+key] = sampled_difference(*mcu_streams, key)
    checks = {'both_run_audits': all(s[0]['audit_passed'] for s in summaries),
              'same_profile': ra['command_profile']['sha256'] == rb['command_profile']['sha256'],
              'same_sources': ra['source_sha256'] == rb['source_sha256'],
              'same_spawn': ra['spawn'] == rb['spawn'],
              'same_vehicle': sha(a/'runtime_vehicle.sdf') == sha(b/'runtime_vehicle.sdf'),
              'world_equal_except_ground_collision_and_batch_paths': wa == wb}
    return {'runs': [s[0] for s in summaries], 'comparison_checks': checks, 'comparison_audit_passed': all(checks.values()),
            'floor_collision_counts': [ca, cb], 'aligned_command_differences': diffs,
            'analysis_sha256': sha(Path(__file__)),
            'limits': ['요청 프로필 동일성과 적용 명령 동일성은 다르다. 차이는 정규화 시각의 유지값 표본으로 보고한다.',
                       '접촉은 사건형이다. 누락은 하중 0이 아니고 최근 표본 ±6 ms 밖은 null이다.',
                       '사전 표본화 요청 대조이며 원래 G1 실패의 정확한 재생/해결이나 실차 제동 검증이 아니다.',
                       '노면 변화가 결과를 바꾸더라도 분할 경계/하중/접촉 엔진 중 단일 기여를 바로 확정하지 않는다.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('a', type=Path)
    parser.add_argument('b', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.a.resolve(), args.b.resolve())
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({'checks': result['comparison_checks'], 'runs': [{k: r[k] for k in
        ('run', 'audit_passed', 'peak_mean_wheel_body_difference_mps', 'max_left_right_surface_difference_mps',
         'body_forward_reverse_wheel_samples')} for r in result['runs']],
        'aligned_command_differences': result['aligned_command_differences']}, indent=2))
