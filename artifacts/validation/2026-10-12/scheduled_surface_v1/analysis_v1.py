"""예약 적용 시점까지 같은 노면 A/B: 출처/형상 관문과 접촉 해석을 분리한다."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from analyze_braking_ab import ROOT, floor_collision, normalized_world, rows, sha
from analyze_scheduled_apply import analyze_run, compare_envelopes
from analyze_scheduled_phase import compare_metrics
from analyze_wheel_contacts import contact_rows, nearest, WHEELS
from analyze_zero_speed_modes import phase_stats

FLAT_MODEL = '''<model name="eval_flat_support"><static>true</static><link name="floor">
<collision name="collision"><pose>0 0 -0.047 0 0 0</pose><geometry><box><size>40 40 0.1</size>
</box></geometry></collision></link></model>'''


def canonical(element):
    return ET.canonicalize(ET.tostring(element, encoding='unicode'), strip_text=True)


def surface_audit(official, flat):
    """노면 제거 비교와 별도로 추가 판 전체를 검사하여 물성/부가 형상을 숨기지 않는다."""
    a, acount = normalized_world(official)
    b, bcount = normalized_world(flat)
    original, candidate = ET.parse(official), ET.parse(flat)
    flat_models = candidate.findall("world/model[@name='eval_flat_support']")
    floors = [c for c in original.findall('world/model/link/collision') if floor_collision(c.get('name', ''))]
    checks = {
        'same_world_except_floor': a == b,
        'official_no_eval_flat': not original.findall("world/model[@name='eval_flat_support']"),
        'official_multiple_floors': acount >= 2,
        'flat_no_old_floors': bcount == 0,
        'exact_flat_model': len(flat_models) == 1 and canonical(flat_models[0]) == canonical(ET.fromstring(FLAT_MODEL)),
        'floor_material_unspecified': bool(floors) and all(c.find('surface') is None for c in floors),
    }
    return {'checks': checks, 'passed': all(checks.values()), 'removed_floor_count': acount,
            'flat_remaining_original_floors': bcount, 'flat_top_z_m': .003,
            'limits': '형상/지지 평형을 함께 바꾸는 대조. 개별 이음매/엔진 내부 원인 확정이 아니다.'}


def zero_application(events, start_us, end_us):
    applied = [e for e in events if e.get('type') == 'applied' and start_us <= e['sim_us'] <= end_us]
    positive = [e for e in applied if e['speed'] > 0]
    if not positive:
        raise ValueError('missing positive applied command')
    last = positive[-1]['sim_us']
    zero = next((e for e in applied if e['sim_us'] > last and e['speed'] == 0), None)
    if zero is None:
        raise ValueError('missing final applied zero')
    return {k: zero[k] for k in ('seq', 'stamp_us', 'apply_us', 'sim_us', 'speed', 'steering')}


def collect_contacts(messages):
    contacts = {w: [] for w in WHEELS}
    seen = set()
    for message in messages:
        for row in contact_rows(message):
            key = json.dumps(row, sort_keys=True)
            if key not in seen:
                seen.add(key)
                contacts[row['wheel']].append(row)
    for records in contacts.values():
        records.sort(key=lambda r: r['t'])
    return contacts


def contact_summary(records, begin, end, peak_t, end_exclusive=False):
    window = [r for r in records if begin <= r['t'] <= end and (not end_exclusive or r['t'] < end)]
    loads = [r['force_world_xyz_n'][2] for r in window if r['force_available']]
    times = [r['t'] for r in window]
    # 같은 시각의 서로 다른 기록 중 하나를 정답으로 고르지 않는다.
    duplicates = len(times) - len(set(times))
    closest = nearest(records, peak_t)
    ambiguous = closest is not None and sum(r['t'] == closest['t'] for r in records) > 1
    return {'observed_records': len(window), 'conflicting_timestamp_records': duplicates,
            'missing_force_records': sum(not r['force_available'] for r in window),
            'fz_min_p05_median_p95_max_n': np.percentile(loads, [0, 5, 50, 95, 100]).tolist() if loads else None,
            'max_observed_gap_s': max((b-a for a, b in zip(times, times[1:])), default=None),
            'other_collisions': sorted({n for r in window for n in r['other_collisions']}),
            'at_peak': None if ambiguous else closest, 'at_peak_ambiguous': ambiguous,
            'peak_time_offset_s': closest['t']-peak_t if closest and not ambiguous else None}


def braking_contacts(data, events, messages, start_ns, duration_s, brake_window, radius):
    start_us = start_ns//1000
    zero = zero_application(events, start_us, start_us+round(duration_s*1e6)+200000)
    tzero = zero['sim_us']/1e6
    begin, end = [start_ns/1e9+t for t in brake_window]
    if not begin < tzero < end:
        raise ValueError('applied zero outside registered braking window')
    contacts = collect_contacts(messages)
    phases = {}
    for name, lo, hi in (('before_zero', begin, tzero), ('after_zero', tzero, end)):
        selected = [r for r in data if lo <= r['sim_time_s'] <= hi and
                    (name != 'before_zero' or r['sim_time_s'] < tzero)]
        if not selected:
            raise ValueError('missing phase dynamics')
        mean_peak = max(selected, key=lambda r: abs(r['wheel_surface_speed_mps']-r['truth_longitudinal_mps']))
        lr_peak = max(selected, key=lambda r: abs(r['left_speed_rad_s']-r['right_speed_rad_s']))
        phases[name] = {**phase_stats(selected), 'left_right_peak_difference_mps':
                       abs(lr_peak['left_speed_rad_s']-lr_peak['right_speed_rad_s'])*radius,
                       'left_right_peak_sim_s': lr_peak['sim_time_s'],
                       'wheel_contacts': {w: contact_summary(records, lo, hi, mean_peak['sim_time_s'],
                                                            end_exclusive=name == 'before_zero')
                                          for w, records in contacts.items()}}
    # 약 20 ms의 관측 진단 그대로 남긴다. 누락을 보간하지 않는다.
    timeline = []
    for r in data:
        if tzero-.06 <= r['sim_time_s'] <= tzero+.3:
            samples = {w: contact_summary(records, r['sim_time_s'], r['sim_time_s'], r['sim_time_s'])
                       for w, records in contacts.items()}
            timeline.append({k: r[k] for k in ('sim_time_s', 'applied_target_mps', 'wheel_surface_speed_mps',
                'truth_longitudinal_mps', 'left_speed_rad_s', 'right_speed_rad_s', 'truth_x_m', 'truth_y_m',
                'truth_z_m', 'truth_yaw_rad', 'motor_torque_nm')} | {
                    'relative_to_zero_s': r['sim_time_s']-tzero,
                    'contact_nearest': {w: samples[w]['at_peak'] for w in WHEELS}})
    return {'actual_zero_envelope': zero, 'zero_sim_s': tzero, 'phases': phases, 'zero_timeline': timeline,
            'limits': ['접촉 기록 없음은 0하중을 뜻하지 않는다. ±6 ms 이내 가까운 기록만 대응시킨다.',
                       '속도/자세는 평가용 정답이다. 제어 입력이나 실차 정지 판단으로 사용하지 않는다.']}


def comparison_checks(a, b, envelopes, shape):
    common = ('source_sha256', 'runtime_vehicle_sha256', 'meshes_sha256', 'profile', 'spawn_pose')
    return {'both_valid': a['passed'] and b['passed'],
            'both_scheduled': a['scheduled'] and b['scheduled'],
            'start_equal': a['start_ns'] == b['start_ns'], 'duration_equal': a['duration_s'] == b['duration_s'],
            'same_initial_step': a['trace']['trace']['spawn'] == b['trace']['trace']['spawn'],
            'step_1ms': a['trace']['step_s'] == b['trace']['step_s'] == .001,
            **{'same_'+k: a['evidence'][k] == b['evidence'][k] for k in common},
            'identical_normal_application': envelopes['same_all_normal_applied_envelopes'],
            'surface_only_intervention': shape['passed']}


def analyze(base):
    registration = json.loads((base/'registered_manifest.json').read_text())
    contract_path = ROOT/'config/tests/scheduled_command_r7_sha256.json'
    contract = json.loads(contract_path.read_text())
    if sha(contract_path) != registration['contract_sha256']:
        raise ValueError('registration contract mismatch')
    bundles = [analyze_run(base/name, contract, registration['plugin_sha256']) for name in ('s1_official', 'f1_flat')]
    a, b = [v[0] for v in bundles]
    start = a['start_ns']//1000
    envelopes = compare_envelopes(bundles[0][2], bundles[1][2], start, start+round(a['duration_s']*1e6)+200000)
    shape = surface_audit(base/a['name']/'runtime_world.sdf', base/b['name']/'runtime_world.sdf')
    checks = comparison_checks(a, b, envelopes, shape)
    metrics, within, secondary = compare_metrics(a['trace'], b['trace'])
    contacts = {}
    for summary, data, events in bundles:
        run = base/summary['name']
        report = json.loads((run/'report.json').read_text())
        radius = float(ET.parse(run/'runtime_vehicle.sdf').findtext("model/plugin[@name='arena::SingleMotorDrive']/wheel_radius"))
        checks[summary['name']+'_registered_sources'] = all(registration['sources'].get(p) == value
            for p, value in report['source_sha256'].items())
        checks[summary['name']+'_mode'] = report['zero_speed_mode'] == 'passive_damping'
        checks[summary['name']+'_surface'] = report['surface'] == ('official' if summary is a else 'flat')
        contacts[summary['name']] = braking_contacts(data, events, rows(run/'wheel_contacts.jsonl'),
            summary['start_ns'], summary['duration_s'], report['command_profile']['brake_window_s'], radius)
    # 후처리 의존 스크립트도 기록하여 재분석 버전을 구분한다.
    import sys
    sources = {str(Path(m.__file__).resolve().relative_to(ROOT)): sha(Path(m.__file__).resolve())
               for m in list(sys.modules.values()) if getattr(m, '__file__', None)
               and Path(m.__file__).resolve().is_relative_to(ROOT/'scripts')}
    sources[Path(__file__).relative_to(ROOT).as_posix()] = sha(Path(__file__))
    return {'runs': [a, b], 'checks': checks, 'surface_comparison_valid': all(checks.values()),
            'surface_audit': shape, 'envelopes': envelopes, 'metrics': metrics,
            'metrics_within_existing_tolerance': within, 'secondary_metrics': secondary,
            'contacts': contacts, 'analysis_source_sha256': sources,
            'limits': ['한 쌍의 평가용 비교이며 실차/전체 G1/물리 수렴 합격이 아니다.',
                       'passive_damping은 실물 MD20A 회로 재현이 아니며 기본 모델에 채택하지 않는다.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.base.resolve())
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('checks', 'surface_comparison_valid', 'metrics')}, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['surface_comparison_valid'] else 1)
