"""보존 원시 자료의 공통 입력 구간 및 오픈 차동 상쇄 진단. 추가 주행 없음."""
import json
from pathlib import Path
from analyze_evaluator_gc import sha
from analyze_scheduled_apply import applied_window

BASE = Path(__file__).resolve().parent
def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]

result = {}
pdir = BASE/'p1_pgs'
p = rows(pdir/'braking_diagnostics.jsonl')
p_events = rows(pdir/'scheduled_events.jsonl')
for name in ('d1_dantzig', 'd2_dantzig'):
    d = rows(BASE/name/'braking_diagnostics.jsonl')
    d_events = rows(BASE/name/'scheduled_events.jsonl')
    a = applied_window(d_events, 2010000, 5710000)
    b = applied_window(p_events, 2010000, 5710000)
    count = next((i for i, pair in enumerate(zip(a, b)) if pair[0] != pair[1]), min(len(a), len(b)))
    if count >= min(len(a), len(b)):
        raise ValueError('expected recorded different tail')
    cutoff = min(a[count]['sim_us'], b[count]['sim_us'])/1e6
    dm = {round(r['sim_time_s']*1e9): r for r in d}
    differences = []
    for row in p:
        ns = round(row['sim_time_s']*1e9)
        if 2.01 <= row['sim_time_s'] < cutoff and ns in dm:
            other = dm[ns]
            differences.append({'sim_s': row['sim_time_s'], 'body_mps': abs(row['truth_longitudinal_mps']-other['truth_longitudinal_mps']),
                'wheel_mps': max(abs(row[k]-other[k])*.0325 for k in ('left_speed_rad_s', 'right_speed_rad_s'))})
    result[name+'_common_prefix'] = {'identical_envelopes': count, 'ends_exclusive_sim_s': cutoff,
        'samples': len(differences), 'peak_body_difference': max(differences, key=lambda x: x['body_mps']),
        'peak_wheel_difference': max(differences, key=lambda x: x['wheel_mps']),
        'first_body_over_001_mps': next((r for r in differences if r['body_mps'] > .01), None),
        'first_wheel_over_001_mps': next((r for r in differences if r['wheel_mps'] > .01), None)}
witness = next(r for r in p if abs(r['sim_time_s']-4.352) < 1e-9)
final = p[-1]
def motion(row):
    left, right = [row[k]*.0325 for k in ('left_speed_rad_s', 'right_speed_rad_s')]
    return {'sim_s': row['sim_time_s'], 'body_mps': row['truth_longitudinal_mps'], 'left_mps': left, 'right_mps': right,
        'mean_mps': (left+right)/2, 'reported_mean_mps': row['wheel_surface_speed_mps'],
        'motor_rad_s': row['motor_speed_rad_s'], 'motor_torque_nm': row['motor_torque_nm']}
result['pgs_cancellation_witness'] = motion(witness)
result['pgs_final_motion'] = motion(final)
result['checks'] = {'opposite_wheel_rotation': witness['left_speed_rad_s']*witness['right_speed_rad_s'] < 0,
    'mean_small_body_moving': abs(witness['wheel_surface_speed_mps']) < .008 and witness['truth_longitudinal_mps'] > .8,
    'single_motor_ratio_consistent': max(abs(r['motor_speed_rad_s']-15*(r['left_speed_rad_s']+r['right_speed_rad_s'])/2) for r in p) < 1e-8,
    'mean_relationship_consistent': max(abs(r['wheel_surface_speed_mps']-.0325*(r['left_speed_rad_s']+r['right_speed_rad_s'])/2) for r in p) < 1e-8}
result['raw_sha256'] = {str(f.relative_to(BASE)): sha(f) for n in ('d1_dantzig', 'd2_dantzig', 'p1_pgs')
    for f in (BASE/n/'braking_diagnostics.jsonl', BASE/n/'scheduled_events.jsonl')}
result['source_sha256'] = {Path(__file__).name: sha(Path(__file__))}
result['limits'] = ['differences before command divergence support physics-side influence, not the size of full-run effect',
    'mean motor-axis speed is not individual wheel speed or chassis standstill',
    'load/contact distribution is not validated physical hardware behavior']
with (BASE/'diagnosis.json').open('x') as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
raise SystemExit(0 if all(result['checks'].values()) else 1)
