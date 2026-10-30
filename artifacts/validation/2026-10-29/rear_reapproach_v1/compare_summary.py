"""OFF(현재 코드)와 ON(rear_lane_keep_trial) 합성 결과를 같은 등록 판정으로 요약한다. 평가 전용."""
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
PAIRS = {'nominal': ('current3', 'lane5'), 'combined_error': ('robust1_off', 'robust3_on'),
         'fast_lead_diagnostic': ('fast1_off', 'fast3_on')}
KEYS = ('scenario_realized', 'forward_progress_m', 'longest_stationary_while_lead_moves_s', 'contact',
        'toward_side_encroachment_m', 'minimum_side_gap_m', 'minimum_alongside_side_gap_m', 'minimum_lead_gap_m',
        'minimum_residual_gap_m', 'maximum_wheels_outside_road', 'stop_command_zero', 'first_release_after_guard_s',
        'rear_lane_keep_applied', 'stop_restart', 'safety_reduced')


def verdict(c):
    lead_moving = c['case'][0] != 'C'
    gates = dict(contact=c['contact'] is None, wheels=c['maximum_wheels_outside_road'] <= 2,
                 stop=c['stop_command_zero'], replay=(c.get('self_replay') or {}).get('passed') is True)
    if lead_moving:
        gates.update(progress=c['forward_progress_m'] >= 2.5, stall=c['longest_stationary_while_lead_moves_s'] <= 2.5)
    else:
        sr = c['stop_restart']
        gates.update(hold=sr['longest_hold_s'] >= 1., restart=sr['restart_delay_s'] is not None and sr['restart_delay_s'] <= 2.5,
                     after=sr['progress_after_restart_m'] >= 1.)
    return dict(gates=gates, passed=c['scenario_realized'] and all(gates.values()),
                scenario_realized=c['scenario_realized'])


out = {}
for label, (off, on) in PAIRS.items():
    a = json.loads((BASE/off/'summary.json').read_text(encoding='utf-8'))['cases']
    b = json.loads((BASE/on/'summary.json').read_text(encoding='utf-8'))['cases']
    out[label] = {name: {'off': {**{k: a[name].get(k) for k in KEYS}, **verdict(a[name])},
                         'on': {**{k: b[name].get(k) for k in KEYS}, **verdict(b[name])}}
                  for name in a if name in b}
out['scope'] = 'synthetic sensor closed loop (ideal bicycle, sequential rays, shadow safety); not ROS/Gazebo/hardware'
(BASE/'comparison_summary.json').write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
for label, rows in out.items():
    if label == 'scope':
        continue
    for name, r in rows.items():
        f = lambda x: (round(x['forward_progress_m'], 2), x['longest_stationary_while_lead_moves_s'],
                       None if x['toward_side_encroachment_m'] is None else round(x['toward_side_encroachment_m'], 3),
                       round(x['minimum_side_gap_m'], 3), x['contact'], x['passed'], x['scenario_realized'])
        print(label, name, 'OFF', f(r['off']), 'ON', f(r['on']))
