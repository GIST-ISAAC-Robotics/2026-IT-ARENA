"""세 각본의 기준·v1·v2 실제 주행을 같은 지표로 모은다. 원래 판정 실패를 그대로 옮긴다(사후 평가)."""
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))
from compare_runs import summary
from hold_slip import run as hold_run
from slip_events import body_speed

import numpy as np

L = BASE.parent / 'rolling_lead_v1'
C = BASE.parent / 'race_follow_closing_v1'
ROWS = {
    'first_curve_17s': dict(baseline=(L / 'run1', L / 'run1_surface.json'),
                            v1_release=(L / 'curve_low_motion', BASE / 'curve_surface.json'),
                            v2_pace_floor=(L / 'curve_paced', BASE / 'curve_paced_surface.json')),
    'stop_restart_15s': dict(baseline=(C / 'stop', C / 'stop_surface.json'),
                             v1_release=(L / 'stop_low_motion', BASE / 'stop_surface.json'),
                             v2_pace_floor=(L / 'stop_paced', BASE / 'stop_paced_surface.json')),
    'fast_stop_9p85s': dict(baseline=(BASE / 'fast_stop_closing', BASE / 'fast_stop_closing_surface.json'),
                            v1_release=(BASE / 'fast_stop_low_motion', BASE / 'fast_stop_low_motion_surface.json'),
                            v2_pace_floor=(BASE / 'fast_stop_paced', BASE / 'fast_paced_surface.json')),
}


def stalls(run):
    r = json.loads((run / 'report.json').read_bytes())
    t, v, _, _ = body_speed(run, r['case_window']['green_sim_s'])
    k = t <= r['case_window']['duration_s']
    t, v = t[k], v[k]
    first = t[np.flatnonzero(v >= .05)[0]]
    out, i = [], 0
    while i < len(t):
        if t[i] >= first and v[i] < .05:
            j = i
            while j + 1 < len(t) and v[j + 1] < .05:
                j += 1
            out.append([round(float(t[i]), 2), round(float(t[j]), 2)])
            i = j + 1
        else:
            i += 1
    return out


def first_bypass(run):
    green = json.loads((run / 'report.json').read_bytes())['case_window']['green_sim_s']
    with (run / 'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            e = json.loads(line)
            if e['kind'] == 'call' and e['data']['output'][2].get('handoff_state') == 'BYPASS':
                return round(e['data']['input']['reference_s'] - green, 3)
    return None


result = {}
for scene, arms in ROWS.items():
    result[scene] = {}
    for arm, (run, surface) in arms.items():
        s = summary(run, surface)
        wheels = json.loads(surface.read_bytes())['interpretations']['any']
        hold = hold_run(run)
        result[scene][arm] = dict(
            run=run.relative_to(BASE.parents[3]).as_posix(), reserve=s['handoff_reserve'], run_valid=s['run_valid'],
            source_unchanged=s['source_unchanged'], shutdown_clean=s['shutdown_clean'], stop_all_passed=all(s['stop_checks'].values()),
            passed=s['passed'], failed_gates=s['failed_gates'], progress_m=s['progress_m'], final_gap_m=s['final_gap_m'],
            min_gap_m=s['min_gap_m'], contact_messages=s['contact_messages'], wall_collision_samples=s['wall_collision_samples'],
            guard_samples=s['guard_samples'], max_off_road_wheels=wheels['max_off_wheels'], penalty_events=wheels['penalty_events_merged'],
            longest_grass_s=wheels['longest_continuous_grass_contact_s'], peak_body_mps=s['peak_body_mps'],
            below_005_s=s['after_start_below_005_s'], below_020_s=s['after_start_below_020_s'], stall_windows_s=stalls(run),
            phases=s['phases'], prepare_hold_s=hold['hold_s'], wheel_body_mismatch_s=hold['mismatch_total_s'],
            wheel_body_events=s['wheel_body_mismatch'], first_bypass_s=first_bypass(run),
            motor_audit_passed=None)
audits = {('first_curve_17s', 'v1_release'): 'curve_motor_audit.json', ('stop_restart_15s', 'v1_release'): 'stop_motor_audit.json',
          ('fast_stop_9p85s', 'baseline'): 'fast_stop_closing_motor_audit.json', ('fast_stop_9p85s', 'v1_release'): 'fast_stop_low_motion_motor_audit.json',
          ('first_curve_17s', 'v2_pace_floor'): 'curve_paced_motor_audit.json', ('stop_restart_15s', 'v2_pace_floor'): 'stop_paced_motor_audit.json',
          ('fast_stop_9p85s', 'v2_pace_floor'): 'fast_paced_motor_audit.json'}
for (scene, arm), name in audits.items():
    result[scene][arm]['motor_audit_passed'] = json.loads((BASE / name).read_bytes())['passed']
with (BASE / 'three_scene_summary.json').open('w', encoding='utf-8', newline='\n') as f:
    json.dump(dict(scenes=result, limits='각 설정 한 번씩의 시뮬레이션 실행. 궤적·스캔 위상이 달라 통계 우열이나 실차 성능을 증명하지 않는다.'),
              f, ensure_ascii=False, indent=1)
for scene, arms in result.items():
    print('==', scene)
    for arm, x in arms.items():
        drops = [round(p['worst_drop_030']['mps'], 3) for p in x['phases']]
        print(f"{arm:14s} prog {x['progress_m']:.3f} final {x['final_gap_m']:.3f} min {x['min_gap_m']:.3f} contact {x['contact_messages']} wall {x['wall_collision_samples']}"
              f" wheels {x['max_off_road_wheels']} pen {x['penalty_events']} hold {x['prepare_hold_s']:.2f} <.05 {x['below_005_s']:.2f} <.20 {x['below_020_s']:.2f}"
              f" drops {drops} bypass@{x['first_bypass_s']} guard {x['guard_samples']} stop {x['stop_all_passed']} audit {x['motor_audit_passed']}")
        print(f"               failed {x['failed_gates']} stalls {x['stall_windows_s']} valid {x['run_valid']} src {x['source_unchanged']} shut {x['shutdown_clean']} mismatch {x['wheel_body_mismatch_s']:.2f}")
