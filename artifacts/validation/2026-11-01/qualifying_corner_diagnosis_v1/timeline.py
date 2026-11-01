"""진행 거리 기준 비교표와 마지막 코너 판단 시간표를 원시 추출본에서 만든다."""
import gzip
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
NAMES = ('rise_only', 'turn_only', 'combined_failed')


def load(name):
    with gzip.open(HERE / f'extract_{name}.json.gz', 'rt', encoding='utf-8') as f:
        return json.load(f)


def speed(r):
    return math.hypot(r['truth_longitudinal_mps'] or 0., r['truth_lateral_mps'] or 0.)


def first_pass(rows, goal):
    for a, b in zip(rows, rows[1:]):
        if a['progress_m'] < goal <= b['progress_m']:
            f = (goal - a['progress_m']) / (b['progress_m'] - a['progress_m'])
            return a, b, f
    return None


def lerp(a, b, f, key):
    return a[key] + f * (b[key] - a[key])


def progress_table(data):
    print('# progress table: t(green-rel)  body  wheel  applied  | per run')
    head = '  p   ' + ''.join(f'| {n:^33s}' for n in NAMES)
    print(head)
    for k in range(0, 97):
        goal = k / 2
        line = f'{goal:5.1f} '
        for n in NAMES:
            rows = [r for r in data[n]['trajectory'] if r['light'] == 'green']
            hit = first_pass(rows, goal)
            if hit is None:
                line += '| ' + ' ' * 33
                continue
            a, b, f = hit
            t = lerp(a, b, f, 'time') - rows[0]['time']
            line += (f"| {t:6.2f} {speed(a)+f*(speed(b)-speed(a)):5.2f} "
                     f"{lerp(a,b,f,'wheel_surface_speed_mps'):5.2f} {lerp(a,b,f,'applied_target_mps'):5.2f} "
                     f"{lerp(a,b,f,'centerline_error_m'):+6.3f} ")
        print(line)


def window(data, name, lo, hi):
    rows = [r for r in data[name]['trajectory'] if r['light'] == 'green' and lo <= r['progress_m'] <= hi]
    t0, t1 = rows[0]['time'], rows[-1]['time']
    print(f'\n# {name} trajectory {lo}-{hi} m: t {t0:.3f}-{t1:.3f}')
    print('  time  simT   prog   body  vlat  wheel  appl   cmd  safe  steerC  stL   stR   yawr  torq   rawD   limD  piAft  state/safety')
    for r in rows:
        print(f"{r['time']:6.3f} {r['sim_time_s']:6.3f} {r['progress_m']:6.3f} {r['truth_longitudinal_mps']:5.2f} "
              f"{r['truth_lateral_mps']:+5.2f} {r['wheel_surface_speed_mps']:5.2f} {r['applied_target_mps']:5.2f} "
              f"{(r['speed_command_mps'] or 0):5.2f} {(r['safe_speed_mps'] or 0):5.2f} {(r['steering_command_rad'] or 0):+6.3f} "
              f"{r['left_steering_rad']:+5.2f} {r['right_steering_rad']:+5.2f} {r['truth_yaw_rate_rad_s']:+5.2f} "
              f"{r['motor_torque_nm']:+6.3f} {r['raw_demand_nm']:+6.3f} {r['limited_demand_nm']:+6.3f} {r['pi_integral_after_nm']:+6.3f} "
              f"{r['state']} {r['safety_reason']}")
    calls = [c for c in data[name]['calls'] if c['kind'] == 'call' and t0 - .1 <= c['reference_s'] <= t1 + .1]
    print(f'\n# {name} decisions')
    print(' ref_s   scan_s  meas  tgt  cArc   lenCap steer  prevS  remain clear  sel/preview/reason')
    for c in calls:
        print(f"{c['reference_s']:6.3f} {c['scan_stamp_s']:6.3f} {c['speed_mps']:5.2f} {c['target']:5.2f} "
              f"{(c['constant_arc_target_mps'] if c['constant_arc_target_mps'] is not None else -1):5.2f} "
              f"{(c['observed_length_speed_cap_mps'] if c['observed_length_speed_cap_mps'] is not None else -1):5.2f} "
              f"{c['steering']:+6.3f} {c['previous_steering_rad']:+6.3f} "
              f"{(c['observed_path_ahead_m'] if c['observed_path_ahead_m'] is not None else -1):5.2f} "
              f"{(c['swept_clearance_m'] if c['swept_clearance_m'] is not None else -1):5.2f} "
              f"{c['reason']} b={c['behavior']} sel={c['boundary_selection']} cand={c['boundary_candidate_reason']}/{c['boundary_candidate_ahead_m']} "
              f"fresh={c['fresh_boundary_accepted']} mem={c['boundary_from_memory']} wall={c['path_wall']} open={c['open_route_speed_enabled']} tp={c['turn_preview']}")


if __name__ == '__main__':
    data = {n: load(n) for n in NAMES}
    progress_table(data)
    window(data, 'combined_failed', 41.0, 44.0)
    window(data, 'rise_only', 41.0, 43.3)
    window(data, 'turn_only', 41.0, 43.3)
