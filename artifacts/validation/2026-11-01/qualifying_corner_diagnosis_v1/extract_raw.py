"""세 원시 실행에서 마지막 코너 진단에 필요한 필드만 뽑아 시간표로 저장한다.

입력은 저장 파일 읽기뿐이다. 차체 정답은 평가용으로 표에만 싣고 제어 입력이 아니다.
"""
import gzip
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RUNS = {
    'rise_only': ROOT / 'race_acceleration_v1/run1',
    'turn_only': ROOT / 'turn_preview_v1/run4',
    'combined_failed': ROOT / 'qualifying_profile_v1/run1',
}
TRAJ_KEYS_DYN = (
    'sim_time_s', 'applied_target_mps', 'reference_speed_mps', 'wheel_surface_speed_mps',
    'truth_longitudinal_mps', 'truth_lateral_mps', 'truth_yaw_rate_rad_s', 'truth_x_m', 'truth_y_m',
    'truth_yaw_rad', 'truth_pitch_rad', 'truth_roll_rad', 'truth_z_m', 'motor_speed_rad_s',
    'pre_motor_speed_rad_s', 'motor_torque_nm', 'raw_demand_nm', 'limited_demand_nm',
    'pi_integral_before_nm', 'pi_integral_after_nm', 'left_speed_rad_s', 'right_speed_rad_s',
    'left_torque_nm', 'right_torque_nm', 'left_steering_rad', 'right_steering_rad',
)
DETAIL_KEYS = (
    'reason', 'behavior', 'path_wall', 'wall_fallback', 'boundary_selection',
    'boundary_candidate_reason', 'boundary_candidate_ahead_m', 'fresh_boundary_accepted',
    'bilateral_alternative_confirmed', 'boundary_age_s', 'boundary_from_memory', 'path_source_s',
    'observed_path_ahead_m', 'swept_clearance_m', 'occupancy_padding_m', 'occupied_points',
    'target_speed_before_hold_mps', 'turn_preview', 'constant_arc_target_mps',
    'open_route_speed_enabled', 'open_route_cap_mps', 'observed_length_speed_cap_mps',
    'observed_search_horizon_m', 'source_first_s', 'scan_age_s', 'side', 'stage',
    'corner_distance_agrees_memory',
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def load_trajectory(folder):
    rows = []
    with (folder / 'trajectory.jsonl').open(encoding='utf-8') as f:
        for line in f:
            r = json.loads(line)
            d = r.get('dynamics') or {}
            a = r.get('autonomy') or {}
            s = r.get('safety') or {}
            row = dict(time=r['time'], x=r['x'], y=r['y'], yaw=r['yaw'], s=r['s'],
                       progress_m=r['progress_m'], centerline_error_m=r['centerline_error_m'],
                       collision=r['collision'], light=r['light'], odom_speed=r.get('odom_speed'),
                       state=a.get('state'), side=a.get('side'),
                       autonomy_sim_s=a.get('sim_time_s'), reference_stamp_s=a.get('reference_stamp_s'),
                       speed_command_mps=a.get('speed_command_mps'),
                       steering_command_rad=a.get('steering_command_rad'),
                       auto_reason=a.get('reason'), auto_behavior=a.get('behavior'),
                       safety_reason=s.get('reason'), safe_speed_mps=s.get('safe_speed_mps'),
                       safe_drive_speed_mps=r.get('safe_drive_speed_mps'),
                       swept_clearance_safety_m=s.get('swept_clearance_m'))
            row.update({k: d.get(k) for k in TRAJ_KEYS_DYN})
            rows.append(row)
    return rows


def load_calls(folder):
    calls = []
    keys_seen = {}
    header = None
    with (folder / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
        for line in f:
            r = json.loads(line)
            if r['kind'] == 'header':
                header = r['data']
            if r['kind'] == 'reset':
                calls.append(dict(seq=r['seq'], kind='reset', reason=r['data'].get('reason')))
                continue
            if r['kind'] != 'call':
                continue
            d = r['data']
            i = d['input']
            target, steering, detail = d['output']
            for k in detail:
                keys_seen[k] = keys_seen.get(k, 0) + 1
            pts = detail.get('path_points')
            row = dict(seq=r['seq'], kind='call', reference_s=i['reference_s'], dt_s=i['dt_s'],
                       scan_stamp_s=i['scan']['sec'] + i['scan']['nanosec'] * 1e-9,
                       speed_mps=i['speed_mps'], side=i['side'],
                       previous_steering_rad=i['previous_steering_rad'],
                       target=target, steering=steering, path_n=len(pts) if pts else 0)
            row.update({k: detail.get(k) for k in DETAIL_KEYS})
            calls.append(row)
    return header, calls, keys_seen


def main():
    out = {}
    for name, folder in RUNS.items():
        traj = load_trajectory(folder)
        header, calls, keys = load_calls(folder)
        with (folder / 'actuation_trace.json').open(encoding='utf-8') as f:
            act = json.load(f)
        act_rows = [dict(now_us=a['now_us'], state=a['state'], reason=a['reason'],
                         target_speed_mps=a['target_speed_mps'], steering_rad=a['steering_rad'],
                         mean_wheel_mps=a['mean_wheel_mps'], feedback_capture_us=a['feedback_capture_us'],
                         motor_counts=a['motor_counts']) for a in act]
        report = json.loads((folder / 'report.json').read_bytes())
        data = dict(folder=str(folder.relative_to(ROOT)).replace('\\', '/'),
                    files={k: sha256(folder / k) for k in (
                        'trajectory.jsonl', 'follow_decisions.jsonl', 'actuation_trace.json', 'report.json')},
                    follow_config=header['follow_config'], motion_config=header['motion_config'],
                    detail_keys=keys, trajectory=traj, calls=calls, actuation=act_rows)
        with gzip.open(HERE / f'extract_{name}.json.gz', 'wt', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, allow_nan=True)
        green = next(r['time'] for r in traj if r['light'] == 'green')
        out[name] = dict(folder=data['folder'], files=data['files'], trajectory_rows=len(traj),
                         calls=sum(1 for c in calls if c['kind'] == 'call'), resets=sum(1 for c in calls if c['kind'] == 'reset'),
                         first_green_s=green, last_progress=traj[-1]['progress_m'],
                         passed=report.get('passed'), detail_keys=sorted(keys))
    with (HERE / 'extract_index.json').open('w', encoding='utf-8', newline='\n') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: {x: y for x, y in v.items() if x not in ('files', 'detail_keys')} for k, v in out.items()}, indent=1))
    print(out['combined_failed']['detail_keys'])


if __name__ == '__main__':
    main()
