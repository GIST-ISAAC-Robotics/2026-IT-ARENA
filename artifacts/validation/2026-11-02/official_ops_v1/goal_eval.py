"""대표 운영 흐름 사후 평가. 정답 자세·중심선은 여기서만 쓴다. 인자: 실행 폴더 [출력 JSON]."""
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official'
MIN_PROGRESS_LEFT = 4.
RIGHT_TARGET_S = 38.5
MAX_PREPARE_S = 3.


def load(run):
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    poses = np.array([json.loads(line)[:4] for line in (run/'ego_poses.jsonl').read_text().splitlines()])
    history = json.loads((run/'ops_history.json').read_text(encoding='utf-8'))
    log = [json.loads(line) for line in (run/'ops_log.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
    return report, poses, history, log


class Course:
    def __init__(self):
        route = np.genfromtxt(WORLD/'centerline.csv', delimiter=',', names=True)
        self.xy, self.s = np.column_stack((route['x_m'], route['y_m'])), route['s_m']
        self.branch = np.genfromtxt(WORLD/'branch_1.csv', delimiter=',', skip_header=1, usecols=(0, 1))

    def segment(self, poses):
        distance = np.linalg.norm(poses[:, None, 1:3]-self.xy[None], axis=2)
        index = np.argmin(distance, axis=1)
        branch = np.min(np.linalg.norm(poses[:, None, 1:3]-self.branch[None], axis=2), axis=1)
        return dict(s=self.s[index], error=distance[np.arange(len(poses)), index], in_branch=branch+.05 < distance.min(axis=1))


def steps_named(steps, name):
    return [row for row in steps if row['step'] == name]


def window(poses, start, end=None):
    mask = poses[:, 0] >= start
    if end is not None:
        mask &= poses[:, 0] <= end
    return poses[mask]


def moved(poses):
    return 0. if len(poses) < 2 else float(np.max(np.linalg.norm(poses[:, 1:3]-poses[0, 1:3], axis=1)))


def drive_segment(course, poses, begin, end_row):
    part = window(poses, begin, end_row['truth'][0])
    measure = course.segment(part)
    return dict(duration_s=float(part[-1, 0]-part[0, 0]), start_s=float(measure['s'][0]), end_s=float(measure['s'][-1]),
                progress_m=float(sum(max(0., d) if d < 1. else 0. for d in np.diff(measure['s']))),
                in_branch_samples=int(measure['in_branch'].sum()), max_center_error_m=float(measure['error'].max()),
                top_speed_mps=float(np.max(np.linalg.norm(np.diff(part[:, 1:3], axis=0), axis=1)/
                                           np.maximum(1e-4, np.diff(part[:, 0])))) if len(part) > 2 else 0.)


def history_between(history, begin, end):
    return [row for row in history if begin <= row['wall'] <= end]


def boundary_checks(run, report):
    """START 경계 뒤 판단이 쓴 스캔은 모두 경계 뒤 취득이어야 한다. 판단 기록은 기존 기록을 재사용한다."""
    boundaries = sorted({row['last']['boundary_s'] for row in steps_named(report['steps'], 'resume_closed')})
    violations, checked = 0, 0
    for name in ('follow_decisions.jsonl', 'follow_decisions_restart.jsonl'):
        path = run/name
        if not path.exists():
            continue
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if event.get('kind') != 'call':
                    continue
                source = event['data']['input']
                scan = source['scan']['sec']+source['scan']['nanosec']*1e-9
                reference = source.get('reference_s')
                if type(reference) not in (int, float):
                    continue
                for boundary in boundaries:
                    if boundary <= reference < boundary+2.:
                        checked += 1
                        violations += scan < boundary
    return dict(boundaries=boundaries, decisions_checked=checked, decisions_using_older_scan=violations)


def main():
    run = Path(sys.argv[1]).resolve()
    report, poses, history, log = load(run)
    steps = report['steps']
    course = Course()
    one = lambda name, index=0: steps_named(steps, name)[index]
    presses = steps_named(steps, 'press_resume_start')
    closes = steps_named(steps, 'resume_closed')
    placed = steps_named(steps, 'placed')
    gates = dict(run_completed=report.get('completed') is True, processes_clean=report.get('remaining_test_pids') == [],
                 source_unchanged=report.get('source_unchanged') is True,
                 gazebo_stop=(report.get('gazebo_stop_service') or {}).get('verified') is True)
    cli = one('first_start_cli_rejected')
    gates.update(first_start_refused_by_adapter=presses[0]['adapter_reason'] == 'race_not_started',
                 first_start_refused_by_supervisor=cli['code'] == 3 and 'race_not_started' in cli['text'],
                 no_motion_without_signal=one('no_motion_before_prepare')['truth_speed'] < .01
                                          and moved(window(poses, 0., one('armed_waiting_signal')['truth'][0])) < .02,
                 visual_start_source=one('visual_started')['source'] == 'visual')
    result = dict(gates=gates, stops=[dict(policy=(r['policy'] or {}).get('reason'), mcu=r['mcu'], physical=r['physical'],
                                           motor=r['motor'], stationary_ready=r['stationary_ready'])
                                      for r in steps_named(steps, 'stopped')])
    gates['stop_chain_observed'] = all(s['policy'] == 'stop_latched' and '확인' in (s['mcu'] or '') and s['physical'] == '미확인'
                                       for s in result['stops'])
    # 왼쪽: 운반 중 마커는 쓰지 않고, 배치 뒤 입력으로 다시 시작한다.
    left = closes[0]
    carry = one('carry_seen')
    left_place = placed[0]
    permit_row = one('left_permitted')
    gates.update(carry_saw_id30=30 in (carry['marker_ids'] or []), carry_did_not_change_side=carry['side'] == 'left',
                 left_side_retained=permit_row['side'] == 'left', left_permitted=left['last']['outcome'] == 'permitted',
                 left_prepare_short=left['sim_seconds'] <= MAX_PREPARE_S)
    left_drive = drive_segment(course, poses, permit_row['truth'][0], one('left_progress'))
    gates.update(left_no_motion_until_permission=moved(window(poses, left_place['truth'][0]+.3, left['truth'][0]-.25)) < .05,
                 left_main_progress=left_drive['progress_m'] >= MIN_PROGRESS_LEFT and left_drive['in_branch_samples'] == 0)
    # 오른쪽: 표지판이 보이지 않는 곳에서 재개한다.
    right = closes[1]
    right_start = one('start_response_dropped')
    right_end = one('right_progress')
    right_history = history_between(history, right_start['wall_s'], right_end['wall_s'])
    right_drive = drive_segment(course, poses, right_start['truth'][0], right_end)
    gates.update(id30_observed_before_stop=one('passed_id30_naturally')['marker_side'] == 30,
                 right_id30_not_visible=bool(right_history) and all(30 not in (r['driving'].get('marker_ids') or [])
                                                                     for r in right_history),
                 right_side_kept=all(r['driving'].get('side') == 'right' for r in right_history),
                 right_permitted=right['last']['outcome'] == 'permitted', right_prepare_short=right['sim_seconds'] <= MAX_PREPARE_S,
                 right_main_progress=right_drive['end_s'] >= RIGHT_TARGET_S and right_drive['in_branch_samples'] == 0,
                 cancelled_by_new_stop=one('cancelled_by_stop')['last']['outcome'] == 'cancelled',
                 no_motion_after_stop_cancel=one('still_held_after_cancel')['truth_speed'] < .02,
                 cancelled_by_link=one('cancelled_by_link')['last']['reason'] == 'operator_link_lost',
                 no_late_start_after_reconnect=one('no_late_start_after_reconnect')['truth_speed'] < .02,
                 duplicate_stop_held=one('cancelled_by_stop')['duplicate'] == 'queued')
    # 프로세스 재시작.
    restart = one('restarted')
    restart_end = one('restart_progress')
    restart_close = closes[2]
    restored = one('restored')
    restart_start = next(row for row in presses if row['wall_s'] > restored['wall_s'])
    restart_history = history_between(history, restart_start['wall_s'], restart_end['wall_s'])
    restart_drive = drive_segment(course, poses, restart_start['truth'][0], restart_end)
    gates.update(restart_new_session=restart['started'] is False and restart['offer'] is not None,
                 start_refused_before_restore=one('start_before_restore')['reason'] == 'race_not_started',
                 restore_needs_confirmation=one('restore_without_confirm')['reason'] == 'same_race_confirmation_required',
                 restored_by_operator=restored['source'] == 'restored_operator_confirmed' and restored['side'] == 'right',
                 restored_visual_not_claimed=restored['visual'] is False,
                 restored_held=one('hold_after_restore')['truth_speed'] < .02,
                 restart_permitted=restart_close['last']['outcome'] == 'permitted',
                 restart_id30_not_visible=bool(restart_history) and all(30 not in (r['driving'].get('marker_ids') or [])
                                                                         for r in restart_history),
                 restart_main_progress=restart_drive['end_s'] >= RIGHT_TARGET_S and restart_drive['in_branch_samples'] == 0)
    boundary = boundary_checks(run, report)
    last_sample = next((row for row in reversed(log) if row.get('row') == 'sample' and row.get('before_boundary')), {})
    gates['boundary_decisions_use_new_scans'] = boundary['decisions_checked'] > 0 and boundary['decisions_using_older_scan'] == 0
    result.update(left=dict(prepare=left['last'], sim_seconds=left['sim_seconds'], route=left['route'], drive=left_drive),
                  right=dict(prepare=right['last'], sim_seconds=right['sim_seconds'], route=right['route'], drive=right_drive),
                  restart=dict(prepare=restart_close['last'], sim_seconds=restart_close['sim_seconds'],
                               route=restart_close['route'], drive=restart_drive),
                  boundary=boundary, before_boundary_counts=last_sample.get('before_boundary'),
                  passed=all(gates.values()), failed=[k for k, v in gates.items() if not v],
                  scope='격리 Gazebo 단독 차량·평가기 재배치. 실차·공용 Wi-Fi·실제 MCU 정지 아님')
    text = json.dumps(result, ensure_ascii=False, indent=1)
    (Path(sys.argv[2]) if len(sys.argv) > 2 else run.parent/f'{run.name}_goal.json').write_text(text, encoding='utf-8', newline='\n')
    print(json.dumps(dict(passed=result['passed'], failed=result['failed']), ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
