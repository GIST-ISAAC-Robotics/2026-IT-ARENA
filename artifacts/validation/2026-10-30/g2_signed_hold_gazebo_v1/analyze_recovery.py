"""실제 활성 창과 STOP 이후의 내부 후보를 구별해 signed 대기 효과를 회수한다."""
from collections import Counter
import json
import math
from pathlib import Path

BASE = Path(__file__).resolve().parent
RUN = BASE/'run1'


def main():
    evaluation = json.loads((BASE/'run1_g2_evaluation.json').read_text())
    green, end = evaluation['green_sim_s'], evaluation['window_end_sim_s']
    calls = [e['data'] for line in (RUN/'g2_decisions.jsonl').open() if (e := json.loads(line))['kind'] == 'call']
    active = [d for d in calls if green <= d['input']['reference_s'] <= end]
    recovery = [dict(sim_s=d['input']['reference_s'], since_green_s=d['input']['reference_s']-green,
                     speed_command_mps=d['output'][0], confirmations=d['output'][2].get('forward_confirmations'))
                for d in active if d['output'][2].get('signed_hold_recovered')]
    negatives = [d for d in active if d['input']['speed_mps'] < 0]
    poses = [json.loads(line) for line in (RUN/'ego_poses.jsonl').open()]
    start = poses[0]
    c, s = math.cos(start[3]), math.sin(start[3])
    wall = []
    for t, x, y, yaw in poses:
        if not green <= t <= end:
            continue
        lateral = -(x-start[1])*s+(y-start[2])*c
        relative = yaw-start[3]
        gap = .42494-abs(lateral)-.1*abs(math.sin(relative))-.075*abs(math.cos(relative))
        wall.append((gap, t-green))
    dynamics = [r for line in (RUN/'braking_diagnostics.jsonl').open()
                if green <= (r := json.loads(line))['sim_time_s'] <= end]
    result = dict(scope='completed run audit; body truth only for evaluation', active_calls=len(active),
        reasons=dict(Counter(d['output'][2]['reason'] for d in active)),
        negative_feedback_calls=len(negatives), positive_command_during_negative_feedback=sum(d['output'][0] > 0 for d in negatives),
        first_negative_since_green_s=negatives[0]['input']['reference_s']-green if negatives else None,
        active_recoveries=recovery,
        post_window_recovery_candidates_not_actuated=sum(d['output'][2].get('signed_hold_recovered', False)
            for d in calls if d['input']['reference_s'] > end),
        minimum_wall_gap_m=min(wall)[0], minimum_wall_gap_since_green_s=min(wall)[1],
        minimum_left_joint_rad_s=min(r['left_speed_rad_s'] for r in dynamics),
        minimum_right_joint_rad_s=min(r['right_speed_rad_s'] for r in dynamics),
        first_pass_since_green_s=evaluation['pass_time_since_green_s'],
        centered_start_since_green_s=evaluation['center_in_window']['first_start_since_green_s'],
        centered_hold_since_green_s=evaluation['center_in_window']['hold_satisfied_since_green_s'],
        limits=['single run; does not establish repeatability or another vehicle scenario',
                'controller holds/reacquires geometry; wheel reversal and imperfect motor-shaft odometry still exist'])
    (BASE/'run1_recovery_analysis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
