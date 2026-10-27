"""보존된 선택기 OFF 기록에 같은 G1 구성 + 평행/통과 선택기 ON을 적용한 비교(개루프, 동일 재생 아님).

입력 스캔·운동 이력은 원래 OFF 출력대로 움직인 차량의 것이다. 첫 명령 분기 전까지는 같은 입력에 대한 정확한 비교이고,
그 뒤의 차이는 '이 입력이었다면'의 참고값이다. 실제 ON 폐루프 결과를 뜻하지 않는다.

정답 자세는 쓰지 않는다. 기록마다 선택기 상태·선택·이유·측방 위협 증거만 모은다. 단독 랩은 다른 차량이 없으므로
측방 위협 판정은 모두 벽/코너 등 비차량 반환에 대한 오촉발이다. 앞차 정지/재개도 앞차가 같은 차로 정면에 있으므로
측방 위협은 기본적으로 오촉발 후보로 분류하고 개별 반환 근거를 남긴다.
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
for folder in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'):
    sys.path.insert(0, str(ROOT/folder))
from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, restore_call  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable  # noqa: E402
from arena_autonomy.local_follow import FollowConfig, LocalFollow  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def compare(path):
    before = digest(path)
    rows, ctl, motion, header = [], None, None, None
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                header = event['data']
                cfg = replace(FollowConfig(**header['follow_config']), parallel_hold_trial=True, pass_release_trial=True)
                motion, ctl = MotionConfig(**header['motion_config']), LocalFollow(cfg)
                continue
            if event['kind'] == 'reset':
                ctl.reset()
                rows.append({'seq': event['seq'], 'reset': True})
                continue
            if event['kind'] != 'call':
                continue
            data, scan, history = restore_call(event['data']['input'], motion)
            try:
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                new = ctl.command(scene, history, data['speed_mps'], data['side'], data['previous_steering_rad'],
                                  data['dt_s'], raw_scan=scan)
            except (MotionUnavailable, ValueError) as error:
                ctl.reset()
                new = (0., 0., {'reason': 'follow_input_stop', 'input_error': str(error)})
            old, d = event['data']['output'], new[2]
            evidence = d.get('selector_evidence') or {}
            threats = {side: {k: evidence[side].get(k) for k in ('present', 'points', 'rearmost_ds_m', 'frontmost_ds_m')}
                       for side in ('left', 'right') if isinstance(evidence.get(side), dict)
                       and evidence[side].get('present')}
            rows.append({'seq': event['seq'], 't': data['reference_s'], 'old_speed': old[0], 'old_steer': old[1],
                         'old_reason': old[2].get('reason'), 'speed': new[0], 'steer': new[1],
                         'reason': d.get('reason'), 'mode': d.get('selector_mode', d.get('selector_mode_before')),
                         'choice': d.get('selector_choice'), 'selector_reason': d.get('selector_reason'),
                         'command_equal': abs(new[0]-old[0]) <= 1e-12 and abs(new[1]-old[1]) <= 1e-12,
                         'baseline_detail_equal': d.get('selector_choice') == 'baseline' and equal_output(
                             comparable_decision([0, 0, {k: v for k, v in freeze(d).items() if not k.startswith('selector_')
                                                         and k != 'pass_release_trial'}])[2],
                             comparable_decision([0, 0, old[2]])[2]),
                         'threats': threats or None, 'parallel_reason': (d.get('parallel_hold_trial') or {}).get('reason')})
    calls = [r for r in rows if not r.get('reset')]
    first_diff = next((r for r in calls if not r['command_equal']), None)
    transitions, last = [], None
    for r in calls:
        if r['mode'] != last:
            transitions.append({'t': r['t'], 'seq': r['seq'], 'mode': r['mode'], 'threats': r['threats']})
            last = r['mode']
    threat_rows = [r for r in calls if r['threats']]
    summary = {'scope': 'fixed_saved_input_open_loop_not_identical_replay', 'input': str(path.relative_to(ROOT)),
               'sha256': before, 'input_unchanged': before == digest(path), 'calls': len(calls),
               'resets': len(rows)-len(calls),
               'header_follow_config_original': {k: header['follow_config'].get(k) for k in
                                                 ('partial_observation_policy', 'parallel_hold_trial', 'pass_release_trial',
                                                  'corridor_fallback_enabled', 'boundary_evidence_mode', 'occupancy_timing_mode')},
               'commands_equal': sum(r['command_equal'] for r in calls),
               'baseline_choice_detail_equal': sum(r['baseline_detail_equal'] for r in calls),
               'first_command_difference': first_diff and {k: first_diff[k] for k in
                                                           ('seq', 't', 'old_speed', 'speed', 'old_steer', 'steer',
                                                            'mode', 'choice', 'selector_reason', 'reason', 'old_reason')},
               'old_positive': sum(r['old_speed'] > 0 for r in calls), 'new_positive': sum(r['speed'] > 0 for r in calls),
               'modes': dict(Counter(r['mode'] for r in calls)), 'choices': dict(Counter(r['choice'] for r in calls)),
               'selector_reasons': dict(Counter(r['selector_reason'] for r in calls)),
               'side_threat_calls': len(threat_rows),
               'side_threat_first': threat_rows[0]['t'] if threat_rows else None,
               'mode_transitions': transitions}
    return summary, rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.jsonl').exists():
        raise FileExistsError(args.output)
    summary, rows = compare(args.input.resolve())
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    with args.output.with_suffix('.jsonl').open('x', encoding='utf-8', newline='\n') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False)+'\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'mode_transitions'}, ensure_ascii=False))
    print('transitions', len(summary['mode_transitions']), json.dumps(summary['mode_transitions'][:8], ensure_ascii=False))
