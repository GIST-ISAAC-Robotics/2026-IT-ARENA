"""다른 제어 모드(M3)로 기록된 센서 입력에 현재 G1 운영 구성의 선택기 OFF/ON을 나란히 적용한다(개루프).

입력 차량 궤적은 기록 당시 제어기(M3)의 것이므로 G1 OFF/ON 어느 쪽의 폐루프 결과도 아니다. 같은 입력에서 선택기가
측방 위협을 어디서 촉발·해제하는지, OFF와 출력이 어디서 갈리는지만 본다. 정답 자세·actor 일정은 쓰지 않는다.
G1 구성은 `--config-from` 기록 헤더(run4 운영 구성)에서 가져오고, 운동 설정은 입력 기록의 헤더를 쓴다.
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
from arena_autonomy.follow_trace import restore_call  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable  # noqa: E402
from arena_autonomy.local_follow import FollowConfig, LocalFollow  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def first_line(path):
    with path.open(encoding='utf-8') as stream:
        return json.loads(stream.readline())


def run(controller, scene_args, data, scan):
    data_, history = scene_args
    try:
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        return controller.command(scene, history, data['speed_mps'], data['side'], data['previous_steering_rad'],
                                  data['dt_s'], raw_scan=scan)
    except (MotionUnavailable, ValueError) as error:
        controller.reset()
        return 0., 0., {'reason': 'follow_input_stop', 'input_error': str(error)}


def compare(path, config_path):
    before = digest(path)
    cfg = FollowConfig(**first_line(config_path)['data']['follow_config'])
    if not (cfg.parallel_hold_trial and cfg.pass_release_trial):
        raise ValueError('config source must be the selector ON operating configuration')
    off_cfg = replace(cfg, parallel_hold_trial=False, pass_release_trial=False)
    on, off, motion, rows = LocalFollow(cfg), LocalFollow(off_cfg), None, []
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                continue
            if event['kind'] == 'reset':
                on.reset()
                off.reset()
                rows.append({'seq': event['seq'], 'reset': event['data'].get('reason')})
                continue
            if event['kind'] != 'call':
                continue
            # 두 제어기에 같은 입력 객체를 쓰지 않도록 각각 복원한다.
            data, scan, history = restore_call(event['data']['input'], motion)
            a = run(off, (data, history), data, scan)
            data2, scan2, history2 = restore_call(event['data']['input'], motion)
            b = run(on, (data2, history2), data2, scan2)
            d = b[2]
            evidence = d.get('selector_evidence') or {}
            threats = {side: {k: evidence[side].get(k) for k in ('points', 'rearmost_ds_m', 'frontmost_ds_m')}
                       for side in ('left', 'right') if isinstance(evidence.get(side), dict) and evidence[side].get('present')}
            region = d.get('selector_region')
            rows.append({'seq': event['seq'], 't': data['reference_s'], 'recorded_speed': event['data']['output'][0],
                         'off_speed': a[0], 'off_steer': a[1], 'off_reason': a[2].get('reason'),
                         'on_speed': b[0], 'on_steer': b[1], 'on_reason': d.get('reason'),
                         'mode': d.get('selector_mode', d.get('selector_mode_before')), 'choice': d.get('selector_choice'),
                         'selector_reason': d.get('selector_reason'), 'threats': threats or None,
                         'sides': d.get('selector_sides'), 'parallel_reason': (d.get('parallel_hold_trial') or {}).get('reason'),
                         'region': region and {k: region.get(k) for k in ('unobserved', 'masked_left', 'masked_right',
                                                                          'masked_tail_extent_m', 'masked_front_samples')},
                         'command_equal': abs(a[0]-b[0]) <= 1e-12 and abs(a[1]-b[1]) <= 1e-12})
    calls = [r for r in rows if 'reset' not in r]
    transitions, last = [], None
    for r in calls:
        if r['mode'] != last:
            transitions.append({'t': r['t'], 'seq': r['seq'], 'mode': r['mode'], 'threats': r['threats'],
                                'sides': r['sides']})
            last = r['mode']
    blocked = [r for r in calls if r['off_speed'] > 0 and r['on_speed'] <= 0]
    streak, best, start = 0., (0., None, None), None
    for prev, r in zip(calls, calls[1:]):
        if r['off_speed'] > 0 and r['on_speed'] <= 0:
            start = start if start is not None else prev['t']
            if r['t']-start > best[0]:
                best = (r['t']-start, start, r['t'])
        else:
            start = None
    summary = {'scope': 'cross_mode_saved_input_open_loop; G1 OFF/ON on M3-driven inputs; not closed loop',
               'input': str(path.relative_to(ROOT)), 'sha256': before, 'input_unchanged': before == digest(path),
               'config_from': str(config_path.relative_to(ROOT)), 'script_sha256': digest(Path(__file__)),
               'calls': len(calls), 'resets': len(rows)-len(calls),
               'commands_equal': sum(r['command_equal'] for r in calls),
               'off_positive': sum(r['off_speed'] > 0 for r in calls), 'on_positive': sum(r['on_speed'] > 0 for r in calls),
               'off_positive_on_zero': len(blocked),
               'off_positive_on_zero_reasons': dict(Counter(r['selector_reason'] for r in blocked)),
               'longest_off_positive_on_zero_s': {'duration': best[0], 'from': best[1], 'to': best[2]},
               'modes': dict(Counter(r['mode'] for r in calls)), 'choices': dict(Counter(r['choice'] for r in calls)),
               'selector_reasons': dict(Counter(r['selector_reason'] for r in calls)),
               'side_threat_calls': sum(bool(r['threats']) for r in calls),
               'side_threat_times': [r['t'] for r in calls if r['threats']][:1] + [r['t'] for r in calls if r['threats']][-1:],
               'mode_transitions': transitions}
    return summary, rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('--config-from', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.jsonl').exists():
        raise FileExistsError(args.output)
    summary, rows = compare(args.input.resolve(), args.config_from.resolve())
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    with args.output.with_suffix('.jsonl').open('x', encoding='utf-8', newline='\n') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False)+'\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'mode_transitions'}, ensure_ascii=False))
    print('transitions', len(summary['mode_transitions']),
          json.dumps([{k: t[k] for k in ('t', 'mode', 'threats')} for t in summary['mode_transitions'][:12]],
                     ensure_ascii=False))
