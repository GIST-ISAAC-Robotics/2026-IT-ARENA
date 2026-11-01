"""실패 기록의 운동 이력만 대조한다. 정답 대입은 원인 진단이며 제어기 수정이 아니다."""
import argparse
import bisect
import json
from collections import Counter
from hashlib import sha256
from pathlib import Path

from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow

HERE = Path(__file__).resolve().parent
RUN = HERE.parent / 'low_speed_arc_reacquire_v1/run1'


class SoloRouteDiagnostic(LocalFollow):
    """저장 입력 대조만: 교통 선택기를 제외해도 경로·점유·부분 관측은 보존한다."""
    def _forward_command(self, scene, history, speed, side, previous_steering=0., control_dt=.02, *, raw_scan=None):
        return self._observed_command(scene, history, speed, side, previous_steering, control_dt, raw_scan=raw_scan)


def run(mode):
    source = RUN / 'follow_decisions.jsonl'
    trajectory = [json.loads(line) for line in (RUN / 'trajectory.jsonl').open()]
    truth = sorted({(r['dynamics']['sim_time_s'], r['dynamics']['truth_longitudinal_mps'])
                    for r in trajectory if r.get('dynamics')})
    times = [r[0] for r in truth]

    def interpolate(t):
        index = bisect.bisect_left(times, t)
        if index < len(times) and times[index] == t:
            return truth[index][1]
        if not 0 < index < len(times):
            raise ValueError('truth diagnostic would extrapolate')
        a, b = truth[index - 1:index + 1]
        return a[1] + (b[1] - a[1]) * (t - a[0]) / (b[0] - a[0])

    calls = mismatches = 0
    counts = Counter()
    rows = []
    last_source = None
    for line in source.open():
        event = json.loads(line)
        kind, data = event['kind'], event['data']
        if kind == 'header':
            motion = MotionConfig(**data['motion_config'])
            cls = SoloRouteDiagnostic if mode == 'solo_route' else LocalFollow
            follow = cls(FollowConfig(**data['follow_config']))
        elif kind == 'reset':
            follow.reset()
        elif kind == 'call':
            inputs = data['input']
            now = inputs['reference_s']
            wheels = inputs['history']['wheels']
            if mode == 'truth_history':
                wheels['values'] = [interpolate(t) if 44.6 <= t <= times[-1] else v
                                    for t, v in zip(wheels['times'], wheels['values'])]
            v, steer, detail = replay_call(follow, inputs, motion)
            calls += 1
            mismatches += abs(v - data['output'][0]) > 1e-9 or abs(steer - data['output'][1]) > 1e-9
            if now < 44.6:
                continue
            partial = detail.get('partial_observation') or {}
            counts[partial.get('reason')] += 1
            scan_source = inputs['scan']['sec'] + inputs['scan']['nanosec'] * 1e-9
            if scan_source == last_source:
                continue
            last_source = scan_source
            anchor = partial.get('anchor_diagnostic') or {}
            rows.append(dict(time_s=now, source_s=scan_source, recorded=data['output'][:2], output=[v, steer],
                reason=detail.get('reason'), partial_reason=partial.get('reason'),
                selector_reason=detail.get('selector_reason'), mode=detail.get('selector_mode'),
                unknown=partial.get('unknown_samples'), latch_time_s=partial.get('latch_time_s'),
                anchor_reason=anchor.get('reason'), fit_reason=(anchor.get('fit') or {}).get('reason'),
                measured_speed_mps=inputs['speed_mps'],
                truth_diagnostic_mps=interpolate(now) if times[0] <= now <= times[-1] else None,
                path_m=detail.get('observed_path_ahead_m'), clearance_m=detail.get('swept_clearance_m')))
    return dict(mode=mode, calls=calls, changed_outputs=mismatches, counts=dict(counts), rows=rows,
        truth_available_time_s=[times[0], times[-1]], truth_mutation_from_s=44.6,
        truth_outside_window='진단 표는 null, 운동 이력은 원래 값 유지. 외삽 없음.',
        trace_sha256=sha256(source.read_bytes()).hexdigest(),
        scope='저장 궤적에 고정된 진단. 실제 차량 이동/독립 보호/완주를 재생하지 않으며 정답 이력을 실사용하지 않는다.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('baseline', 'truth_history', 'solo_route'))
    args = parser.parse_args()
    value = run(args.mode)
    with (HERE / f'{args.mode}.json').open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
    print(json.dumps({k: v for k, v in value.items() if k != 'rows'}, ensure_ascii=False))
    if args.mode == 'baseline' and value['changed_outputs']:
        raise SystemExit(1)
