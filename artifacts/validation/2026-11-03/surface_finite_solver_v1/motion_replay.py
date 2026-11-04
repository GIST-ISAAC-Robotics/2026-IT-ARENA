"""이미 도착한 모터축 이력만 인과 필터링해 격리 판단 재생. 정답 입력 없음."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = ROOT/'build/surface_finite_solver_v1/original'
TRACES = {
    'failed_ray': ROOT/'artifacts/validation/2026-11-02/solo_ray_combination_v1/run1/follow_decisions.jsonl',
    'selected_solo': ROOT/'artifacts/validation/2026-11-03/solo_post_free_v1/run1/follow_decisions.jsonl',
    'irregular_bypass': ROOT/'artifacts/validation/2026-11-03/irregular_actor_v1/run1/follow_decisions.jsonl',
}


class CausalSeries:
    def __init__(self, mode):
        self.mode = mode
        self.raw = {}
        self.values = {}
        self.last = None

    def apply(self, times, values):
        output = []
        for stamp, value in zip(times, values):
            if stamp in self.raw:
                if value != self.raw[stamp]:
                    raise ValueError('recorded same timestamp has conflicting motor value')
            else:
                if self.last is None:
                    filtered = value
                else:
                    dt = stamp-self.last
                    if dt <= 0:
                        raise ValueError('unseen historical motor sample')
                    prior = self.values[self.last]
                    if self.mode == 'lowpass40ms':
                        filtered = prior+(-math.expm1(-dt/.04))*(value-prior)
                    elif self.mode == 'rate8':
                        filtered = prior+min(8*dt, max(-8*dt, value-prior))
                    else:
                        raise ValueError('unknown hypothesis')
                self.raw[stamp] = value
                self.values[stamp] = filtered
                self.last = stamp
            output.append(self.values[stamp])
        return output


def child(mode, label):
    sys.path.insert(0, str(BASE))
    import numpy as np
    from arena_autonomy.follow_trace import replay_call, freeze, equal_output
    from arena_autonomy.local_follow import FollowConfig, LocalFollow
    from arena_autonomy.lidar_motion import MotionConfig
    from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
    baseline = json.loads((HERE/f'{label}_original_decisions.json').read_bytes())
    rows, costs, footer, seq, series = [], [], None, 0, CausalSeries(mode)
    for line in TRACES[label].open(encoding='utf-8'):
        event = json.loads(line)
        if event['seq'] != seq or footer is not None:
            raise ValueError('trace ordering')
        seq += 1
        data = event['data']
        if event['kind'] == 'header':
            cfg, motion = FollowConfig(**data['follow_config']), MotionConfig(**data['motion_config'])
            control = (BypassHandoffFollow(cfg, HandoffExecutionConfig.from_mapping(data['execution_config']), allow_simulation_trial=True)
                       if 'execution_config' in data else LocalFollow(cfg))
        elif event['kind'] == 'reset':
            control.reset()
            series = CausalSeries(mode)
        elif event['kind'] == 'call':
            original = data['input']
            wheel = original['history']['wheels']
            filtered = series.apply(wheel['times'], wheel['values'])
            changed_input = {**original, 'history': {**original['history'], 'wheels': {**wheel, 'values': filtered}}}
            # 속도 명령/측정 속도·gyro·원시 LiDAR·출력 제한에는 손대지 않는다.
            began = time.perf_counter()
            actual = freeze(replay_call(control, changed_input, motion))
            costs.append((time.perf_counter()-began)*1000)
            d = actual[2]
            normal = d.get('normal')
            part = ((normal if isinstance(normal, dict) else d).get('partial_observation') or {})
            rows.append(dict(seq=event['seq'], t=original['reference_s'], dt=original['dt_s'], speed=actual[0], steer=actual[1],
                             state=d.get('handoff_state'), reason=d.get('reason'), partial_reason=part.get('reason')))
        elif event['kind'] == 'footer':
            footer = data
        else:
            raise ValueError('unknown event')
    if (not rows or footer is None or any(footer.get(k) != 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
            or footer.get('worker_alive') is not False or footer.get('written') != seq-1 or footer.get('accepted') != seq-1):
        raise ValueError('incomplete source')
    if len(rows) != len(baseline) or any(a['seq'] != b['seq'] for a, b in zip(rows, baseline)):
        raise ValueError('baseline event mismatch')
    paths = [Path(m.__file__).resolve() for n, m in sys.modules.items()
             if n.startswith(('arena_autonomy', 'arena_vehicle_interface')) and getattr(m, '__file__', None)]
    if not paths or not all(p.is_relative_to(BASE.resolve()) for p in paths):
        raise ValueError('mixed package sources')
    differences = [dict(seq=a['seq'], t=a['t'], original=[b['speed'], b['steer'], b['state']],
                        candidate=[a['speed'], a['steer'], a['state']], original_reason=b['reason'], reason=a['reason'])
                   for a, b in zip(rows, baseline)
                   if not equal_output([a['speed'], a['steer'], a['state']], [b['speed'], b['steer'], b['state']])]
    result = dict(mode=mode, trace=label, calls=len(rows), source_motor_samples=len(series.raw),
                  changed_controls=len(differences), first_differences=differences[:10],
                  new_zero_calls=sum(a['speed'] <= 1e-9 < b['speed'] for a, b in zip(rows, baseline)),
                  released_zero_calls=sum(b['speed'] <= 1e-9 < a['speed'] for a, b in zip(rows, baseline)),
                  maximum_steering_difference_rad=max(abs(a['steer']-b['steer']) for a, b in zip(rows, baseline)),
                  zero_command_sample_time_s=sum(a['dt'] for a in rows if a['speed'] <= 1e-9),
                  median_cost_ms=float(np.median(costs)), p95_cost_ms=float(np.percentile(costs, 95)),
                  scope='인과 모터 이력 가설의 판단 재생. 정답/미래 정보 없음. 실제 궤적·통합 보호층·실물 증명 아님. 필터 상수 미채택.')
    (HERE/f'{label}_{mode}_motion_decisions.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf-8')
    (HERE/f'{label}_{mode}_motion.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))


def self_check():
    for mode in ('lowpass40ms', 'rate8'):
        one, incremental = CausalSeries(mode), CausalSeries(mode)
        t, v = [0., .01, .02, .03], [1., 1., -1., 1.]
        expected = one.apply(t, v)
        assert incremental.apply(t[:3], v[:3]) == expected[:3]
        assert incremental.apply(t[1:], v[1:]) == expected[1:]
        assert expected[:2] == [1., 1.]
        negative = CausalSeries(mode).apply(t, [-1.]*4)
        assert negative == [-1.]*4  # 유효한 지속 역주행을 0으로 바꾸지 않는다.


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--child', nargs=2)
    args = parser.parse_args()
    self_check()
    if args.child:
        child(*args.child)
    else:
        for label in TRACES:
            for mode in ('lowpass40ms', 'rate8'):
                with (HERE/f'{label}_{mode}_motion.log').open('x') as log:
                    result = subprocess.run([sys.executable, '-B', str(Path(__file__).resolve()), '--child', mode, label],
                                            stdout=log, stderr=subprocess.STDOUT, cwd=ROOT)
                (HERE/f'{label}_{mode}_motion.exit').write_text(f'ACTUAL_EXIT_CODE={result.returncode}\n', encoding='utf-8')
                if result.returncode:
                    raise SystemExit(result.returncode)
