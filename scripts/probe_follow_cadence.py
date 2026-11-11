#!/usr/bin/env python3
"""저장 센서 궤적에서 판단 호출 생략의 영향만 조사한다. ROS/명령 발행은 없다.

busy 모드는 처리 중 도착한 틱을 버리고 다음 기록 틱에서 시작한다.
비용은 같은 seq의 기존 Jetson 계측값을 사용하므로 상태 변경 뒤의 실제
계산 시간을 예측하지 않는다. 입력 속도/이전 조향도 원 기록 그대로다.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time

from arena_autonomy import follow_trace as ft
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


class Cadence:
    """새 틱을 큐에 쌓지 않는다. reset은 상태/진행 중 작업 경계도 분리한다."""
    def __init__(self, mode='busy', dt_mode='recorded'):
        if mode not in ('all', 'busy') or dt_mode not in ('recorded', 'elapsed'):
            raise ValueError('invalid cadence mode')
        self.mode, self.dt_mode = mode, dt_mode
        self.reset()

    def reset(self):
        self.busy_until = self.previous_start = self.previous_tick = None

    def offer(self, now, cost, dt):
        if any(isinstance(x, bool) or not isinstance(x, (int, float))
               or not math.isfinite(x) for x in (now, cost, dt)):
            raise ValueError('nonfinite cadence input')
        if now < 0 or cost < 0 or dt <= 0:
            raise ValueError('negative time or nonpositive dt')
        if self.previous_tick is not None and now < self.previous_tick:
            raise ValueError('time reversed without reset')
        self.previous_tick = now
        if self.mode == 'busy' and self.busy_until is not None and now < self.busy_until:
            return None
        selected_dt = (now-self.previous_start if self.dt_mode == 'elapsed'
                       and self.previous_start is not None else dt)
        self.previous_start = now
        self.busy_until = now + (cost if self.mode == 'busy' else 0.)
        return selected_dt, self.busy_until


def measured_costs(report):
    if not report.get('input_unchanged') or not report.get('source_unchanged'):
        raise ValueError('cost evidence changed')
    costs = {}
    for item in report['per_call_stages']:
        seq, total = item['seq'], 0.
        if seq in costs:
            raise ValueError('duplicate cost sequence')
        for name in ('motion_scene', 'follow_command'):
            stage = item['stages'][name]
            value = stage['wall_ms']
            if stage['calls'] != 1 or isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError('invalid measured stage')
            total += value / 1000.
        costs[seq] = total  # 오프라인 restore_input은 실시간 비용에서 제외한다.
    if len(costs) != report['calls']:
        raise ValueError('incomplete costs')
    return costs


def earliest_scan_stamp(data):
    scan = data['scan']
    if scan.get('representation') == 'native_measurements_v1':
        return scan['measurements']['acquired_at_s'][0]
    return scan['sec'] + scan['nanosec']*1e-9


def stats(values):
    if not values:
        return {'count': 0}
    import numpy as np
    return dict(count=len(values), minimum=min(values), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), maximum=max(values))


def run(trace, costs, cadence, row_sink):
    count = calls = skipped = 0
    footer = None
    follow = motion = None
    rows = []
    reasons, mismatches, command_mismatches = Counter(), [], []
    newly_zero, formerly_zero = [], []
    resets, seen_costs = [], set()
    for line in trace:
        event = json.loads(line)
        if footer is not None or event['seq'] != count:
            raise ValueError('trace gap, reorder, or trailing event')
        count += 1
        seq, kind, data = event['seq'], event['kind'], event['data']
        if count == 1:
            if not (kind == 'header' and data['schema'] == 5
                    and data['scope'] == 'g1_g2_handoff_decision_stage_only'
                    and data['controller_mode'] == 'g1_g2_handoff_trial'):
                raise ValueError('requires schema5 handoff trace')
            follow = BypassHandoffFollow(ft.FollowConfig(**data['follow_config']),
                HandoffExecutionConfig.from_mapping(data['execution_config']), allow_simulation_trial=True)
            motion = ft.MotionConfig(**data['motion_config'])
        elif kind == 'reset':
            follow.reset()
            cadence.reset()
            resets.append(dict(seq=seq, reason=data.get('reason')))
        elif kind == 'call':
            calls += 1
            if seq not in costs:
                raise ValueError('missing cost')
            seen_costs.add(seq)
            source = data['input']
            admitted = cadence.offer(source['reference_s'], costs[seq], source['dt_s'])
            if admitted is None:
                skipped += 1
                continue
            dt, done = admitted
            changed_input = dict(source, dt_s=dt)
            output = ft.comparable_decision(ft.freeze(ft.replay_call(follow, changed_input, motion)))
            expected = ft.comparable_decision(data['output'])
            if not ft.equal_output(output, expected):
                mismatches.append(seq)
            if not ft.equal_output(output[:2], expected[:2]):
                command_mismatches.append(seq)
            if output[0] == 0 and expected[0] > 0:
                newly_zero.append(seq)
            elif output[0] > 0 and expected[0] == 0:
                formerly_zero.append(seq)
            detail = output[2]
            normal = detail.get('normal', detail)
            reason = normal.get('selector_reason') or normal.get('reason')
            reasons[str(reason)] += 1
            row = dict(seq=seq, reference_s=source['reference_s'], simulated_finish_s=done,
                assigned_cost_s=costs[seq], dt_s=dt,
                oldest_age_at_start_s=source['reference_s']-earliest_scan_stamp(source),
                oldest_age_at_finish_s=done-earliest_scan_stamp(source),
                speed_mps=output[0], steering_rad=output[1],
                expected_speed_mps=expected[0], expected_steering_rad=expected[1],
                reason=reason, normal_reason=normal.get('reason'), outer_reason=detail.get('reason'),
                normal_speed_mps=normal.get('target_speed_before_hold_mps'),
                path_source_s=normal.get('path_source_s'), boundary_age_s=normal.get('boundary_age_s'),
                selector_choice=normal.get('selector_choice'),
                partial_reason=(normal.get('partial_observation') or {}).get('reason'),
                partial_detail=(normal.get('partial_observation') or {}).get('detail'),
                normal_keys=sorted(normal),
                state=detail.get('handoff_state'), input_error=detail.get('input_error'))
            rows.append(row)
            row_sink.write(json.dumps(row, allow_nan=False, separators=(',', ':'))+'\n')
        elif kind == 'footer':
            footer = data
        else:
            raise ValueError('unexpected trace event')
    if (footer is None or not rows or any(footer.get(k) != 0
            for k in ('capture_errors', 'failed', 'dropped', 'pending'))
            or footer.get('worker_alive') is not False
            or footer.get('accepted') != count-1 or footer.get('written') != count-1
            or seen_costs != costs.keys()):
        raise ValueError('incomplete trace/cost coverage')
    intervals = [b['reference_s']-a['reference_s'] for a,b in zip(rows, rows[1:])
                 if not any(a['seq'] < reset['seq'] < b['seq'] for reset in resets)]
    return dict(calls=calls, admitted=len(rows), skipped=skipped, resets=resets,
        full_mismatch_count=len(mismatches), full_mismatch_seqs=mismatches,
        command_mismatch_count=len(command_mismatches), command_mismatch_seqs=command_mismatches,
        newly_zero_command_seqs=newly_zero, formerly_zero_command_seqs=formerly_zero,
        reasons=reasons, decision_interval_s=stats(intervals),
        nominal_dt_s=stats([r['dt_s'] for r in rows]),
        dt_outside_current_0_to_02=sum(not 0 < r['dt_s'] <= .2 for r in rows),
        oldest_age_at_start_s=stats([r['oldest_age_at_start_s'] for r in rows]),
        oldest_age_at_finish_s=stats([r['oldest_age_at_finish_s'] for r in rows]))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace', type=Path, required=True)
    p.add_argument('--costs', type=Path, required=True)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--mode', choices=('all', 'busy'), default='busy')
    p.add_argument('--dt-mode', choices=('recorded', 'elapsed'), default='recorded')
    args = p.parse_args()
    root = Path(ft.__file__).resolve().parents[3]
    manifest = json.loads(args.manifest.read_bytes())
    unchanged = lambda: all(digest(root/name) == entry['sha256'] for name,entry in manifest['files'].items())
    if not unchanged():
        raise ValueError('bundle mismatch')
    costs_hash, trace_hash = digest(args.costs), digest(args.trace)
    costs = json.loads(args.costs.read_bytes())
    if costs['trace_sha256'] != trace_hash:
        raise ValueError('cost/trace mismatch')
    args.output.mkdir(parents=True, exist_ok=False)
    began = time.monotonic()
    with args.trace.open(encoding='utf8') as trace, (args.output/'admitted.jsonl').open('x', encoding='utf8') as rows:
        result = run(trace, measured_costs(costs), Cadence(args.mode, args.dt_mode), rows)
    result.update(mode=args.mode, dt_mode=args.dt_mode, trace_sha256=trace_hash,
        costs_sha256=costs_hash, script_sha256=digest(__file__),
        source_unchanged=unchanged(), inputs_unchanged=(costs_hash == digest(args.costs) and trace_hash == digest(args.trace)),
        elapsed_s=time.monotonic()-began,
        scope='open-loop saved-input subsequence; fixed per-seq measured cost; no ROS, output hold, STOP latency, or closed-loop proof',
        limitations=['input speed/previous steering/trajectory stay recorded',
            'cost comes from original full sequence and excludes ROS, projection and protection',
            'reset is an immediate trace barrier; no asynchronous result acceptance is implemented',
            'dt elapsed is uncapped diagnostic; current controller rejects dt greater than 0.2 s'])
    (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if not k.endswith('_seqs')}, allow_nan=False))


if __name__ == '__main__':
    main()
