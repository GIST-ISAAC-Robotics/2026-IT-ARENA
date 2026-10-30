"""시험 구간의 원시 측정속도·선호 적용/해제·평행 후보 측정 검사를 판단 기록에서 분리한다.

평가 전용 사후 분석이다. 제어 입력이 아니며, 측정값은 모터축 환산값으로 실제 차체 속도와 같다고 보지 않는다.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import thaw


def main(run, output):
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    first = report['case_window']['green_sim_s']
    last = first+report['case_window']['duration_s']
    rows = []
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            data = thaw(event['data'])
            t = data['input']['reference_s']
            if not first <= t <= last:
                continue
            speed, steer, detail = data['output']
            keep = detail.get('rear_lane_keep') or {}
            observed = keep.get('observed_speed') or {}
            trial = detail.get('parallel_hold_trial') or {}
            rows.append(dict(
                reference_s=t, elapsed_s=t-first, measured_speed_mps=data['input']['speed_mps'],
                command=[speed, steer], mode=detail.get('selector_mode'), choice=detail.get('selector_choice'),
                selector_reason=detail.get('selector_reason'), lane_keep_active=keep.get('active_sides'),
                lane_keep_applied=bool(keep.get('applied')), lane_keep_reason=keep.get('reason'),
                observed_speed_reason=observed.get('reason'), observed_speed_applied=observed.get('applied'),
                parallel_trial_reason=trial.get('reason'),
                lane_keep_parallel_reason=keep.get('parallel_trial_reason'),
                parallel_measured_above=trial.get('measured_above_target_cap', False)))
    maximum = header['follow_config']['max_speed_mps']
    applied = [r for r in rows if r['lane_keep_applied']]
    endings = []
    for previous, current in zip(rows, rows[1:]):
        if previous['lane_keep_applied'] and not current['lane_keep_applied']:
            endings.append(dict(at=current, before=previous))
    above = [r for r in rows if r['measured_speed_mps'] > maximum]
    result = dict(
        scope='evaluation-only decision trace extraction; motor-shaft converted speed, not true body speed',
        policy=header['follow_config'].get('rear_lane_keep_policy'), max_speed_mps=maximum,
        window_s=[first, last], calls=len(rows),
        measured_speed_max_mps=max((r['measured_speed_mps'] for r in rows), default=None),
        lane_keep_applied_calls=len(applied),
        lane_keep_first_applied_s=applied[0]['reference_s'] if applied else None,
        lane_keep_last_applied_s=applied[-1]['reference_s'] if applied else None,
        lane_keep_applied_with_measured_above_max=sum(r['measured_speed_mps'] > maximum for r in applied),
        lane_keep_applied_max_measured_mps=max((r['measured_speed_mps'] for r in applied), default=None),
        preference_endings=endings[:10], preference_ending_count=len(endings),
        measured_above_max_calls=len(above),
        measured_above_max_lane_keep_reasons=dict(Counter(str(r['lane_keep_reason']) for r in above)),
        measured_above_max_parallel_reasons=dict(Counter(str(r['parallel_trial_reason']) for r in above)),
        measured_above_max_lane_keep_parallel_reasons=dict(Counter(str(r['lane_keep_parallel_reason']) for r in above)),
        lane_keep_parallel_reasons=dict(Counter(str(r['lane_keep_parallel_reason']) for r in rows)),
        lane_keep_reasons=dict(Counter(str(r['lane_keep_reason']) for r in rows)),
        observed_speed_reasons=dict(Counter(str(r['observed_speed_reason']) for r in rows)),
        parallel_trial_reasons=dict(Counter(str(r['parallel_trial_reason']) for r in rows)),
        selector_modes=dict(Counter(str(r['mode']) for r in rows)),
        input_sha256=hashlib.sha256((run/'follow_decisions.jsonl').read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    with output.with_suffix('.rows.jsonl').open('x', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False)+'\n')
    brief = {k: v for k, v in result.items() if k not in ('preference_endings',)}
    print(json.dumps(brief, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    raise SystemExit(main(args.run.resolve(), args.output.resolve()))
