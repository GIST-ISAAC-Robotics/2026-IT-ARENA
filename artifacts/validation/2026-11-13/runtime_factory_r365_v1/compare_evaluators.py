"""준비용: 원 r353 자식과 r365 factory의 제한 동일 입력 차등검증. 아직 실행하지 않았다."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'),
                str(A.parent/'final_geometry_crawl_drive_r353_v1'),
                str(A.parent.parent/'2026-11-12/current_region_runtime_r282_v1')]
import worker_candidate
import low_solo_worker
from factory_reference import evaluator_factory
from arena_autonomy.runtime_dual_process import Lane, plan_reference


def calculate(lane, data, generation, sequence, reason):
    lane.offer(data, generation, sequence, reason)
    until = time.monotonic()+5.
    while True:
        lane.connection.poll(.02)
        result = lane.collect()
        if result is not None:
            return result['output']
        if time.monotonic() >= until:
            raise RuntimeError('bounded evaluator comparison deadline')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace', type=Path, default=ROOT/'artifacts/validation/2026-11-13/final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl')
    p.add_argument('--limit', type=int, default=12)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not 1 <= args.limit <= 32:
        p.error('limit outside 1..32')
    calls, header, reset_reason = [], None, None
    with args.trace.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'header':
                header = row['data']
            elif row['kind'] == 'reset':
                reset_reason = row['data'].get('reason')
            elif row['kind'] == 'call':
                calls.append((row['data'], reset_reason))
                reset_reason = None
                if len(calls) >= args.limit:
                    break
    if not calls or header is None:
        raise RuntimeError('recorded header/calls required')
    header = {**header, 'numeric_execution': 'native_v1', 'r365_policy': 'r353'}
    low_solo_worker.install()
    from overlap import Lane as OriginalLane
    comparisons, mismatches, cleanup = [], [], []
    # 동시 자식은 최대2개다. 단계별로 회수/종료한 뒤 다음 단계를 만든다.
    for stage in ('plan', 'current'):
        old = OriginalLane(stage, header, capacity=1024*1024, timeout=5., test_floor_s=0.)
        new = None
        try:
            new = Lane(stage, header, capacity=1024*1024, timeout=5., evaluator_factory=evaluator_factory)
            for index, (row, reason) in enumerate(calls):
                if stage == 'current' and not row.get('current_rebase'):
                    continue
                data = row['input'] if stage == 'plan' else dict(output=row['output'],
                    submitted=plan_reference(row['input']), current=row['current_rebase']['input'],
                    retained=bool(row['current_rebase'].get('compute', {}).get('route_check', {}).get('retained')))
                generation = row.get('compute', {}).get('generation', 0)
                before = calculate(old, data, generation, index+1, reason)
                after = calculate(new, data, generation, index+1, reason)
                comparisons.append(dict(stage=stage, case=index, equal=before == after))
                if before != after:
                    mismatches.append(dict(stage=stage, case=index))
        finally:
            if new is not None:
                cleanup.append(dict(side='factory', **new.close()))
            cleanup.append(dict(side='original', **old.close()))
    passed = (not mismatches and any(r['stage'] == 'current' for r in comparisons)
              and all(not r['forced'] and r['exitcode'] == 0 for r in cleanup))
    report = dict(passed=passed, calls=len(calls), comparisons=comparisons, mismatches=mismatches,
                  cleanup=cleanup, trace_sha256=hashlib.sha256(args.trace.read_bytes()).hexdigest(),
                  scope='same bounded saved inputs through original and factory calculations; not original-run replay, ROS, timing or hardware proof')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report))
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
