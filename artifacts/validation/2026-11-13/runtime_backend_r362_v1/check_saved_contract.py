"""저장 r353 계산 결과로 이관 전후 발행 계약을 대조한다. 계산/ROS/장치는 실행하지 않는다."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'),
                str(A.parent/'current_ros_bridge_r336_v1'),
                str(A.parent/'current_deceleration_r334_v1')]
from deceleration import CurrentDecelerationProcess
from ros_delivery import Delivery
import candidate_policy
from arena_autonomy.decision_delivery import source_identity
from arena_autonomy.runtime_dual_state import RuntimeDualProcess
from arena_autonomy.runtime_dual_delivery import RuntimeDualDelivery


def make(case, new):
    d = copy.deepcopy(case)
    current = d['current_rebase']
    plan = dict(stage='plan', generation=0, sequence=1, input=d['input'], output=d['output'])
    metadata = dict(revision=8, retained=bool(current.get('compute', {}).get('route_check', {}).get('retained')))
    plan['route_check'] = copy.deepcopy(metadata)
    row = dict(stage='current', generation=0, sequence=2, input=current['input'],
               output=current['output'], plan=plan, route_check=metadata)
    cls = RuntimeDualProcess if new else CurrentDecelerationProcess
    worker = cls.__new__(cls)
    worker.ready = row
    worker.retained = worker.saved_plan = worker.current_input = worker.pending_speed_cap = worker.fault = None
    worker.current = SimpleNamespace(inflight=None)
    worker.revision, worker.generation = 8, 0
    worker.geometry_age_limit = .5
    worker.last_application = -math.inf
    worker.supported = candidate_policy.supported
    worker.collect = lambda: []
    now = current['input']['reference_s']+.001
    delivery = (RuntimeDualDelivery if new else Delivery)(worker, lambda: now)
    state = dict(now_s=now, previous_steering_rad=current['input']['previous_steering_rad'],
                 side=current['input']['side'], motion_epoch=current['input']['epoch'],
                 sensor_epoch=source_identity(current['input'])[0])
    return delivery, worker, state


def exercise(case, new, scenario):
    delivery, worker, state = make(case, new)
    if scenario == 'expired':
        state['now_s'] += .3
    elif scenario == 'motion_epoch':
        state['motion_epoch'] += 1
    elif scenario == 'sensor_epoch':
        state['sensor_epoch'] += 1
    elif scenario == 'clock_regressed':
        state['now_s'] = case['current_rebase']['input']['reference_s']-.001
    result = delivery.poll(**state)
    receipt = None
    if delivery.pending_publication is not None:
        selected = result['output']
        args = dict(published=scenario != 'unpublished', reference_s=result['command_reference_s'],
                    speed_mps=selected[0], steering_rad=selected[1], now_s=state['now_s'])
        if scenario == 'lower_speed':
            args['speed_mps'] *= .5
        elif scenario == 'changed_steering':
            args['steering_rad'] += .001
        elif scenario == 'late_ack':
            args['now_s'] += .3
        elif scenario == 'stop_before_ack':
            delivery.invalidate()
        receipt = delivery.acknowledge_publication(**args)
    return dict(result=result, receipt=receipt, ready=worker.ready, retained=worker.retained,
                pending=delivery.pending_publication, counts=delivery.counts,
                generation=worker.generation, revision=worker.revision,
                last_application=delivery.last_application)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace', type=Path, default=ROOT/'artifacts/validation/2026-11-13/final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl')
    p.add_argument('--limit', type=int, default=12)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not 1 <= args.limit <= 32:
        p.error('limit outside 1..32')
    cases = []
    with args.trace.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'call' and row['data'].get('current_rebase'):
                cases.append(row['data'])
                if len(cases) >= args.limit:
                    break
    if not cases:
        raise RuntimeError('no recorded current-check outputs')
    scenarios = ('normal', 'expired', 'motion_epoch', 'sensor_epoch', 'clock_regressed',
                 'unpublished', 'lower_speed', 'changed_steering', 'late_ack', 'stop_before_ack')
    mismatches = []
    candidates = 0
    for i, case in enumerate(cases):
        for scenario in scenarios:
            old = exercise(case, False, scenario)
            new = exercise(case, True, scenario)
            candidates += old['pending'] is not None or old['receipt'] is not None
            if old != new:
                mismatches.append(dict(case=i, scenario=scenario))
    report = dict(passed=not mismatches, recorded_cases=len(cases), comparisons=len(cases)*len(scenarios),
                  candidate_receipts=candidates, mismatches=mismatches,
                  trace_sha256=hashlib.file_digest(args.trace.open('rb'), 'sha256').hexdigest(),
                  scope='recorded plan/current outputs through old/new delivery only; no planner, timing, ROS or hardware run')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
