"""r328 저장 입력으로 신형 시설 대응 후보 묶음의 영향부터 분리한다."""
import argparse
import collections
import copy
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
PARENT = A.parent/'old610_drive_r328_v1'
sys.path[:0] = [str(PARENT), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, replay_reset, freeze, equal_output, comparable_decision
from arena_autonomy.numeric_kernels import configure


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('recorded', 'common'))
    args = parser.parse_args()
    if args.mode == 'recorded':
        import candidate
        candidate.install()
    path = PARENT/'run1/follow_decisions.jsonl'
    with path.open() as f:
        header = json.loads(next(f))['data']
        events = [json.loads(line) for line in f]
    configure(header.get('numeric_execution', 'python'))
    controller, motion = make_controller(copy.deepcopy(header))
    counts = collections.Counter()
    rows, timings = [], []
    differences = commands = 0
    for event in events:
        if event['kind'] == 'reset':
            replay_reset(controller, event['data'])
        if event['kind'] != 'call':
            continue
        call = event['data']
        begin = time.perf_counter()
        output = freeze(replay_call(controller, call['input'], motion))
        timings.append((time.perf_counter()-begin)*1000)
        differences += not equal_output(comparable_decision(output), comparable_decision(call['output']))
        changed = not equal_output(output[:2], call['output'][:2])
        commands += changed
        counts[output[2]['reason']] += 1
        counts['positive' if output[0] > 0 else 'zero'] += 1
        rows.append(dict(time=call['input']['reference_s'], speed=output[0], angle=output[1],
            original_speed=call['output'][0], command_changed=changed,
            reason=output[2]['reason'], boundary=output[2].get('boundary_candidate_reason'),
            boundary_selection=output[2].get('boundary_selection')))
    result = dict(mode=args.mode, calls=len(rows), output_differences=differences,
        command_differences=commands, counts=dict(counts),
        trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        cost_ms=dict(median=float(np.median(timings)), p95=float(np.percentile(timings, 95))),
        scope='fixed recorded trajectory/previous steering; not fresh driving')
    (A/(args.mode+'_rows.json')).write_text(json.dumps(rows)+'\n')
    (A/(args.mode+'_report.json')).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    if args.mode == 'recorded':
        assert differences == 0, 'frozen runtime must reproduce before interpreting comparison'


if __name__ == '__main__':
    main()
