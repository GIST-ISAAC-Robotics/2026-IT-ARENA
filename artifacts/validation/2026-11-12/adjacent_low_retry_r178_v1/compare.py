"""r174 전체 기록 재현 뒤 r178 기존 저속 검사의 진입 순서를 비교한다. 새 주행 아님."""
import importlib.util
import json
import hashlib
import time
import sys
import platform
import numpy as np
from collections import Counter
from pathlib import Path
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output

A = Path(__file__).resolve().parent
TRACE = A.parent/'released_parallel_runtime_r174_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    spec = importlib.util.spec_from_file_location('r178', A/'candidate.py')
    candidate = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(candidate)
    candidate.prior.install()
    events = [json.loads(s) for s in TRACE.open()]
    digest = hashlib.sha256(TRACE.read_bytes()).hexdigest()
    mismatch = []
    changes = []
    counts = Counter()
    candidate_rows = []
    baseline_outputs = {}
    baseline_command_mismatches = []
    baseline_difference_paths = Counter()
    timing = {'baseline': [], 'candidate': []}
    def differences(left, right, prefix=''):
        if equal_output(left, right):
            return
        if isinstance(left, dict) and isinstance(right, dict):
            for key in left.keys() | right.keys():
                yield from differences(left.get(key), right.get(key), prefix+'.'+key)
        elif isinstance(left, list) and isinstance(right, list) and len(left)==len(right):
            for i, (a, b) in enumerate(zip(left, right)):
                yield from differences(a, b, prefix+f'[{i}]')
        else:
            yield prefix
    for label in ('baseline', 'candidate'):
        if label == 'candidate':
            candidate.install()
        owner, motion = make_controller(events[0]['data'])
        for event in events[1:-1]:
            if event['kind'] == 'reset':
                owner.reset()
            elif event['kind'] == 'call':
                started = time.perf_counter_ns()
                raw = replay_call(owner, event['data']['input'], motion)
                elapsed = (time.perf_counter_ns()-started)/1e6
                out = freeze(raw)
                timing[label].append(dict(seq=event['seq'], ms=elapsed, positive=out[0]>0))
                expected = event['data']['output']
                same = equal_output(comparable_decision(out), comparable_decision(expected))
                if label == 'baseline' and not same:
                    mismatch.append(event['seq'])
                    baseline_difference_paths.update(differences(comparable_decision(out), comparable_decision(expected)))
                if label == 'baseline':
                    baseline_outputs[event['seq']] = out
                    if not equal_output(out[:2], expected[:2]):
                        baseline_command_mismatches.append(event['seq'])
                if label == 'candidate':
                    expected = baseline_outputs[event['seq']]
                    same = equal_output(comparable_decision(out), comparable_decision(expected))
                    normal = out[2].get('normal', out[2])
                    proof = normal.get('partial_observation') or {}
                    row = dict(seq=event['seq'], time_s=event['data']['input']['reference_s'],
                        before=expected[:2], after=out[:2], reason=normal.get('reason'),
                        partial=proof.get('reason'), selector=normal.get('selector_reason'),
                        body=normal.get('body_side_gap_candidate'), guard=normal.get('selector_guard_candidate'), retry=normal.get('selector_adjacent_low_retry'))
                    candidate_rows.append(dict(seq=event['seq'], output=out))
                    if not same:
                        changes.append(row)
                    counts['stop_to_go'] += expected[0] <= 0 < out[0]
                    counts['go_to_stop'] += out[0] <= 0 < expected[0]
                    counts['low_retry_applied'] += (normal.get('selector_adjacent_low_retry') or {}).get('applied') is True
    candidate.uninstall()
    summary = dict(calls=len(candidate_rows), baseline_mismatches=mismatch,
        baseline_command_mismatches=baseline_command_mismatches,
        baseline_difference_paths=dict(baseline_difference_paths), candidate_comparison_basis='same_platform_baseline',
        counts=dict(counts), changed=len(changes), trace_sha256=digest,
        environment=dict(python=sys.version, numpy=np.__version__, machine=platform.machine()),
        timing_ms={label: dict(median=float(np.median([r['ms'] for r in values])),
            p95=float(np.percentile([r['ms'] for r in values], 95)),
            maximum=max(r['ms'] for r in values)) for label, values in timing.items()},
        trace_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest() == digest)
    (A/'comparison.json').write_text(json.dumps(dict(summary=summary, changes=changes), indent=2))
    (A/'expected.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in candidate_rows))
    (A/'timing.json').write_text(json.dumps(timing, indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k not in ('baseline_mismatches','baseline_difference_paths')}))
    assert not baseline_command_mismatches and summary['trace_unchanged']


if __name__ == '__main__':
    main()
