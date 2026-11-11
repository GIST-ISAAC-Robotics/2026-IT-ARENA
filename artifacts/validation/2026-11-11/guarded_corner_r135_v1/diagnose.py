"""r133 정체의 실제 GUARD 검사 좌표를 소스 변경 없이 저장 입력에서 수집한다."""
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.pass_release_trial import PassReleaseSelector

A = Path(__file__).resolve().parent
TRACE = A.parent / 'parallel_rebase_r133_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    digest = hashlib.sha256(TRACE.read_bytes()).hexdigest()
    events = [json.loads(line) for line in TRACE.open(encoding='utf8')]
    owner, motion = make_controller(events[0]['data'])
    captures = []; summaries = []; mismatches = []; calls = 0
    def local(frame, event, arg):
        if event != 'return':
            return local
        f = frame.f_locals
        context = f.get('context') or {}
        scene = f['scene']
        if 'queries' not in context:
            return local
        q, free, masked = (context[k] for k in ('queries', 'free', 'masked'))
        unobserved = ~free & ~masked
        rays = [freeze(asdict(r)) for r in scene.rays]
        sample = dict(reference_s=scene.t_ref_s, source_s=scene.source_first_s,
            baseline=freeze(f['baseline']), returned=freeze(arg), speed=f['speed'],
            previous=f['previous'], dt=f['dt'], age=f['age'],
            partial=freeze(f['baseline'][2].get('partial_observation')),
            region=freeze(f.get('region')), sides=freeze(f['self'].sides),
            ray_status=dict(Counter(r['status'] for r in rays)),
            unknown_rays=[r for r in rays if r['status'] == 'unknown'],
            bounds_unknown=np.vstack((q[unobserved].min(axis=0), q[unobserved].max(axis=0))).tolist()
                if unobserved.any() else None,
            follow_config=freeze(asdict(f['self'].follow_config)))
        np.savez_compressed(A / f'geometry_{len(captures)}.npz', queries=q, free=free,
            masked=masked, padding=context['padding'], points=scene.occupied_endpoints_xy_m,
            rays=np.array([json.dumps(r) for r in rays]))
        captures.append(sample)
        return local
    def enter(frame, event, arg):
        if event == 'call' and frame.f_code is PassReleaseSelector._guard_curve_at_cap.__code__:
            return local
        return None
    for row in events[1:]:
        if row['kind'] == 'reset':
            owner.reset(); continue
        if row['kind'] != 'call':
            continue
        calls += 1
        selected = row['seq'] == 790
        if selected: sys.settrace(enter)
        try:
            actual = freeze(replay_call(owner, row['data']['input'], motion))
        finally:
            sys.settrace(None)
        if not equal_output(comparable_decision(actual), comparable_decision(row['data']['output'])):
            mismatches.append(row['seq'])
        detail = actual[2].get('normal') or {}
        if actual[2].get('mode') == 'FOLLOW':
            summaries.append(dict(seq=row['seq'], t=row['data']['input']['reference_s'],
                speed=actual[0], reason=detail.get('reason'), mode=detail.get('selector_mode'),
                choice=detail.get('selector_choice'), selector_reason=detail.get('selector_reason'),
                sides=detail.get('selector_sides'),
                partial=detail.get('partial_observation')))
    result = dict(calls=calls, mismatch_sequences=mismatches, captures=captures,
        output_groups=dict(Counter(str((s['mode'], s['choice'], s['reason'])) for s in summaries)),
        trace_sha256=digest, trace_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest()==digest)
    (A/'result.json').write_text(json.dumps(result, indent=2), encoding='utf8')
    (A/'states.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in summaries), encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k != 'captures'}))
    return int(bool(mismatches) or not result['trace_unchanged'])


if __name__ == '__main__': raise SystemExit(main())
