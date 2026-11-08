"""같은 기록을 현재 코드/미연결 사본으로 대조한다. 명령 차이 이후는 주행 증거가 아니다."""
from collections import Counter
from pathlib import Path
import hashlib
import importlib.util
import json
import sys
import time

from arena_autonomy.follow_trace import MotionConfig, FollowConfig, replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.local_follow import LocalFollow

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
manifest = json.loads((A/'stage_manifest.json').read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(ROOT/manifest['source']) == manifest['before_sha256']
assert sha(A/'local_follow_candidate.py') == manifest['after_sha256']
assert sha(A/'candidate.py') == manifest['helper_sha256']

def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m

module('arena_autonomy.corner_boundary_support', A/'candidate.py')
candidate = module('corner_support_staged_follow', A/'local_follow_candidate.py').LocalFollow._baseline_command
original = LocalFollow._baseline_command
trace = A.parent/'fragment_boundary_connected_v1/right_parallel_first_curve/run1/follow_decisions.jsonl'
counts = [Counter(), Counter()]
records = []
old_matches = resets = changed_commands = 0
first_detail = first_command = None
timings = [[], []]
started = time.perf_counter()
try:
    for line in trace.open(encoding='utf8'):
        e = json.loads(line); d = e['data']
        if e['kind'] == 'header':
            motion = MotionConfig(**d['motion_config'])
            cfg = FollowConfig(**d['follow_config'])
            ex = HandoffExecutionConfig.from_mapping(d['execution_config'])
            engines = [BypassHandoffFollow(cfg, ex, allow_simulation_trial=True) for _ in range(2)]
        elif e['kind'] == 'reset':
            for engine in engines: engine.reset()
            resets += 1
        elif e['kind'] == 'call':
            outputs = []
            for i, method in enumerate((original, candidate)):
                LocalFollow._baseline_command = method
                t0 = time.perf_counter()
                outputs.append(freeze(replay_call(engines[i], d['input'], motion)))
                timings[i].append((time.perf_counter()-t0)*1000)
            t = d['input']['reference_s']
            old, new = outputs
            old_matches += equal_output(comparable_decision(old), comparable_decision(d['output']))
            if first_detail is None and not equal_output(comparable_decision(old), comparable_decision(new)):
                first_detail = dict(t=t, old=old, new=new)
            if not equal_output(old[:2], new[:2]):
                changed_commands += 1
                if first_command is None: first_command = dict(t=t, old=old, new=new)
            rows = []
            for i, out in enumerate(outputs):
                detail = out[2].get('normal', out[2])
                counts[i][detail.get('reason', 'missing')] += 1
                counts[i]['zero_command'] += out[0] <= 0.
                rows.append(dict(speed=out[0], steering=out[1], reason=detail.get('reason'),
                    selection=detail.get('boundary_selection'), path_source=detail.get('path_source_s'),
                    support=detail.get('current_corner_support'),
                    selector_reason=detail.get('selector_reason'),
                    partial_reason=detail.get('partial_observation', {}).get('reason')))
            records.append(dict(t=t, source=d['input'].get('source_first_s'), old=rows[0], new=rows[1]))
finally:
    LocalFollow._baseline_command = original
import numpy as np
out = dict(scope='saved input with current baseline and staged corner support; after first changed command not a new trajectory',
    source_manifest=manifest, trace_sha256=sha(trace), calls=len(records), resets=resets,
    baseline_exact_historical_calls=old_matches, changed_commands=changed_commands,
    first_detail_difference=first_detail, first_command_difference=first_command,
    counts=[dict(c) for c in counts], wall_s=time.perf_counter()-started,
    cost_ms=[dict(median=float(np.median(t)), p95=float(np.percentile(t, 95)), maximum=max(t)) for t in timings])
(A/'stage_replay_result.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
(A/'stage_replay_rows.jsonl').write_text(''.join(json.dumps(r, ensure_ascii=False)+'\n' for r in records), encoding='utf8')
print(json.dumps(dict(calls=len(records), historical_matches=old_matches, changed_commands=changed_commands,
    first_detail=first_detail and first_detail['t'], first_command=first_command and first_command['t'],
    counts=out['counts'], cost_ms=out['cost_ms']), ensure_ascii=False), flush=True)
