"""동결 r82의 동일 순서 판단을 유지하면서 일부 호출만 함수 계측한다."""
import argparse
import cProfile
import hashlib
import json
import platform
import pstats
import time
from pathlib import Path

from arena_autonomy import follow_trace as ft

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--trace', type=Path, required=True)
p.add_argument('--manifest', type=Path, required=True)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--stride', type=int, default=20)
a = p.parse_args()
assert a.stride > 0 and not a.out.exists()
root = Path(ft.__file__).resolve().parents[3]
manifest = json.loads(a.manifest.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert all(sha(root/n) == v['sha256'] for n, v in manifest['files'].items())
before = sha(a.trace)
original = ft.replay_call
profiler = cProfile.Profile()
calls = 0
selected = 0
event = None
command_diffs = []

def call(*args, **kwargs):
    global calls, selected
    calls += 1
    sampled = calls % a.stride == 0
    if sampled:
        selected += 1
        profiler.enable()
    try:
        output = original(*args, **kwargs)
    finally:
        if sampled:
            profiler.disable()
    actual = ft.comparable_decision(ft.freeze(output))
    expected = ft.comparable_decision(event['data']['output'])
    if not ft.equal_output(actual[:2], expected[:2]):
        command_diffs.append(event['seq'])
    return output

def events():
    global event
    for line in a.trace.open(encoding='utf-8'):
        event = json.loads(line)
        yield event

started = time.monotonic()
ft.replay_call = call
try:
    result = ft.replay_events(events())
finally:
    ft.replay_call = original
stats = pstats.Stats(profiler)
rows = []
for key, values in stats.stats.items():
    cc, nc, tt, ct, callers = values
    rows.append(dict(file=key[0], line=key[1], function=key[2],
                     primitive_calls=cc, calls=nc, self_s=tt, cumulative_s=ct,
                     callers=[dict(file=k[0], line=k[1], function=k[2], stats=list(v))
                              for k, v in callers.items()]))
report = dict(scope='sampled function profile; overhead included; not uninstrumented latency or ROS deadline',
    calls=result['calls'], profiled_calls=selected, stride=a.stride,
    full_outputs_matched=result['matched'], all_outputs_matched=result['passed'],
    commands_matched=not command_diffs, command_difference_seqs=command_diffs,
    trace_sha256=before, input_unchanged=sha(a.trace) == before,
    source_unchanged=all(sha(root/n) == v['sha256'] for n, v in manifest['files'].items()),
    elapsed_s=time.monotonic()-started, machine=platform.machine(), python=platform.python_version(),
    top_self=sorted(rows, key=lambda r:r['self_s'], reverse=True)[:50],
    top_cumulative=sorted(rows, key=lambda r:r['cumulative_s'], reverse=True)[:50])
a.out.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
profiler.dump_stats(str(a.out.with_suffix('.pstats')))
print(json.dumps({k:v for k,v in report.items() if not k.startswith('top_')}))
