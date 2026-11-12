"""r235 소스를 보존하고 완료 알림/GC 계측만 추가한 r236 비교본을 만든다."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
A = Path(__file__).resolve().parent
old = ROOT/'artifacts/validation/2026-11-12/two_stage_recheck_r235_v1'
s = (old/'probe.py').read_text(encoding='utf-8')

def change(before, after):
    global s
    assert s.count(before) == 1, before
    s = s.replace(before, after)

change('import csv\n', 'import csv\nimport gc\n')
s = s.replace('r235', 'r236').replace('domain197', 'domain196').replace("!= '197'", "!= '196'")
change('from two_stage_process import TwoStageProcess', 'from two_stage_process import TwoStageProcess\n    from completion_notice import CompletionNotice')
change("    assert args.label.isidentifier()", "    assert args.label.isidentifier()\n    assert args.mode == 'two_stage', 'this candidate tests completion notices only'")
change('    fault = None\n', '''    fault = None
    notice = None
    gc_started, gc_events = [], []
    def gc_trace(phase, info):
        if phase == 'start':
            gc_started.append((time.monotonic(), info['generation']))
        elif gc_started:
            began, generation = gc_started.pop()
            gc_events.append((began, time.monotonic(), generation))
    gc.callbacks.append(gc_trace)
''')
change('        ticks.append(time.monotonic())\n', '')
change('''        result = worker.pump()
        pump_ms''', '''        result = worker.pump()
        if notice is not None:
            notice.acknowledge()
            if notice.fault:
                raise RuntimeError(notice.fault)
        pump_ms''')
change('''    def tick():
        started = time.perf_counter()''', '''    def tick(kind='timer'):
        if kind == 'timer' and state['start'] is not None and not state['finished']:
            ticks.append(time.monotonic())
        started = time.perf_counter()''')
change("        rx.append((int(msg.angular_velocity.x), stamp, now))", "        rx.append((int(msg.angular_velocity.x), stamp, now, time.monotonic()))")
change('''    executor.add_node(node)
    started''', '''    executor.add_node(node)
    guard = node.create_guard_condition(lambda: tick('ready'))
    notice = CompletionNotice(worker.connection, guard.trigger)
    started''')
change('''        cleanup = worker.close()
        executor.remove_node(node)''', '''        notice_cleanup = notice.close()
        cleanup = worker.close()
        node.destroy_guard_condition(guard)
        gc.callbacks.remove(gc_trace)
        executor.remove_node(node)''')
change("        writer.writerow(['seq', 'stamp_ns', 'received_ns'])", "        writer.writerow(['seq', 'stamp_ns', 'received_ns', 'received_monotonic_s'])")
change("    report = dict(mode=args.mode,", '''    long_delays = []
    for row, delay in zip(rx, latencies):
        if delay < 30:
            continue
        receipt = row[3]
        overlaps = [(start, end, generation) for start, end, generation in gc_events
                    if start <= receipt and end >= receipt-delay/1000.]
        long_delays.append(dict(seq=row[0], delay_ms=delay, gc_overlap=overlaps))
    (target/'gc_events.json').write_text(json.dumps(gc_events, indent=2)+'\\n')
    (target/'long_delays.json').write_text(json.dumps(long_delays, indent=2)+'\\n')
    report = dict(completion_notice=notice_cleanup, gc_events=len(gc_events),
        gc_generation2_ms=[1000*(end-start) for start,end,generation in gc_events if generation==2],
        delays_30ms_or_more=len(long_delays), delays_30ms_without_gc=sum(not row['gc_overlap'] for row in long_delays),
        mode=args.mode,''')
change("not fault and len(answers)==len(calls)", "not fault and not notice_cleanup['alive'] and not notice_cleanup['fault'] and len(answers)==len(calls)")
(A/'probe.py').write_text(s, encoding='utf-8')
(A/'two_stage_process.py').write_bytes((old/'two_stage_process.py').read_bytes())
(A/'publisher_source.py').write_text((old/'publisher_source.py').read_text(encoding='utf-8').replace('r235','r236'), encoding='utf-8')
original = json.loads((old/'bundle_manifest.json').read_bytes())
paths = {name for name in original['files'] if name.startswith('src/')}
paths.update(str(path.relative_to(ROOT)).replace('\\','/') for path in A.glob('*.py'))
files = {name: dict(size=(ROOT/name).stat().st_size, sha256=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()) for name in sorted(paths)}
trace = original['trace']
assert hashlib.sha256((ROOT/trace).read_bytes()).hexdigest() == original['trace_sha256']
manifest = dict(trace=trace, trace_sha256=original['trace_sha256'],
    parent_manifest_sha256=hashlib.sha256((old/'bundle_manifest.json').read_bytes()).hexdigest(), files=files)
(A/'bundle_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
target = ROOT/'build/r236_bundle.zip'
with zipfile.ZipFile(target, 'w', zipfile.ZIP_DEFLATED) as bundle:
    for name in sorted(paths | {trace, str((A/'bundle_manifest.json').relative_to(ROOT)).replace('\\','/')}):
        bundle.write(ROOT/name, name)
result = dict(path='build/r236_bundle.zip', bytes=target.stat().st_size,
    sha256=hashlib.sha256(target.read_bytes()).hexdigest(), files=len(files), raw_input_git_excluded=True)
(A/'bundle_receipt.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
print(json.dumps(result))
