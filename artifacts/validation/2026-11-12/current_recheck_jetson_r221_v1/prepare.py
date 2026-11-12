from pathlib import Path
import hashlib,json,tarfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
trace=A.parent/'expiry_null_runtime_r217_v1/two_actor_run1/run1/follow_decisions.jsonl'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
events=[json.loads(line) for line in trace.open()]
with (A/'samples.jsonl').open('x') as out:
    out.write(json.dumps(dict(header=events[0]['data'],source_trace_sha256=sha(trace)))+'\n')
    count=0
    for event in events:
        if event['kind']!='call' or 'current_rebase' not in event['data']:continue
        d=event['data'];row=dict(seq=event['seq'],original=dict(input=d['input'],output=d['output']),
                               current=d['current_rebase']['input'],expected=d['current_rebase']['output'])
        out.write(json.dumps(row,separators=(',',':'))+'\n');count+=1
assert count>100
files=[p for p in (ROOT/'src/arena_autonomy/arena_autonomy').rglob('*.py')]
files += [p for p in (ROOT/'src/arena_vehicle_interface/arena_vehicle_interface').rglob('*.py')]
files += [A/'samples.jsonl',A/'profile.py']
manifest={p.relative_to(ROOT).as_posix():dict(size=p.stat().st_size,sha256=sha(p)) for p in files}
bundle=ROOT/'build/current_recheck_r221_20261112.tar.gz'
with tarfile.open(bundle,'x:gz') as archive:
    for p in files:archive.add(p,arcname=p.relative_to(ROOT).as_posix())
with (A/'bundle_manifest.json').open('x') as out:
    json.dump(dict(archive_sha256=sha(bundle),files=manifest,source_trace_sha256=sha(trace),samples=count),out,indent=2);out.write('\n')
print(json.dumps(dict(samples=count,files=len(files),bytes=bundle.stat().st_size,sha256=sha(bundle))))
