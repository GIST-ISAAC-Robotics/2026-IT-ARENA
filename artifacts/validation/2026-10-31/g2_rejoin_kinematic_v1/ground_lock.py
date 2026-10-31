"""복귀 시작부터4초 차체 진행이 막힌 별도 합성 조건. run1과 원본은 보존한다."""
import inspect
import hashlib,json,zipfile
from pathlib import Path
import probe as base

source=inspect.getsource(base.run_case)
needle='ground_speed=command_speed*(.2 if slip and 1.<=t<5. else 1.)'
assert source.count(needle)==1
source=source.replace(needle,'ground_speed=command_speed*(0. if slip and 0.<=t<4. else 1.)')
namespace=dict(vars(base));exec(compile(source,'ground_lock_component','exec'),namespace)

if __name__=='__main__':
    output=base.BASE/'run2';output.mkdir(exist_ok=False)
    files=[Path(__file__),base.BASE/'probe.py',base.source,*(base.ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')]
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();hashes={p.relative_to(base.ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(base.ROOT/p,p)
    results={};rows={}
    for enabled in (False,True):
        name='ground_lock_'+('refresh' if enabled else 'original')
        results[name],rows[name]=namespace['run_case'](enabled,True)
    immutable=all(sha(base.ROOT/p)==v for p,v in hashes.items())
    result=dict(results=results,source_unchanged=immutable,source_sha256=hashes,
        scope='additional component: 0..4s no ground progress with wheel rotation; then ideal grip, not real friction')
    (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    (output/'trajectories.json').write_text(json.dumps(rows,ensure_ascii=False))
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'}))
    raise SystemExit(0 if immutable else 1)
