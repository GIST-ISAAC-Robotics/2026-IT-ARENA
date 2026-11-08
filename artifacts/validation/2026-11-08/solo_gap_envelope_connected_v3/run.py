"""현행 r25 선택형 단독 후보를 원 r19과 같은 계측/물성으로 실행한다."""
import hashlib,inspect,json,os,signal,subprocess,sys
from pathlib import Path
import validate_basic_autonomy as target
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
r=json.loads((HERE/'registered.json').read_text());out=HERE/'run1'
assert not out.exists()
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in r['sources'].items())
original=target.start_demo_process;readers=[]
def instrumented(*args,**kwargs):
    bound=inspect.signature(original).bind(*args,**kwargs);bound.apply_defaults()
    bound.arguments['extra_arguments']=[*bound.arguments['extra_arguments'],'evaluation_wheel_contacts:=true']
    process=original(*bound.args,**bound.kwargs)
    for name,topic in (('wheel_contacts','/eval/wheels/contact'),('drivetrain_gz','/model/arena_car/drivetrain')):
        output=(out/f'{name}.jsonl').open('w');error=(out/f'{name}_reader.log').open('w')
        child=subprocess.Popen(['gz','topic','-e','-t',topic,'--json-output'],stdout=output,stderr=error,text=True,start_new_session=True)
        readers.append((name,topic,child,output,error))
    return process

def finish():
    summaries=[]
    for name,topic,child,output,error in readers:
        forced=[]
        for sig,seconds in ((signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,3)):
            if child.poll() is not None:break
            try:os.killpg(child.pid,sig);child.wait(timeout=seconds)
            except (ProcessLookupError,subprocess.TimeoutExpired):pass
            if sig!=signal.SIGINT:forced.append(sig.name)
        output.close();error.close();path=out/f'{name}.jsonl'
        digest=hashlib.sha256();lines=0;bad=0
        with path.open('rb') as f:
            for line in f:
                digest.update(line);lines+=1
                try:json.loads(line)
                except (ValueError,TypeError):bad+=1
        summaries.append(dict(name=name,topic=topic,pid=child.pid,exit=child.poll(),forced=forced,lines=lines,parse_errors=bad,bytes=path.stat().st_size,sha256=digest.hexdigest()))
    (HERE/'reader_summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
    changed=[p for p,h in r['sources'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
    (HERE/'postflight_integrity.json').write_text(json.dumps(dict(inputs=len(r['sources']),changed=changed),indent=2)+'\n')
target.start_demo_process=instrumented
sys.argv=r['args'];sys.argv[sys.argv.index('--output')+1]=str(out)
try:code=target.main()
finally:finish()
raise SystemExit(code)
