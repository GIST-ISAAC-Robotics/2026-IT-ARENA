"""최신 공유 계산의 계획 병목 진단. 원 기록 순서/초기화를 보존하며 장치는 열지 않는다."""
from pathlib import Path
import cProfile,hashlib,importlib.util,json,platform,pstats,sys,time

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision
from arena_autonomy.numeric_kernels import configure

def main():
    out=A/'result';out.mkdir(exist_ok=False)
    trace=ROOT/'artifacts/validation/2026-11-13/final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl'
    original=A.parent/'shared_ray_timed_r358_v1/source_manifest.json'
    manifest=json.loads(original.read_bytes())
    def changed():return [n for n,h in manifest.items() if not (ROOT/n).is_file() or hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h]
    assert not changed()
    spec=importlib.util.spec_from_file_location('r359_boundary',A.parents[1]/'2026-11-12/terminal_boundary_r278_v1/candidate.py')
    boundary=importlib.util.module_from_spec(spec);spec.loader.exec_module(boundary)
    boundary.install();configure('native_v1')
    # 간격 표본과 상태 첫 등장. profile 표본을 원래 비용 순위로 골랐다고 주장하지 않는다.
    selection={0,50,100,150,200,250,300,350,398};seen=set();index=0
    with trace.open() as stream:
        for line in stream:
            r=json.loads(line)
            if r['kind']!='call':continue
            d=r['data']['output'][2];normal=d.get('normal',d)
            key=(d.get('handoff_state'),normal.get('selector_mode'),normal.get('reason'))
            if key not in seen:
                seen.add(key)
                if len(selection)<16:selection.add(index)
            index+=1
    rows=[];cases=[];diff=[];commands_differ=[];began=time.monotonic();calls=0
    try:
        with trace.open() as stream:
            header=json.loads(next(stream))['data'];follow,motion=make_controller(header)
            for line in stream:
                event=json.loads(line);kind=event['kind'];data=event['data']
                if kind=='reset':replay_reset(follow,data)
                elif kind=='call':
                    prof=cProfile.Profile() if calls in selection else None
                    if prof:prof.enable()
                    start=time.perf_counter()
                    try:actual=freeze(replay_call(follow,data['input'],motion))
                    finally:
                        elapsed=(time.perf_counter()-start)*1000
                        if prof:prof.disable()
                    expected=data['output'];same=equal_output(comparable_decision(actual),comparable_decision(expected))
                    keys=('mode','reason','handoff_state')
                    x=[*actual[:2],{k:actual[2].get(k) for k in keys}]
                    y=[*expected[:2],{k:expected[2].get(k) for k in keys}]
                    operational=equal_output(x,y)
                    if not same:diff.append(event['seq'])
                    if not operational:commands_differ.append(dict(seq=event['seq'],actual=x,expected=y))
                    rows.append(dict(index=calls,seq=event['seq'],profiled=bool(prof),elapsed_ms=elapsed,
                        speed=actual[0],steering=actual[1],state=actual[2].get('handoff_state'),
                        comparable_equal=same,operational_equal=operational))
                    if prof:
                        functions=[]
                        for (filename,lineno,name),(cc,nc,tt,ct,_) in sorted(pstats.Stats(prof).stats.items(),key=lambda p:p[1][3],reverse=True)[:35]:
                            functions.append(dict(file=filename.replace(str(ROOT)+'/',''),line=lineno,name=name,
                                calls=nc,self_ms=tt*1000,cumulative_ms=ct*1000))
                        cases.append(dict(index=calls,seq=event['seq'],reference_s=data['input']['reference_s'],
                            speed=actual[0],state=actual[2].get('handoff_state'),top=functions))
                    calls+=1
    finally:boundary.uninstall()
    report=dict(calls=calls,profiled=len(cases),selected=sorted(selection),comparable_differences=diff,
        command_differences=commands_differ,changed_sources=changed(),trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        elapsed_s=time.monotonic()-began,platform=platform.platform(),devices_opened=False,
        scope='latest native planner diagnostic; cProfile includes instrumentation overhead; serial replay, not runtime deadline proof')
    for name,value in [('report',report),('durations',rows),('functions',cases)]:
        (out/(name+'.json')).write_text(json.dumps(value,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    assert not report['changed_sources'] and not commands_differ

if __name__=='__main__':main()
