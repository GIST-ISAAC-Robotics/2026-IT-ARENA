"""실제 고유 plan 제출/reset의 r457 원 순서 재생. current/주행은 실행하지 않는다."""
from pathlib import Path
import collections,hashlib,json,math,os,sys,time

HERE=Path(__file__).resolve().parent
MODE=sys.argv[1]
assert MODE in ('baseline','candidate')
ROOT=HERE/'execution_v1'
PKG=ROOT/MODE
OUT=ROOT/(MODE+'_result')
OUT.mkdir(exist_ok=False)
sys.dont_write_bytecode=True
sys.path[:0]=[str(PKG/'src/arena_autonomy'),str(PKG/'src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((ROOT/'manifest.json').read_text())
assert all(sha(PKG/n)==h for n,h in manifest['files'][MODE].items())
trace=HERE.parent/'solo_full_lap_r457_v1/run1/follow_decisions.jsonl'
assert sha(trace)==manifest['trace_sha256']
rows=[json.loads(line) for line in trace.open(encoding='utf-8')]
headers=[r['data']['header'] for r in rows if r['kind']=='runtime_evaluator_header']
assert len(headers)==1
header=headers[0]
assert header['numeric_execution']=='native_v1'
submitted=[r['data'] for r in rows if r['kind']=='runtime_plan_submitted']
completed={(r['data']['generation'],r['data']['sequence']):r['data']
           for r in rows if r['kind']=='runtime_plan_completed'}
assert len(submitted)==len(completed)==405
assert len({(r['generation'],r['sequence']) for r in submitted})==405
if MODE=='candidate':
    previous=json.loads((ROOT/'baseline_result/summary.json').read_text())
    assert previous['passed'] and previous['replayed']==405 and previous['numeric_mismatches']==0

def differences(a,b,path='$',limit=12):
    if type(a) is float and type(b) is float:
        return [] if math.isclose(a,b,rel_tol=1e-12,abs_tol=1e-12) else [{'path':path,'expected':b,'actual':a}]
    if type(a) is not type(b):return [{'path':path,'expected_type':type(b).__name__,'actual_type':type(a).__name__}]
    if isinstance(a,dict):
        if a.keys()!=b.keys():return [{'path':path,'added':sorted(a.keys()-b.keys()),'removed':sorted(b.keys()-a.keys())}]
        out=[]
        for k in a:
            out.extend(differences(a[k],b[k],path+'.'+k,limit))
            if len(out)>=limit:return out[:limit]
        return out
    if isinstance(a,list):
        if len(a)!=len(b):return [{'path':path,'expected_length':len(b),'actual_length':len(a)}]
        out=[]
        for i,(x,y) in enumerate(zip(a,b)):
            out.extend(differences(x,y,path+'['+str(i)+']',limit))
            if len(out)>=limit:return out[:limit]
        return out
    return [] if a==b else [{'path':path,'expected':b,'actual':a}]

checks=[];costs=[];outputs=[];generation=None;reset_events=[];evaluator=None;error=None
immutable=hashlib.sha256()
started=time.monotonic()
try:
    evaluator=Evaluator('plan',header)
    for item in submitted:
        key=(item['generation'],item['sequence'])
        if generation!=key[0]:
            evaluator.reset(item['reset_reason']);generation=key[0]
            reset_events.append({'generation':generation,'sequence':key[1],'reason':item['reset_reason']})
        encoded=json.dumps(item['input'],sort_keys=True).encode()
        immutable.update(encoded)
        began=time.monotonic();actual=evaluator(item['input']);costs.append((time.monotonic()-began)*1000)
        assert encoded==json.dumps(item['input'],sort_keys=True).encode()
        expected=completed[key]['output']
        delta=differences(actual,expected)
        n,old=actual[2].get('normal',{}),expected[2].get('normal',{})
        check={'generation':key[0],'sequence':key[1],'reference_s':item['input']['reference_s'],
            'epoch':item['input']['epoch'],'measured_mps':item['input']['speed_mps'],
            'original_command':expected[:2],'actual_command':actual[:2],
            'bit_exact':actual==expected,'numeric_equal':not delta,'command_equal':not differences(actual[:2],expected[:2]),
            'path_equal':not differences(n.get('path_points'),old.get('path_points')),
            'path_wall_equal':n.get('path_wall')==old.get('path_wall'),
            'selector_sides_equal':n.get('selector_sides')==old.get('selector_sides'),
            'selector_mode_equal':n.get('selector_mode')==old.get('selector_mode'),
            'handoff_state_equal':actual[2].get('handoff_state')==expected[2].get('handoff_state'),
            'original_reason':old.get('reason'),'actual_reason':n.get('reason'),
            'original_fresh':old.get('fresh_boundary_accepted'),'actual_fresh':n.get('fresh_boundary_accepted'),
            'actual_agreement':n.get('boundary_history_agreement'),
            'original_source_s':old.get('path_source_s'),'actual_source_s':n.get('path_source_s'),
            'original_wall':old.get('path_wall'),'actual_wall':n.get('path_wall'),
            'original_selector_mode':old.get('selector_mode'),'actual_selector_mode':n.get('selector_mode'),
            'original_selector_sides':old.get('selector_sides'),'actual_selector_sides':n.get('selector_sides'),
            'differences':delta}
        checks.append(check)
        outputs.append({'generation':key[0],'sequence':key[1],'output':actual})
        if MODE=='baseline' and delta:
            error={'reason':'baseline_mismatch','key':list(key),'differences':delta}
            break
except BaseException as exc:
    error={'type':type(exc).__name__,'message':str(exc)}
    raise
finally:
    if evaluator is not None:evaluator.close()
    modules={n:str(Path(v.__file__).resolve()) for n,v in sys.modules.items()
        if n.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(v,'__file__',None)}
    outside={n:p for n,p in modules.items() if not Path(p).is_relative_to(PKG)}
    source_changes=[n for n,h in manifest['files'][MODE].items() if sha(PKG/n)!=h]
    import numpy as np
    summary={'mode':MODE,'pid':os.getpid(),'replayed':len(checks),'error':error,
        'numeric_mismatches':sum(not x['numeric_equal'] for x in checks),
        'bit_exact':sum(x['bit_exact'] for x in checks),
        'command_equal':sum(x['command_equal'] for x in checks),
        'positive_to_zero':[x['sequence'] for x in checks if x['original_command'][0]>0 and x['actual_command'][0]==0],
        'zero_to_positive':[x['sequence'] for x in checks if x['original_command'][0]==0 and x['actual_command'][0]>0],
        'path_changes':[x['sequence'] for x in checks if not x['path_equal']],
        'selected_wall_changes':[x['sequence'] for x in checks if not x['path_wall_equal']],
        'selector_side_changes':[x['sequence'] for x in checks if not x['selector_sides_equal']],
        'selector_mode_changes':[x['sequence'] for x in checks if not x['selector_mode_equal']],
        'handoff_state_changes':[x['sequence'] for x in checks if not x['handoff_state_equal']],
        'reset_events':reset_events,'input_epoch_counts':dict(collections.Counter(x['epoch'] for x in checks)),
        'signed_motion_policy':header['follow_config']['signed_motion_policy'],
        'time_policy':header.get('runtime_time_policy','verified_only'),
        'observation_scope':header.get('observation_input_scope'),
        'input_digest_before_after':immutable.hexdigest(),'input_unchanged':True,
        'trace_sha256':sha(trace),'trace_unchanged':sha(trace)==manifest['trace_sha256'],
        'source_changes':source_changes,'external_project_imports':outside,'modules':modules,
        'wall_elapsed_s':time.monotonic()-started,
        'evaluator_cost_ms':{'median':float(np.median(costs)),'p95':float(np.percentile(costs,95)),
                             'max':max(costs)} if costs else None,
        'scope':'WSL saved plan replay only; no current, drive, ROS, Jetson',
        'passed':error is None and len(checks)==405 and not outside and not source_changes}
    for name,value in [('summary.json',summary),('checks.json',checks),('outputs.json',outputs),
                       ('focus_641_645.json',[x for x in checks if x['sequence'] in (641,643,644,645)])]:
        (OUT/name).write_text(json.dumps(value,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('modules','reset_events')},ensure_ascii=False))
assert summary['passed']
