"""r186 시험 prefix의 None 경로 예외를 재현하고 가드 한 줄 후보를 대조한다."""
from pathlib import Path
import hashlib,importlib.util,json,sys,traceback,types
from collections import Counter
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy import local_follow as local
A=Path(__file__).resolve().parent;P=A.parent
trace=P/'static_side_runtime_r212_v1/two_actor_run1/run1/follow_decisions.jsonl'
events=[json.loads(s) for s in trace.open()]
source=P/'expiry_route_reference_r186_v1/candidate_v2.py'
original=local.LocalFollow._baseline_command
spec=importlib.util.spec_from_file_location('r216_old',source);old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
before_source=source.read_text()
needle='if (recent is not None and reference_wall is not None'
assert before_source.count(needle)==1
fixed_source=before_source.replace(needle,'if (candidate is not None and recent is not None and reference_wall is not None')
(A/'candidate.py').write_text(fixed_source)
candidate=types.ModuleType('r216_fixed');candidate.__file__=str(A/'candidate.py');exec(compile(fixed_source,candidate.__file__,'exec'),candidate.__dict__)
all_results=[];exception=[];baseline=None;summary=[]
for variant,module in [('before',old),('after',candidate)]:
    local.LocalFollow._baseline_command=original;module.install()
    owner,mode=make_controller(events[0]['data']);outputs={};errors=[];counts=Counter()
    def trace_error(frame,event,arg):
        if event=='exception' and arg[0].__name__=='AxisError':
            exception.append(dict(variant=variant,type=arg[0].__name__,message=str(arg[1]),stack=traceback.format_stack(frame)))
        return trace_error
    for e in events[1:]:
        d=e['data']
        if e['kind']=='reset':
            if d.get('reason')=='decision_source_expired':module.reset_reference(owner)
            else:owner.reset()
        if e['kind']!='call':continue
        if e['seq']==775:sys.settrace(trace_error)
        try:out=freeze(replay_call(owner,d['input'],mode))
        finally:sys.settrace(None)
        outputs[e['seq']]=out;counts['calls']+=1
        normal=out[2].get('normal',{})
        if variant=='before':
            if not equal_output(comparable_decision(out),comparable_decision(d['output'])):errors.append(e['seq'])
        else:
            b=baseline[e['seq']]
            counts['stop_to_positive']+=b[0]<=0<out[0];counts['new_stop']+=out[0]<=0<b[0]
            counts['input_error']+=out[2].get('reason')=='follow_input_stop'
            if b[:2]!=out[:2] or 59<=d['input']['reference_s']<=61:
                all_results.append(dict(seq=e['seq'],reference_s=d['input']['reference_s'],before=b[:2],after=out[:2],
                    reason=normal.get('reason',out[2].get('reason')),wall=normal.get('path_wall'),
                    boundary=normal.get('boundary_selection'),input_error=out[2].get('input_error')))
    if variant=='before':baseline=outputs
    summary.append(dict(variant=variant,counts=dict(counts),baseline_mismatches=errors))
with (A/'comparison.json').open('x') as f:json.dump(dict(summary=summary,rows=all_results,exceptions=exception,
    trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),scope='stored-input comparison, no new driving'),f,indent=2);f.write('\n')
print(json.dumps(summary));print('exception records',len(exception));assert not any(x['baseline_mismatches'] for x in summary)
