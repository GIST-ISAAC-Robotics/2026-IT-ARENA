"""r474 최종 정책의 저장 plan/current 각 한 번. 장치/ROS/구동 접근 없음."""
from pathlib import Path
import hashlib,json,os,resource,statistics,sys,time
A=Path(__file__).resolve().parent;S=A/'src'
sys.path[:0]=[str(S/'arena_autonomy'),str(S/'arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator,supported
from arena_autonomy.runtime_dual_state import current_request
from arena_autonomy.follow_trace import equal_output

digest=lambda x:hashlib.sha256(json.dumps(x,sort_keys=True,allow_nan=False).encode()).hexdigest()
def stats(rows):
    values=sorted(r['ms'] for r in rows);i=(len(values)-1)*.95;j=int(i)
    return dict(n=len(values),median_ms=statistics.median(values),p95_ms=values[j]+(values[min(j+1,len(values)-1)]-values[j])*(i-j),
        max_ms=max(values),over_100ms=sum(v>100 for v in values),first_ms=rows[0]['ms'],
        median_excluding_first_ms=statistics.median(r['ms'] for r in rows[1:]))

def main():
    data=json.loads((A/'inputs.json').read_bytes());header=data['header'];initial=digest(data)
    expected_plans={tuple(r['key']):r['output'] for r in map(json.loads,(A/'expected_plan_outputs.jsonl').read_text().splitlines())}
    expected_currents=json.loads((A/'expected_current_outputs.json').read_bytes())
    assert len(data['plans'])==217 and len(data['currents'])==113
    assert not any(header.get(k) for k in ('vehicle_marker_enabled','vehicle_marker_trial'))
    plan_rows=[];current_rows=[];outputs={};generation=None;constructor_ms={}
    start=time.perf_counter_ns();plan=Evaluator('plan',header);constructor_ms['plan']=(time.perf_counter_ns()-start)/1e6
    try:
        with (A/'results/plan_outputs.jsonl').open('x') as f:
            for item in data['plans']:
                key=item['generation'],item['sequence']
                if key[0]!=generation:plan.reset(item['reset_reason']);generation=key[0]
                start=time.perf_counter_ns();output=plan(item['input']);ms=(time.perf_counter_ns()-start)/1e6
                outputs[key]=output;expected=expected_plans[key];normal=output[2].get('normal') or {};old=expected[2].get('normal') or {}
                row=dict(key=key,reference_s=item['input']['reference_s'],ms=ms,command=output[:2],expected_command=expected[:2],
                    exact=output==expected,existing_comparator_equal=equal_output(output,expected),command_equal=equal_output(output[:2],expected[:2]),
                    reason=normal.get('reason'),expected_reason=old.get('reason'),selector_reason=normal.get('selector_reason'),
                    expected_selector_reason=old.get('selector_reason'),mode=normal.get('selector_mode'),expected_mode=old.get('selector_mode'))
                plan_rows.append(row);f.write(json.dumps(dict(key=key,output=output),allow_nan=False)+'\n')
    finally:plan.close()
    start=time.perf_counter_ns();current=Evaluator('current',header);constructor_ms['current']=(time.perf_counter_ns()-start)/1e6
    try:
        with (A/'results/current_outputs.jsonl').open('x') as f:
            for item in data['currents']:
                output_plan=outputs[tuple(item['key'])];assert supported(output_plan)
                request=current_request(output_plan,item['submitted'],item['current'],retained=item['retained'],crop_current_selection=True)
                before=digest(request);start=time.perf_counter_ns();output=current(request);ms=(time.perf_counter_ns()-start)/1e6
                assert before==digest(request)
                expected=expected_currents[str(item['trace_seq'])]
                assert output['runtime_authorized'] is False and output['independent_guard_required'] is True
                row=dict(trace_seq=item['trace_seq'],key=item['key'],reference_s=item['current']['reference_s'],ms=ms,
                    command=[output['speed_mps'],output['steering_rad']],expected_command=[expected['speed_mps'],expected['steering_rad']],
                    exact=output==expected,existing_comparator_equal=equal_output(output,expected),
                    command_equal=equal_output([output['speed_mps'],output['steering_rad']],[expected['speed_mps'],expected['steering_rad']]),
                    reason=output['reason'],expected_reason=expected['reason'],policy=output.get('current_observation_policy'))
                current_rows.append(row);f.write(json.dumps(dict(trace_seq=item['trace_seq'],output=output),allow_nan=False)+'\n')
    finally:current.close()
    assert digest(data)==initial
    imports={n:str(Path(m.__file__).resolve()) for n,m in sys.modules.items()
        if n.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(m,'__file__',None)}
    assert all(Path(p).is_relative_to(S) for p in imports.values())
    result=dict(plan=stats(plan_rows),current=stats(current_rows),plan_rows=plan_rows,current_rows=current_rows,
        constructor_ms=constructor_ms,imports=imports,input_unchanged=True,plan_calls=217,current_calls=113,
        warmup_calls=0,first_call_included_in_statistics=True,marker_default_off=True,
        plan_full_comparator=sum(r['existing_comparator_equal'] for r in plan_rows),current_full_comparator=sum(r['existing_comparator_equal'] for r in current_rows),
        plan_command_equal=sum(r['command_equal'] for r in plan_rows),current_command_equal=sum(r['command_equal'] for r in current_rows),
        peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        scope='ARM candidate against existing WSL candidate; one saved sequence, no live sensors/ROS/IPC/driving')
    (A/'results/result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('plan_rows','current_rows','imports')}),flush=True)

if __name__=='__main__':main()
