"""회수된 ARM 결과와 기존 WSL 결과의 차이만 분석한다."""
from pathlib import Path
from collections import Counter
import hashlib,json,math,statistics,datetime
A=Path(__file__).resolve().parent
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=load(A/'remote_result/results/result.json');receipt=load(A/'remote_result/execution_receipt.json')
expected=load(A/'bundle/expected_current_outputs.json')
actual={str(row['trace_seq']):row['output'] for row in map(json.loads,(A/'remote_result/results/current_outputs.jsonl').read_text(encoding='utf-8').splitlines())}
def differences(a,b,path='$'):
    if isinstance(a,(int,float)) and isinstance(b,(int,float)):
        return [] if math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-10) else [dict(path=path,expected=a,actual=b)]
    if isinstance(a,dict) and isinstance(b,dict):
        out=[]
        for k in sorted(a.keys()|b.keys()):
            if k not in a or k not in b:out.append(dict(path=path+'.'+k,expected=a.get(k,'<missing>'),actual=b.get(k,'<missing>')))
            else:out+=differences(a[k],b[k],path+'.'+k)
        return out
    if isinstance(a,(list,tuple)) and isinstance(b,(list,tuple)):
        if len(a)!=len(b):return [dict(path=path+'.length',expected=len(a),actual=len(b))]
        return [d for i,(x,y) in enumerate(zip(a,b)) for d in differences(x,y,path+'['+str(i)+']')]
    return [] if a==b else [dict(path=path,expected=a,actual=b)]
failed=[]
for row in r['current_rows']:
    if not row['existing_comparator_equal']:
        seq=str(row['trace_seq']);failed.append(dict(trace_seq=row['trace_seq'],reference_s=row['reference_s'],
            command_equal=row['command_equal'],reason=row['reason'],expected_reason=row['expected_reason'],
            differences=differences(expected[seq],actual[seq])))
policy=Counter(row.get('selector_reason') for row in r['plan_rows'])
current_policy=Counter(row.get('policy') for row in r['current_rows'])
def group_stats(rows):
    values=sorted(row['ms'] for row in rows);i=(len(values)-1)*.95;j=int(i)
    return dict(n=len(values),median_ms=statistics.median(values),p95_ms=values[j]+(values[min(j+1,len(values)-1)]-values[j])*(i-j),
        max_ms=max(values),over_100ms=sum(x>100 for x in values))
groups=dict(plan_route=group_stats([row for row in r['plan_rows'] if row['selector_reason']=='current_route_priority_v1']),
    plan_other=group_stats([row for row in r['plan_rows'] if row['selector_reason']!='current_route_priority_v1']),
    current_route=group_stats([row for row in r['current_rows'] if row['policy']=='current_route_priority_v1']),
    current_other=group_stats([row for row in r['current_rows'] if row['policy']!='current_route_priority_v1']))
manifest=load(A/'bundle/source_manifest.json')
changed=[p for p,h in manifest['files'].items() if sha(A/'bundle'/p)!=h]
result=dict(plan=r['plan'],current=r['current'],plan_raw_exact=sum(row['exact'] for row in r['plan_rows']),
    current_raw_exact=sum(row['exact'] for row in r['current_rows']),
    plan_full_comparator=r['plan_full_comparator'],current_full_comparator=r['current_full_comparator'],
    plan_commands_equal=r['plan_command_equal'],current_commands_equal=r['current_command_equal'],
    plan_policy_counts=dict(policy),current_policy_counts=dict(current_policy),
    branch_timing=groups,first_original_stop_current=next(row for row in r['current_rows'] if row['trace_seq']==823),
    plan_over_100ms=[row for row in r['plan_rows'] if row['ms']>100],
    current_full_comparator_failures=failed,
    local_bundle_changes=changed,trace_unchanged=sha(A.parent/'solo_full_lap_r469_v1/run1/follow_decisions.jsonl')==manifest['trace_sha256'],
    resource_return=dict(time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),returned=True,
        child_pid=receipt['step']['pid'],exit_code=receipt['step']['exit_code'],forced=receipt['step']['forced'],
        residual_group=receipt['step']['residual_group'],remaining=receipt['remaining'],
        original_native_unchanged=receipt['native_original_unchanged'],source_changes=receipt['source_changes'],
        native_build=False,packages_installed=False,devices_opened=False,ros_started=False,settings_changed=False))
(A/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
(A/'resource_return.json').write_text(json.dumps(result['resource_return'],indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('resource_return','plan_over_100ms')},indent=2))
print('slow_plans',json.dumps([dict(key=row['key'],reference_s=row['reference_s'],ms=row['ms'],mode=row['mode'],reason=row['selector_reason']) for row in result['plan_over_100ms']]))
assert not changed and result['trace_unchanged'] and receipt['completed']
