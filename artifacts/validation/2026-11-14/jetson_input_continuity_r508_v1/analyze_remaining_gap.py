import gzip,json
from pathlib import Path
A=Path(__file__).resolve().parent
p=A/'candidate_shm16_drain_proof/remote_result/result'
with gzip.open(p/'commands.jsonl.gz','rt',encoding='utf8') as f:commands=[json.loads(l) for l in f]
positive=[r for r in commands if r['topic']=='/drive' and r['speed']>0]
a,b=max(zip(positive,positive[1:]),key=lambda pair:pair[1]['received_s']-pair[0]['received_s'])
controls=json.loads((p/'controller_controls.json').read_bytes())
timing=[json.loads(l) for l in (p/'local_pursuit_timing.jsonl').read_text(encoding='utf8').splitlines()]
failures=[]
for row in controls:
    if row.get('state')!='MOTION_STOP' or not 7.62<=row['control_reference_s']<27.52:continue
    preceding=[t for t in timing if row['begin_monotonic_ns']-200000000<t['begin_monotonic_ns']<row['begin_monotonic_ns']]
    failures.append(dict(control=row,preceding_largest_callbacks=[
        {k:t[k] for k in ('callback','duration_ms','begin_monotonic_ns','end_monotonic_ns')}
        for t in sorted(preceding,key=lambda t:t['duration_ms'],reverse=True)[:3]]))
calls=[]
for line in (p/'controller_trace.jsonl').open(encoding='utf8'):
    event=json.loads(line)
    if event['kind']!='call':continue
    d=event['data'];ref=d['input']['reference_s']
    if a['received_s']-.1<=ref<=b['received_s']+.1:
        calls.append(dict(reference_s=ref,output=d['output'][:2],
            normal_reason=d['output'][2].get('normal',{}).get('reason'),delivery=d['delivery']))
out=dict(longest_positive_gap_s=b['received_s']-a['received_s'],before=a,after=b,
    active_not_eof=True,active_motion_stale=failures,gap_calls=calls,
    full_trace_bytes=(p/'controller_trace.jsonl').stat().st_size,
    limitation='full diagnostic trace enabled; synchronous freeze occurs in control; trace share of long callback not measured')
(A/'remaining_gap_analysis.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(gap_s=out['longest_positive_gap_s'],active_stale=len(failures))))
