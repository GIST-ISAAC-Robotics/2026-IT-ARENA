"""종료된 주행의 요약 기록 범위/실제 최대 무진행을 검토한다. 원래 판정은 보존."""
from collections import deque
import hashlib
import json
from pathlib import Path

base=Path(__file__).resolve().parent
run=base/'run1'
names=['report.json','trajectory.jsonl','ego_poses.jsonl','follow_decisions.jsonl']
hashes={n:hashlib.sha256((run/n).read_bytes()).hexdigest() for n in names}
report=json.loads((run/'report.json').read_text(encoding='utf-8'))
first,last=report['green_observed_sim_s'],report['stop_request_sim_s']
rows=[r for line in (run/'trajectory.jsonl').open() if first<=(r:=json.loads(line))['t']<=last]
lo=0;low=deque();high=deque();best=(0.,0,0)
for hi,row in enumerate(rows):
    value=row['progress_m']
    while low and rows[low[-1]]['progress_m']>=value:low.pop()
    while high and rows[high[-1]]['progress_m']<=value:high.pop()
    low.append(hi);high.append(hi)
    while rows[high[0]]['progress_m']-rows[low[0]]['progress_m']>.05:
        lo+=1
        while low[0]<lo:low.popleft()
        while high[0]<lo:high.popleft()
    duration=row['t']-rows[lo]['t']
    if duration>best[0]:best=(duration,lo,hi)
poses=[json.loads(line) for line in (run/'ego_poses.jsonl').open(encoding='utf-8')]
tail=[x for x in poses if rows[-1]['t']<=x[0]<=last]
trace_counts={'continuous_prefix_reason_calls':0,'motion_pause_retained_calls':0,'motion_pause_resumed_calls':0}
for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
    if 'continuous_short_observed_prefix_geometry_only' in line:trace_counts['continuous_prefix_reason_calls']+=1
    if '"motion_pause_intent_retained": true' in line:trace_counts['motion_pause_retained_calls']+=1
    if '"motion_pause_resumed": true' in line:trace_counts['motion_pause_resumed_calls']+=1
result={'scope':'post-run diagnostic only; original coverage FAIL unchanged; no extra simulation',
 'trajectory':{'green_s':first,'stop_request_s':last,'first_s':rows[0]['t'],'last_s':rows[-1]['t'],
   'start_gap_s':rows[0]['t']-first,'end_gap_s':last-rows[-1]['t'],
   'maximum_sample_gap_s':max(b['t']-a['t'] for a,b in zip(rows,rows[1:])),
   'actual_maximum_5cm_window_s':best[0],'window_first_s':rows[best[1]]['t'],'window_last_s':rows[best[2]]['t']},
 'dense_pose_tail':{'samples':len(tail),'first_s':tail[0][0],'last_s':tail[-1][0],
   'x_span_m':max(x[1] for x in tail)-min(x[1] for x in tail),
   'y_span_m':max(x[2] for x in tail)-min(x[2] for x in tail)},
 'trace_counts':trace_counts,'raw_sha256':hashes,
 'raw_unchanged':hashes=={n:hashlib.sha256((run/n).read_bytes()).hexdigest() for n in names}}
with (base/'run1_review.json').open('x') as stream:json.dump(result,stream,indent=2)
print(json.dumps(result))
