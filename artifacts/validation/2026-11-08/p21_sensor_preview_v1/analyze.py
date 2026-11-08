"""저장 r11 센서 전 랩 탐색. 별도 단계에서만 본선 s와 마커·입구 매칭."""
import bisect,hashlib,json,sys
from pathlib import Path
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig,MotionUnavailable
from observe import observe
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
D=ROOT/'artifacts/validation/2026-11-07/junction_handoff_v1/run1'
rows=[];last=None;skipped=0
for l in (D/'follow_decisions.jsonl').open():
    e=json.loads(l)
    if e['kind']=='header':mc=MotionConfig(**e['data']['motion_config'])
    if e['kind']!='call':continue
    d,raw,h=restore_call(e['data']['input'],mc)
    source=(raw.header.stamp.sec,raw.header.stamp.nanosec)
    if source==last:continue
    last=source
    try:s=motion_scene(raw,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    except (ValueError,MotionUnavailable):skipped+=1;continue
    rows.append(dict(t=d['reference_s'],source=source,candidate=observe(s)))
# Fixed sensor observations are complete before evaluator reads the trajectory or map.
tr=json.loads((D/'trace.json').read_bytes());ts=[v['time'] for v in tr]
scene=json.loads((ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/scene.json').read_bytes())
branch=next(b for b in scene['branches'] if 'P21' in b['name']);s0=branch['s0_m']
frames=sorted({v['autonomy']['vision']['last_result']['stamp'] for v in tr if 30 in v['autonomy'].get('marker_ids',[])})
for r in rows:
    i=bisect.bisect_left(ts,r['t']);v=min(tr[max(0,i-1):min(len(tr),i+1)],key=lambda v:abs(v['time']-r['t']))
    r.update(eval_s=v['s'],eval_near_entry=-.4<=s0-v['s']<=2.2,
      prior_id30=bool(frames and frames[0]<=r['t']),nearest_eval_dt=abs(r['t']-v['time']))
found=[r for r in rows if r['candidate']]
res=dict(scope='single recorded main-route solo approach; point/angle probes only, no steering/no proof of traversable branch',
  scans=len(rows),skipped=skipped,found=len(found),near=sum(r['eval_near_entry'] for r in found),far=sum(not r['eval_near_entry'] for r in found),
  near_scans=sum(r['eval_near_entry'] for r in rows),id30_stamps=frames,near_candidates=[r for r in found if r['eval_near_entry']],
  far_examples=[r for r in found if not r['eval_near_entry']][:3],
  sensor_sha256=hashlib.sha256((D/'follow_decisions.jsonl').read_bytes()).hexdigest())
(HERE/'observations.json').write_text(json.dumps(rows,ensure_ascii=False,indent=1)+'\n')
(HERE/'summary.json').write_text(json.dumps(res,ensure_ascii=False,indent=2)+'\n')
print({k:res[k] for k in ['scans','skipped','found','near','far','near_scans']})
