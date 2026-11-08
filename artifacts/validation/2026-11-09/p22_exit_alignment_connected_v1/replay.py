"""연결 r51과 저장 제안 일치. 허가 뒤 반사실 입력은 실제 주행 성적으로 쓰지 않는다."""
import hashlib,json
from pathlib import Path
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.p22_route_trial import P22Route

A=Path(__file__).resolve().parent;source=A.parent/'p22_route_connected_v1/run1/follow_decisions.jsonl'
sha=lambda:hashlib.sha256(source.read_bytes()).hexdigest();before=sha()
expected=json.loads((A.parent/'p22_exit_continuity_v1/probe_summary.json').read_bytes())['new_first_exit']
first=None;mismatches=[];calls=0
for line in source.open():
 e=json.loads(line);k=e['kind'];d=e['data']
 if k=='header':mc=MotionConfig(**d['motion_config']);f=P22Route(LocalFollow(FollowConfig(**d['follow_config'])))
 elif k=='p22_frame':f.frame(**d)
 elif k=='reset':f.reset(force=d.get('force',d['reason'] in ('autonomy_reset','legacy_reset')))
 elif k=='call':
  calls+=1;t=d['input']['reference_s'];out=replay_call(f,d['input'],mc)
  if t<23.82 and not equal_output(freeze(out[:2]),d['output'][:2]):mismatches.append(t)
  if f.phase=='exit':
   first=dict(t=t,output=freeze(out),path=f.path.tolist());break
out=dict(calls=calls,before_old_exit_mismatches=mismatches,first_exit=first,
         prototype_equal=equal_output(first,expected),input_unchanged=sha()==before,input_sha256=before)
(A/'replay_summary.json').write_text(json.dumps(out,separators=(',',':'))+'\n')
print('calls',calls,'prefix_mismatches',mismatches,'prototype_equal',out['prototype_equal'],'input_unchanged',out['input_unchanged'])
print('first',None if first is None else (first['t'],first['output'][:2]))
assert first and not mismatches and out['prototype_equal'] and out['input_unchanged']
