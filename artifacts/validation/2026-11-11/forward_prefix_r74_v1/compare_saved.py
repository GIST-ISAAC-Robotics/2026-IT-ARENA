"""현재 코드와 r74 직전의 저장 입력 대조. 새 주행/물성 대조가 아니다."""
import hashlib,json,sys,types,zipfile
from pathlib import Path
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
TRACE=ROOT/'artifacts/validation/2026-11-10/combined_c1_facility_v1/two_actor_run1/run1/follow_decisions.jsonl'
OUT=HERE/'saved_compare.json'
if OUT.exists():raise SystemExit('preserve comparison')
source='src/arena_autonomy/arena_autonomy/local_follow.py'
with zipfile.ZipFile(HERE/'before_r74.zip') as z:old_source=z.read(source)
module=types.ModuleType('arena_autonomy.local_follow_before_r74');sys.modules[module.__name__]=module
exec(compile(old_source,'<before_r74>','exec'),module.__dict__)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
hashes={str(TRACE):sha(TRACE),source:sha(ROOT/source)}
changed=[];prefix=[];mismatches=[];calls=0
for line in TRACE.open(encoding='utf-8'):
 e=json.loads(line);d=e['data']
 if e['kind']=='header':
  cfg=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
  before=BypassHandoffFollow(cfg,HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
  before.normal=module.LocalFollow(cfg)
  after=BypassHandoffFollow(cfg,HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
 elif e['kind']=='reset':before.reset();after.reset()
 elif e['kind']=='call':
  a=freeze(replay_call(before,d['input'],motion));b=freeze(replay_call(after,d['input'],motion));calls+=1
  if not equal_output(comparable_decision(a),comparable_decision(d['output'])):mismatches.append(e['seq'])
  if not equal_output(comparable_decision(a),comparable_decision(b)):
   changed.append(dict(seq=e['seq'],time_s=d['input']['reference_s'],before=a[:2],after=b[:2],before_reason=a[2].get('normal',{}).get('reason'),after_reason=b[2].get('normal',{}).get('reason')))
  detail=b[2].get('normal',{}).get('retained_forward_prefix')
  if detail is not None:prefix.append(dict(seq=e['seq'],time_s=d['input']['reference_s'],evidence=detail))
result=dict(scope='fixed saved sensors counterfactual; not new trajectory',calls=calls,baseline_mismatches=mismatches,changed_calls=len(changed),zero_to_positive=sum(x['before'][0]==0 and x['after'][0]>0 for x in changed),positive_to_zero=sum(x['before'][0]>0 and x['after'][0]==0 for x in changed),prefix_calls=len(prefix),input_sha256=hashes,unchanged=all(sha(Path(k) if Path(k).is_absolute() else ROOT/k)==h for k,h in hashes.items()),changed=changed,prefix=prefix)
OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('changed','prefix')},ensure_ascii=False))
