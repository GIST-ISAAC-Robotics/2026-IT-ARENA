"""r76 보존 소스의 선택기와 격리 연결 후보를 같은 센서 입력으로 비교한다."""
import hashlib,importlib.util,json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
FROZEN=ROOT/'build/r76_frozen_diagnosis'
sys.path[:0]=[str(FROZEN/'src/arena_autonomy'),str(FROZEN/'src/arena_vehicle_interface'),str(FROZEN/'scripts')]
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
spec=importlib.util.spec_from_file_location('r78_saved_candidate',HERE/'candidate.py')
candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)
original=ps.PassReleaseSelector.select
TRACE=HERE.parent/'facility_noise_r76_v1/two_actor_run1/run1/follow_decisions.jsonl'
OUT=HERE/'saved_compare.json'
if OUT.exists():raise SystemExit('preserve prior comparison')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files=[TRACE,HERE/'candidate.py',Path(ps.__file__)]
hashes={str(p):sha(p) for p in files}
calls=0;mismatches=[];changes=[];selected=[]
for line in TRACE.open(encoding='utf-8'):
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        motion=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config'])
        execution=HandoffExecutionConfig.from_mapping(d['execution_config'])
        before=BypassHandoffFollow(cfg,execution,allow_simulation_trial=True)
        after=BypassHandoffFollow(cfg,execution,allow_simulation_trial=True)
    elif e['kind']=='reset':before.reset();after.reset()
    elif e['kind']=='call':
        ps.PassReleaseSelector.select=original
        a=freeze(replay_call(before,d['input'],motion))
        ps.PassReleaseSelector.select=candidate.select
        try:b=freeze(replay_call(after,d['input'],motion))
        finally:ps.PassReleaseSelector.select=original
        calls+=1
        if not equal_output(comparable_decision(a),comparable_decision(d['output'])):mismatches.append(e['seq'])
        normal=b[2].get('normal',{});detail=normal.get('selector_confirmed_rear_recheck')
        if detail:selected.append(dict(seq=e['seq'],time_s=d['input']['reference_s'],detail=detail))
        if not equal_output(comparable_decision(a),comparable_decision(b)):
            changes.append(dict(seq=e['seq'],time_s=d['input']['reference_s'],before=a[:2],after=b[:2],
                before_reason=a[2].get('normal',{}).get('selector_reason'),after_reason=normal.get('selector_reason')))
result=dict(scope='frozen r76 source with r78 only; fixed inputs, no new trajectory',calls=calls,
    baseline_mismatches=mismatches,changed_calls=len(changes),selected_calls=len(selected),
    zero_to_positive=sum(x['before'][0]==0 and x['after'][0]>0 for x in changes),
    positive_to_zero=sum(x['before'][0]>0 and x['after'][0]==0 for x in changes),
    first_changes=changes[:20],last_changes=changes[-5:],selected=selected,
    input_sha256=hashes,unchanged=all(sha(Path(k))==h for k,h in hashes.items()))
OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('first_changes','last_changes','selected','input_sha256')},ensure_ascii=False))
raise SystemExit(int(bool(mismatches) or not result['unchanged']))
