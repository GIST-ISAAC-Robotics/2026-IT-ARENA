"""두 저장 입력의 실제 연결 출력을 v2 비교 결과와 대조한다."""
import hashlib,json,sys,time
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
HERE=Path(__file__).resolve().parent
name=sys.argv[1]
sources={'stationary':'stationary_rejoin_state_v1/persistent_block/run1/follow_decisions.jsonl','restart':'stop_restart_newcourse_v1/moving_stop_restart/run1/follow_decisions.jsonl'}
source=HERE.parent/sources[name]
shadow=json.loads((HERE.parent/'rear_face_motion_shadow_v2'/f'{name}.json').read_text())
assert shadow['source_sha256']==hashlib.sha256(source.read_bytes()).hexdigest()
changes={r['t']:r['after'] for r in shadow['changes']}
events=[json.loads(l) for l in source.read_text().splitlines()];h=events[0]['data'];mc=MotionConfig(**h['motion_config'])
ctl=BypassHandoffFollow(FollowConfig(**h['follow_config']),HandoffExecutionConfig.from_mapping(h['execution_config']),allow_simulation_trial=True)
count=0;costs=[]
for e in events[1:]:
    if e['kind']=='reset':ctl.reset()
    elif e['kind']=='call':
        d=e['data'];start=time.perf_counter();out=freeze(replay_call(ctl,d['input'],mc));costs.append(time.perf_counter()-start)
        expected=changes.get(d['input']['reference_s'],d['output'][:2])
        assert equal_output(out[:2],expected),(d['input']['reference_s'],out[:2],expected)
        count+=1
result=dict(calls=count,all_numeric_match_prior_shadow=True,changes=len(changes),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),wall_decision_seconds={'median':sorted(costs)[len(costs)//2],'max':max(costs)},scope='fixed input; no actual progress or Jetson timing proof')
(HERE/f'{name}_replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
