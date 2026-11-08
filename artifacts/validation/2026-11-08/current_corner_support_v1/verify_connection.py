"""적용된 함수/보조 파일과 사본의 동일성 및 실제 연결 명령을 대조한다."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import time
from arena_autonomy.follow_trace import MotionConfig, FollowConfig, replay_call, freeze, equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig

A=Path(__file__).resolve().parent; ROOT=A.parents[3]; P=A.parent/'corner_path_support_probe_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((P/'stage_manifest.json').read_bytes())
assert sha(ROOT/m['source'])==m['after_sha256']
assert sha(ROOT/'src/arena_autonomy/arena_autonomy/corner_boundary_support.py')==m['helper_sha256']
expected=[json.loads(l) for l in (P/'stage_replay_rows.jsonl').open()]
trace=A.parent/'fragment_boundary_connected_v1/right_parallel_first_curve/run1/follow_decisions.jsonl'
rows=[]; reasons=Counter(); n=0; support_calls=0; start=time.perf_counter()
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        mc=MotionConfig(**d['motion_config'])
        engine=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
    elif e['kind']=='reset':engine.reset()
    elif e['kind']=='call':
        out=freeze(replay_call(engine,d['input'],mc));old=expected[n]
        assert old['t']==d['input']['reference_s']
        assert equal_output(out[:2],[old['new']['speed'],old['new']['steering']]),old['t']
        detail=out[2].get('normal',out[2]);support=detail.get('current_corner_support')
        support_calls+=bool(support and support.get('accepted'))
        row=dict(t=old['t'],speed=out[0],steering=out[1],reason=detail.get('reason'),
            selector_reason=detail.get('selector_reason'),selection=detail.get('boundary_selection'),
            path_source=detail.get('path_source_s'),support=support)
        rows.append(row);n+=1
        if out[0]<=0.:reasons[(row['reason'],row['selector_reason'])]+=1
assert n==len(expected)==1640
(A/'connected_rows.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows),encoding='utf8')
result=dict(matched_commands=n,source_and_helper_equal_staged=True,accepted_support_rows=support_calls,
    zero_reasons=[dict(reason=k[0],selector_reason=k[1],calls=v) for k,v in reasons.items()],
    wall_s=time.perf_counter()-start,scope='same saved input; current production matches staged commands, not new driving',
    trace_sha256=sha(trace))
(A/'connection_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(result,ensure_ascii=False),flush=True)
