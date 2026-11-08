"""설치 소스와 분리된 후보의 OFF 재현 및 첫 ON 명령 비교. 실제 움직임은 재생하지 않는다."""
from pathlib import Path
import json,sys,hashlib
A=Path(__file__).resolve().parent;sys.path.insert(0,str(A/'overlay'))
from arena_autonomy.follow_trace import MotionConfig,FollowConfig,replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
import arena_autonomy.bypass_handoff as module
assert (A/'overlay') in Path(module.__file__).parents
trace=A.parent/'ops_traffic_centered_entry_v1/run1/follow_decisions.jsonl'
count=mismatch=0;first=None
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        m=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config'])
        off,on=[BypassHandoffFollow(cfg,HandoffExecutionConfig.from_mapping({**d['execution_config'],'adaptive_entry_preview':enabled}),allow_simulation_trial=True) for enabled in (False,True)]
    elif e['kind']=='reset':off.reset();on.reset()
    elif e['kind']=='call':
        i=d['input'];t=i['reference_s']
        if t>30:break
        expected=d['output'];actual=freeze(replay_call(off,i,m));count+=1
        mismatch+=not equal_output(comparable_decision(actual),comparable_decision(expected))
        if first is not None:continue
        v,st,detail=replay_call(on,i,m);b=detail.get('bypass') or {};preview=b.get('adaptive_preview') or {}
        if preview.get('applied') and (v!=expected[0] or st!=expected[1]):
            first=dict(t=t,original_command=expected[:2],new_command=[v,st],preview=preview,alignment=b.get('preview_alignment'),candidate=b.get('selected'),
                       scope='first command divergence on unchanged saved inputs; not a driven result')
assert mismatch==0 and first
out=dict(off_matched=count,mismatch=mismatch,first_candidate=first,loaded_module=str(Path(module.__file__).resolve()),overlay_manifest_sha256=hashlib.sha256((A/'overlay_manifest.json').read_bytes()).hexdigest(),piecewise_parent_sha256=hashlib.sha256((A.parents[3]/'src/arena_autonomy/arena_autonomy/piecewise_wall.py').read_bytes()).hexdigest())
(A/'replay_overlay.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps(out))
