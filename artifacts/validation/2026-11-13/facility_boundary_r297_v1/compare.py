"""r294 원 궤적에서 기존 선택형 경계 갱신을 비교. 시설을 지우지 않는다."""
from pathlib import Path
import copy,hashlib,json,sys,inspect,textwrap
from collections import Counter
A=Path(__file__).resolve().parent;R=A.parents[3];B=R/'artifacts/validation/2026-11-12/cad_facility_lap_r294_v1'
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision
from arena_autonomy import local_follow as lf
original=lf.LocalFollow._baseline_command
def enable_fragment_candidate():
    source=inspect.getsource(original)
    old="if cfg.traffic_boundary_refresh_trial and info.get('wall_model') != 'piecewise_corner':"
    assert source.count(old)==1
    source=source.replace(old,"if (cfg.traffic_boundary_refresh_trial or cfg.narrow_occluder_bridge) and info.get('wall_model') != 'piecewise_corner':")
    namespace={};exec(compile(textwrap.dedent(source),__file__,'exec'),lf.__dict__,namespace)
    lf.LocalFollow._baseline_command=namespace['_baseline_command']
with (B/'run1/follow_decisions.jsonl').open() as f:
    header=json.loads(next(f))['data'];events=[json.loads(line) for line in f]
summary={};changes={}
for mode in ('baseline','current_fragment_refresh'):
    h=copy.deepcopy(header)
    if mode!='baseline':enable_fragment_candidate()
    follow,motion=make_controller(h);n=0;diff=[];late=Counter();selected=Counter();rows=[]
    for e in events:
        if e['kind']=='reset':replay_reset(follow,e['data'])
        if e['kind']!='call':continue
        c=e['data'];out=freeze(replay_call(follow,c['input'],motion));n+=1
        if not equal_output(comparable_decision(out),comparable_decision(c['output'])):
            diff.append(c['input']['reference_s']);rows.append(dict(reference_s=c['input']['reference_s'],before=c['output'][:2],after=out[:2],detail=out[2]))
        if c['input']['reference_s']>=22:
            late['positive' if out[0]>0 else 'zero']+=1;late[out[2].get('reason')]+=1
        selected[out[2].get('boundary_selection','none')]+=1
    summary[mode]=dict(calls=n,different=len(diff),first_difference=diff[0] if diff else None,after_22s=dict(late),selected=dict(selected))
    changes[mode]=rows
    if mode=='baseline':assert not diff,summary[mode]
(A/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
(A/'changed_decisions.json').write_text(json.dumps(changes,separators=(',',':'))+'\n')
print(json.dumps(summary))
