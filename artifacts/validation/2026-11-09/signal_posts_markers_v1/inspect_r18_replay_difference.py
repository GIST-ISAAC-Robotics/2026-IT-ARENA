"""첫 기존 r18 불일치가 명령인지 후속 진단 필드인지 확인한다."""
import json,types,zipfile
from pathlib import Path
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
A=Path(__file__).resolve().parent;R=A.parents[3]
m=types.ModuleType('r55_forecast')
with zipfile.ZipFile(A/'before_r56.zip') as z:exec(compile(z.read('src/arena_autonomy/arena_autonomy/bypass_handoff_corridor_trial.py'),'r55','exec'),m.__dict__)
def differences(a,b,path=''):
    if equal_output(a,b):return []
    if isinstance(a,dict) and isinstance(b,dict):
        return [d for k in a.keys()|b.keys() for d in differences(a.get(k,'<absent>'),b.get(k,'<absent>'),path+'/'+str(k))]
    if isinstance(a,(list,tuple)) and isinstance(b,(list,tuple)) and len(a)==len(b):
        return [d for i,(x,y) in enumerate(zip(a,b)) for d in differences(x,y,path+'/'+str(i))]
    return [dict(path=path,current=a,recorded=b)]
for e in read_events(R/'artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/run1/follow_decisions.jsonl'):
    d=e['data']
    if e['kind']=='header':
        c,mc=build_controller(d)
        c.bypass.guide._padding=types.MethodType(m.ForecastGuide._padding,c.bypass.guide)
        c.bypass.guide.padding_provider=c.bypass.guide._padding
    elif e['kind']=='reset':c.reset()
    elif e['kind']=='call':
        o=freeze(replay_call(c,d['input'],mc))
        if not equal_output(comparable_decision(o),comparable_decision(d['output'])):
            out=dict(t=d['input']['reference_s'],commands_equal=equal_output(o[:2],d['output'][:2]),
                differences=differences(comparable_decision(o),comparable_decision(d['output'])))
            (A/'r18_first_difference.json').write_text(json.dumps(out,indent=2)+'\n')
            print(json.dumps(out,indent=1)[:6500]);break
