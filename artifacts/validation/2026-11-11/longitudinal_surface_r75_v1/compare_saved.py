"""r73 입력을 고정하고 r74/r75 이전과 현재 제어 출력을 비교한다."""
import hashlib
import json
import sys
import types
import zipfile
from pathlib import Path
import arena_autonomy.partial_surface_alignment as alignment
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision

HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
TRACE=HERE.parent/'gearing_81t_r73_v1/solo_run1/follow_decisions.jsonl'
OUT=HERE/'saved_compare.json'
if OUT.exists():raise SystemExit('preserve previous comparison')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files=[TRACE,ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py',
       ROOT/'src/arena_autonomy/arena_autonomy/partial_surface_alignment.py']
hashes={str(p):sha(p) for p in files}

def frozen_module(archive, path, name):
    module=types.ModuleType(name);sys.modules[name]=module
    with zipfile.ZipFile(archive) as z:source=z.read(path)
    exec(compile(source,str(archive)+'!'+path,'exec'),module.__dict__)
    return module

old_follow=frozen_module(HERE.parent/'forward_prefix_r74_v1/before_r74.zip',
    'src/arena_autonomy/arena_autonomy/local_follow.py','arena_autonomy.r73_local_follow')
old_alignment=frozen_module(HERE/'before_r75.zip',
    'src/arena_autonomy/arena_autonomy/partial_surface_alignment.py','arena_autonomy.r74_surface_alignment')
new_fit=alignment.surface_translation

def replace_aliases(old,new):
    for name,module in list(sys.modules.items()):
        if name.startswith('arena_autonomy.') and module:
            for key,value in list(vars(module).items()):
                if value is old:setattr(module,key,new)

def run(cls):
    rows=[];mismatches=[]
    for line in TRACE.open(encoding='utf-8'):
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            controller=cls(FollowConfig(**d['follow_config']));motion=MotionConfig(**d['motion_config'])
        elif e['kind']=='reset':controller.reset()
        elif e['kind']=='call':
            actual=freeze(replay_call(controller,d['input'],motion))
            if not equal_output(comparable_decision(actual),comparable_decision(d['output'])):mismatches.append(e['seq'])
            rows.append(dict(seq=e['seq'],time_s=d['input']['reference_s'],speed=actual[0],steer=actual[1],reason=actual[2].get('reason')))
    return rows,mismatches

replace_aliases(new_fit,old_alignment.surface_translation)
before,mismatches=run(old_follow.LocalFollow)
replace_aliases(old_alignment.surface_translation,new_fit)
after,_=run(LocalFollow)
changes=[dict(before=a,after=b) for a,b in zip(before,after)
         if abs(a['speed']-b['speed'])>1e-10 or abs(a['steer']-b['steer'])>1e-10 or a['reason']!=b['reason']]
def summary(rows):
    return dict(calls=len(rows),known_obstacle_latched=sum(r['reason']=='follow_known_obstacle_latched' for r in rows),
        positive_after_37_12=sum(r['time_s']>=37.12 and r['speed']>0 for r in rows),last=rows[-1])
result=dict(scope='fixed saved input, no new trajectory; before r73 versus r74+r75',
    baseline_mismatches=mismatches,before=summary(before),after=summary(after),changed_calls=len(changes),
    input_sha256=hashes,unchanged=all(sha(Path(k))==v for k,v in hashes.items()),changed=changes)
OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='changed'},ensure_ascii=False))
