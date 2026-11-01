"""실제 연속 광선이 지지하는 먼 직선 벽을 이어 보는 저장 입력 진단. 런타임 변경 없음."""
import ast
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import local_follow as module
from arena_autonomy.follow_trace import replay_call,restore_call,freeze,equal_output,comparable_decision
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from sampling import supported_gaps

original=module.local_path
scene=None
activity=Counter()


def break_indices(points,gaps,limits,horizon):
    if scene is None or horizon<=1.8:return (gaps>limits).nonzero()[0]
    joins=supported_gaps(points,gaps,limits,scene,horizon)
    activity['joins']+=int(joins.sum())
    activity['calls_with_join']+=int(joins.any())
    return ((gaps>limits)&~joins).nonzero()[0]


tree=ast.parse((ROOT/'src/arena_autonomy/arena_autonomy/local_path.py').read_text(encoding='utf-8'))
fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='local_path')
modified=0
for statement in fn.body:
    if isinstance(statement,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='breaks' for t in statement.targets):
        statement.value=ast.parse('break_indices(points,gaps,limits,horizon)',mode='eval').body;modified+=1
assert modified==1
namespace={**original.__globals__,'break_indices':break_indices}
exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'diagnostic_sampling_path','exec'),namespace)
candidate=namespace['local_path']


if __name__=='__main__':
    source=HERE.parent/'solo_corridor_arc_v1/run1/follow_decisions.jsonl'
    metrics=json.loads((HERE.parent/'solo_corridor_arc_v1/run1_metrics.json').read_bytes())
    begin=metrics['first_green_sample_s'];end=begin+metrics['lap_s_from_first_green_sample']
    files=[Path(__file__).resolve(),HERE/'sampling.py',*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={p.relative_to(ROOT).as_posix():sha256(p.read_bytes()).hexdigest() for p in files}
    input_hash=sha256(source.read_bytes()).hexdigest();counts=Counter();changes=[];last=-1;footer=None
    try:
        with source.open(encoding='utf-8') as stream:
            for line in stream:
                e=json.loads(line);d=e['data'];kind=e['kind']
                if e['seq']!=last+1 or footer is not None:raise ValueError('trace order')
                last=e['seq']
                if kind=='header':
                    cfg=module.FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
                    old=module.LocalFollow(cfg);new=module.LocalFollow(cfg)
                elif kind=='reset':old.reset();new.reset()
                elif kind=='footer':footer=d
                elif kind=='call':
                    module.local_path=original;a=replay_call(old,d['input'],motion)
                    if not equal_output(comparable_decision(freeze(a)),comparable_decision(d['output'])):raise ValueError('baseline mismatch '+str(last))
                    counts['baseline_matched']+=1
                    inp,scan,history=restore_call(d['input'],motion)
                    scene=motion_scene(scan,history,inp['reference_s'],timing_verified=inp['timing_verified'],epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
                    module.local_path=candidate;b=replay_call(new,d['input'],motion)
                    t=inp['reference_s'];dt=inp['dt_s']
                    if not begin<=t<=end:continue
                    counts['lap_calls']+=1
                    for name,value in [('baseline',a[0]),('candidate',b[0])]:
                        counts[name+'_target_integral_m']+=value*dt
                        counts[name+'_target_ge_2p6_s']+=dt*(value>=2.6)
                        counts[name+'_peak_mps']=max(counts[name+'_peak_mps'],value)
                    if abs(a[0]-b[0])>1e-8 or abs(a[1]-b[1])>1e-8:
                        counts['commands_changed']+=1;counts['new_zero']+=int(a[0]>.001>=b[0]);counts['zero_released']+=int(b[0]>.001>=a[0])
                        changes.append(dict(seq=last,t=t,old_speed=a[0],new_speed=b[0],old_steering=a[1],new_steering=b[1],old_reason=a[2].get('reason'),new_reason=b[2].get('reason')))
                else:raise ValueError('unknown event')
    finally:module.local_path=original
    if (footer is None or footer['written']!=last or footer['accepted']!=last or footer['worker_alive'] is not False
        or any(footer[k] for k in ('failed','dropped','capture_errors','pending'))):raise ValueError('incomplete trace')
    unchanged=all(sha256((ROOT/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
    output=dict(scope='연속된 실제 광선+양측 세 점의 직선/방사 잔차를 확인하는 진단. 자유 공간 인증이나 새 주행 예측 아님.',counts=dict(counts),activity=dict(activity),changes=changes,
                source_sha256=hashes,source_unchanged=unchanged,input_sha256=input_hash,input_unchanged=input_hash==sha256(source.read_bytes()).hexdigest())
    with (HERE/'sampling_shadow.json').open('x',encoding='utf-8') as stream:json.dump(freeze(output),stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({k:output[k] for k in ('counts','activity','source_unchanged','input_unchanged')}))
    if not unchanged or not output['input_unchanged']:raise SystemExit(1)
