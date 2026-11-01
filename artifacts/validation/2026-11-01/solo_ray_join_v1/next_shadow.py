"""다음 병목 후보를 메모리 사본에만 적용해 저장 입력 명령을 비교한다. 런타임 소스 변경 없음, 주행 예측 아님.

호출: next_shadow.py <run-folder> <출력.json>
"""
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path
import sys
import time
import types

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import local_follow as base_module
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.lidar_motion import MotionConfig

SOURCE=(ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py').read_bytes().decode('utf-8').replace('\r\n','\n')
GATE='and 0. < target <= cfg.max_speed_mps\n'
WALL="""            open_route = bool(cruise and self.path_source == scene.source_first_s and
                              self.path_wall == side and front and all(r.status == 'return' for r in front))
"""
BILATERAL="""            if fresh:
                self.shadow_selection = detail.get('boundary_selection')
            open_route = bool(cruise and self.path_source == scene.source_first_s and
                              (self.path_wall == side or getattr(self, 'shadow_selection', None) == 'bilateral_confirmed_alternative')
                              and front and all(r.status == 'return' for r in front))
"""
VARIANTS={'preview_above_base':[(GATE,'and 0. < target\n')],'bilateral_open_route':[(WALL,BILATERAL)],
          'both':[(GATE,'and 0. < target\n'),(WALL,BILATERAL)]}


def patched(pairs,name):
    text=SOURCE
    for old,new in pairs:
        if text.count(old)!=1:raise ValueError('source shape changed: '+old)
        text=text.replace(old,new)
    module=types.ModuleType('shadow_'+name)
    module.__dict__['__file__']=base_module.__file__
    exec(compile(text,'shadow_'+name,'exec'),module.__dict__)
    return module


def run(folder,window,modules):
    source=folder/'run1/follow_decisions.jsonl'
    digest=sha256(source.read_bytes()).hexdigest()
    stats={k:Counter() for k in modules};changes={k:[] for k in modules};elapsed=Counter()
    last=-1;motion=None;footer=None
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            e=json.loads(line);d=e['data']
            if e['seq']!=last+1:raise ValueError('trace order')
            last=e['seq']
            if e['kind']=='header':
                raw=d['follow_config'];motion=MotionConfig(**d['motion_config'])
                old=base_module.LocalFollow(base_module.FollowConfig(**raw))
                new={k:m.LocalFollow(m.FollowConfig(**raw)) for k,m in modules.items()}
            elif e['kind']=='reset':
                old.reset()
                for follow in new.values():follow.reset()
            elif e['kind']=='footer':footer=d
            elif e['kind']=='call':
                started=time.perf_counter();a=replay_call(old,d['input'],motion);elapsed['baseline']+=time.perf_counter()-started
                if not equal_output(comparable_decision(freeze(a)),comparable_decision(d['output'])):raise ValueError(f'baseline mismatch {last}')
                t=d['input']['reference_s'];dt=d['input']['dt_s']
                inside=window[0]<=t<=window[1]
                for name,follow in new.items():
                    started=time.perf_counter();b=replay_call(follow,d['input'],motion);elapsed[name]+=time.perf_counter()-started
                    if not inside:continue
                    s=stats[name];s['lap_calls']+=1
                    s['baseline_target_integral_m']+=a[0]*dt;s['candidate_target_integral_m']+=b[0]*dt
                    s['baseline_time_proxy_s']+=dt*(a[0]>0);s['candidate_peak_mps']=max(s['candidate_peak_mps'],b[0])
                    if abs(a[0]-b[0])>1e-8 or abs(a[1]-b[1])>1e-8:
                        s['commands_changed']+=1;s['new_zero']+=int(a[0]>.001>=b[0]);s['zero_released']+=int(b[0]>.001>=a[0])
                        s['speed_up']+=int(b[0]>a[0]+1e-8);s['speed_down']+=int(b[0]<a[0]-1e-8)
                        s['steering_changed']+=int(abs(a[1]-b[1])>1e-8)
                        s['rise_integral_m']+=max(0.,b[0]-a[0])*dt
                        changes[name].append(dict(t=t,old=a[0],new=b[0],old_reason=a[2].get('reason'),new_reason=b[2].get('reason'),old_steering=a[1],new_steering=b[1]))
    if footer is None or digest!=sha256(source.read_bytes()).hexdigest():raise ValueError('incomplete or changed input')
    return dict(run=folder.relative_to(ROOT).as_posix(),input_sha256=digest,counts={k:dict(v) for k,v in stats.items()},changes=changes,elapsed_s=dict(elapsed))


if __name__=='__main__':
    folder=Path(sys.argv[1]);output=Path(sys.argv[2])
    metrics=json.loads((folder/'run1_metrics.json').read_bytes())
    begin=metrics['first_green_sample_s'];lap=metrics['lap_s_from_first_green_sample']
    modules={k:patched(v,k) for k,v in VARIANTS.items()}
    result=run(folder,(begin,begin+lap if lap else float('inf')),modules)
    result['scope']='메모리 사본의 저장 입력 비교. 후보 채택이나 주행 시간 예측이 아니다.'
    result['script_sha256']=sha256(Path(__file__).read_bytes()).hexdigest()
    output.write_text(json.dumps(freeze(result),ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
    print(json.dumps({'counts':result['counts'],'elapsed_s':result['elapsed_s']},ensure_ascii=False))
