"""실행 입력에서 옵션 ON 재현과 OFF 대조로 예측 사용량을 센다. 평가 전용."""
from collections import Counter
from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import local_follow as module
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.lidar_motion import MotionConfig

source=HERE/'run1/follow_decisions.jsonl'
metrics=json.loads((HERE/'run1_metrics.json').read_bytes())
begin=metrics['first_green_sample_s'];lap=metrics['lap_s_from_first_green_sample']
end=begin+lap if lap else float('inf')
counts=Counter();changes=[];last=-1;footer=None
with source.open(encoding='utf-8') as stream:
    for line in stream:
        e=json.loads(line);d=e['data']
        if e['seq']!=last+1 or footer is not None:raise ValueError('trace order')
        last=e['seq']
        if e['kind']=='header':
            config=module.FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
            if not config.preview_above_base_speed or config.ray_wall_continuity:raise ValueError('option mismatch')
            on=module.LocalFollow(config);off=module.LocalFollow(replace(config,preview_above_base_speed=False))
        elif e['kind']=='reset':on.reset();off.reset()
        elif e['kind']=='footer':footer=d
        elif e['kind']=='call':
            a=replay_call(on,d['input'],motion)
            if not equal_output(comparable_decision(freeze(a)),comparable_decision(d['output'])):raise ValueError(f'on replay mismatch {last}')
            counts['on_matched']+=1
            b=replay_call(off,d['input'],motion)
            t=d['input']['reference_s'];dt=d['input']['dt_s']
            if not begin<=t<=end:continue
            counts['lap_calls']+=1
            preview=a[2].get('turn_preview')
            if preview:
                counts['previewed_s']+=dt;counts['preview_'+preview]+=1
                before=a[2].get('constant_arc_target_mps',0.)
                counts['previewed_above_base_s']+=dt*(before>config.max_speed_mps)
                if preview=='observed_turn_clear' and before>config.max_speed_mps:
                    counts['lifted_above_base_s']+=dt;counts['lifted_above_base_integral_m']+=(a[0]-before)*dt
            if abs(a[0]-b[0])>1e-8 or abs(a[1]-b[1])>1e-8:
                counts['commands_differ_from_off']+=1
                counts['faster_than_off']+=int(a[0]>b[0]+1e-8);counts['slower_than_off']+=int(a[0]<b[0]-1e-8)
                counts['steering_differs']+=int(abs(a[1]-b[1])>1e-8)
                changes.append(dict(t=t,on_speed=a[0],off_speed=b[0],on_steering=a[1],off_steering=b[1],on_reason=a[2].get('reason'),off_reason=b[2].get('reason')))
if footer is None or footer['written']!=last or footer['worker_alive'] is not False or any(footer[k] for k in ('failed','dropped','capture_errors','pending')):
    raise ValueError('incomplete trace')
result=dict(scope='실행 입력의 ON 재현과 같은 입력 OFF 대조. OFF 값은 폐루프 반사실 주행이 아니다.',counts=dict(counts),changes=changes,
            input_sha256=sha256(source.read_bytes()).hexdigest(),script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
with (HERE/'usage.json').open('x',encoding='utf-8',newline='\n') as stream:json.dump(freeze(result),stream,ensure_ascii=False,indent=1)
print(json.dumps(result['counts']))
