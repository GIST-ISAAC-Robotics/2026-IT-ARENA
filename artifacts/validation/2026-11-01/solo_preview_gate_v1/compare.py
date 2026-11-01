"""실제 옵션의 저장 입력 대조. OFF 기록 재현, ON은 앞선 메모리 사본과 일치 확인. 주행 예측 아님."""
from collections import Counter
from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import local_follow as module
from arena_autonomy.follow_trace import replay_call,replay_events,freeze,equal_output,comparable_decision
from arena_autonomy.lidar_motion import MotionConfig

SOLO=('solo_corridor_arc_v1','solo_route_v1','solo_headway_v1','solo_ray_join_v1')
SHADOW={'solo_corridor_arc_v1':'next_shadow_arc.json','solo_ray_join_v1':'next_shadow_ray.json'}


def compare(folder,window):
    source=folder/'follow_decisions.jsonl'
    digest=sha256(source.read_bytes()).hexdigest()
    counts=Counter();changes=[];last=-1;footer=None;costs=dict(off=[],on=[])
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            e=json.loads(line);d=e['data'];kind=e['kind']
            if e['seq']!=last+1 or footer is not None:raise ValueError('trace order')
            last=e['seq']
            if kind=='header':
                base=module.FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
                old=module.LocalFollow(base);new=module.LocalFollow(replace(base,preview_above_base_speed=True))
            elif kind=='reset':old.reset();new.reset()
            elif kind=='footer':footer=d
            elif kind=='call':
                began=time.perf_counter();a=replay_call(old,d['input'],motion);middle=time.perf_counter()
                b=replay_call(new,d['input'],motion);ended=time.perf_counter()
                if not equal_output(comparable_decision(freeze(a)),comparable_decision(d['output'])):raise ValueError(f'baseline mismatch {last}')
                counts['baseline_matched']+=1
                costs['off'].append(middle-began);costs['on'].append(ended-middle)
                t=d['input']['reference_s'];dt=d['input']['dt_s']
                if not window[0]<=t<=window[1]:continue
                counts['lap_calls']+=1
                for tag,value in (('baseline',a),('candidate',b)):
                    counts[tag+'_target_integral_m']+=value[0]*dt
                    counts[tag+'_peak_mps']=max(counts[tag+'_peak_mps'],value[0])
                preview=b[2].get('turn_preview')
                if preview:
                    counts['previewed_s']+=dt;counts['preview_'+preview]+=1
                    counts['previewed_above_base_s']+=dt*(b[2].get('constant_arc_target_mps',0.)>base.max_speed_mps)
                if abs(a[0]-b[0])>1e-8 or abs(a[1]-b[1])>1e-8:
                    counts['commands_changed']+=1;counts['new_zero']+=int(a[0]>.001>=b[0]);counts['zero_released']+=int(b[0]>.001>=a[0])
                    counts['speed_up']+=int(b[0]>a[0]+1e-8);counts['speed_down']+=int(b[0]<a[0]-1e-8)
                    counts['steering_changed']+=int(abs(a[1]-b[1])>1e-8)
                    counts['largest_rise_mps']=max(counts['largest_rise_mps'],b[0]-a[0])
                    changes.append(dict(t=t,old=a[0],new=b[0],old_steering=a[1],new_steering=b[1],old_reason=a[2].get('reason'),new_reason=b[2].get('reason')))
            else:raise ValueError('unknown event')
    if footer is None or footer['written']!=last or footer['accepted']!=last or footer['worker_alive'] is not False or any(footer[k] for k in ('failed','dropped','capture_errors','pending')):
        raise ValueError('incomplete trace')
    if digest!=sha256(source.read_bytes()).hexdigest():raise ValueError('input changed')
    summary={k:{p:float(np.percentile(np.array(v)*1000,p)) for p in (50,95,99,100)} for k,v in costs.items()}
    return dict(run=folder.relative_to(ROOT).as_posix(),input_sha256=digest,counts=dict(counts),changes=changes,replay_ms=summary)


if __name__=='__main__':
    files=[Path(__file__).resolve(),*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={p.relative_to(ROOT).as_posix():sha256(p.read_bytes()).hexdigest() for p in files}
    results={}
    for name in SOLO:
        folder=HERE.parent/name
        metrics=json.loads((folder/'run1_metrics.json').read_bytes())
        begin=metrics['first_green_sample_s'];lap=metrics['lap_s_from_first_green_sample']
        results[name]=compare(folder/'run1',(begin,begin+lap if lap else float('inf')))
    # 교통 구성은 실행 당시 제어기 클래스를 복원하는 기존 재생기로만 대조한다. ON을 만들 수 없다.
    traffic=HERE.parent/'corner_catchup_v1/run1/follow_decisions.jsonl'
    with traffic.open(encoding='utf-8') as stream:
        replay=replay_events(map(json.loads,stream))
    results['corner_catchup_v1']=dict(run=traffic.parent.relative_to(ROOT).as_posix(),input_sha256=sha256(traffic.read_bytes()).hexdigest(),
        counts=dict(baseline_matched=replay['calls'],mismatches=len(replay['mismatches'])),changes=[])
    if replay['mismatches']:raise ValueError('traffic baseline mismatch')
    agreement={}
    for name,file in SHADOW.items():
        expected=json.loads((HERE.parent/'solo_ray_join_v1'/file).read_bytes())['changes']['preview_above_base']
        actual=results[name]['changes']
        agreement[name]=dict(expected=len(expected),actual=len(actual),same=len(expected)==len(actual) and all(
            abs(p['t']-q['t'])<1e-9 and abs(p['new']-q['new'])<1e-9 and abs(p['new_steering']-q['new_steering'])<1e-9 for p,q in zip(expected,actual)))
    unchanged=all(sha256((ROOT/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
    output=dict(scope='preview_above_base_speed OFF 재현과 ON 명령 비교. 저장 입력이며 새 주행 예측 아님. 재생 비용은 PC 1회 값.',results=results,
                on_matches_shadow=agreement,source_sha256=hashes,source_unchanged=unchanged)
    (HERE/'compare.json').write_text(json.dumps(freeze(output),ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
    print(json.dumps({k:{'counts':v['counts'],'replay_ms':v.get('replay_ms')} for k,v in results.items()},ensure_ascii=False))
    print('on_matches_shadow',agreement,'source_unchanged',unchanged)
    if not all(v['same'] for v in agreement.values()) or not unchanged:raise SystemExit(1)
