"""실제 옵션의 저장 입력 대조. OFF는 기록 출력 재현, ON은 명령 변화만 센다. 주행 예측 아님."""
from collections import Counter
from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import local_follow as module
from arena_autonomy.follow_trace import replay_call,replay_events,freeze,equal_output,comparable_decision
from arena_autonomy.lidar_motion import MotionConfig

SOLO=('solo_corridor_arc_v1','solo_route_v1','solo_headway_v1')


def compare(folder,window):
    source=folder/'follow_decisions.jsonl'
    digest=sha256(source.read_bytes()).hexdigest()
    counts=Counter();changes=[];last=-1;footer=None
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            e=json.loads(line);d=e['data'];kind=e['kind']
            if e['seq']!=last+1 or footer is not None:raise ValueError('trace order')
            last=e['seq']
            if kind=='header':
                base=module.FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
                old=module.LocalFollow(base)
                new=None
                if base.solo_route_only and base.blend_open_speed_by_length:
                    new=module.LocalFollow(replace(base,ray_wall_continuity=True))
            elif kind=='reset':
                old.reset()
                if new is not None:new.reset()
            elif kind=='footer':footer=d
            elif kind=='call':
                a=replay_call(old,d['input'],motion)
                if not equal_output(comparable_decision(freeze(a)),comparable_decision(d['output'])):raise ValueError(f'baseline mismatch {last}')
                counts['baseline_matched']+=1
                if new is None:continue
                b=replay_call(new,d['input'],motion)
                t=d['input']['reference_s'];dt=d['input']['dt_s']
                if not window[0]<=t<=window[1]:continue
                counts['lap_calls']+=1
                for tag,value in (('baseline',a),('candidate',b)):
                    counts[tag+'_target_integral_m']+=value[0]*dt
                    counts[tag+'_target_ge_2p6_s']+=dt*(value[0]>=2.6)
                    counts[tag+'_peak_mps']=max(counts[tag+'_peak_mps'],value[0])
                    ahead=value[2].get('observed_path_ahead_m')
                    if ahead is not None:counts[tag+'_path_ge_2p6_s']+=dt*(ahead>=2.6)
                if abs(a[0]-b[0])>1e-8 or abs(a[1]-b[1])>1e-8:
                    counts['commands_changed']+=1;counts['new_zero']+=int(a[0]>.001>=b[0]);counts['zero_released']+=int(b[0]>.001>=a[0])
                    counts['speed_up']+=int(b[0]>a[0]+1e-8);counts['speed_down']+=int(b[0]<a[0]-1e-8)
                    changes.append(dict(t=t,old_speed=a[0],new_speed=b[0],old_steering=a[1],new_steering=b[1],old_reason=a[2].get('reason'),new_reason=b[2].get('reason')))
            else:raise ValueError('unknown event')
    if footer is None or footer['written']!=last or footer['accepted']!=last or footer['worker_alive'] is not False or any(footer[k] for k in ('failed','dropped','capture_errors','pending')):
        raise ValueError('incomplete trace')
    if digest!=sha256(source.read_bytes()).hexdigest():raise ValueError('input changed')
    return dict(run=folder.relative_to(ROOT).as_posix(),input_sha256=digest,counts=dict(counts),changes=changes)


if __name__=='__main__':
    files=[Path(__file__).resolve(),*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={p.relative_to(ROOT).as_posix():sha256(p.read_bytes()).hexdigest() for p in files}
    results={}
    for name in SOLO:
        folder=HERE.parent/name
        metrics=json.loads((folder/'run1_metrics.json').read_bytes()) if (folder/'run1_metrics.json').exists() else {}
        begin=metrics.get('first_green_sample_s') or 0.;lap=metrics.get('lap_s_from_first_green_sample')
        results[name]=compare(folder/'run1',(begin,begin+lap if lap else float('inf')))
    # 교통 구성은 실행 당시 제어기 클래스를 복원하는 기존 재생기로만 대조한다. ON을 만들 수 없다.
    traffic=HERE.parent/'corner_catchup_v1/run1/follow_decisions.jsonl'
    with traffic.open(encoding='utf-8') as stream:
        replay=replay_events(map(json.loads,stream))
    results['corner_catchup_v1']=dict(run=traffic.parent.relative_to(ROOT).as_posix(),input_sha256=sha256(traffic.read_bytes()).hexdigest(),
        counts=dict(baseline_matched=replay['calls'],mismatches=len(replay['mismatches'])),changes=[])
    if replay['mismatches']:raise ValueError('traffic baseline mismatch')
    prototype=json.loads((HERE/'audit_chain_h20.json').read_bytes())['changes']
    actual=results['solo_corridor_arc_v1']['changes']
    same=len(prototype)==len(actual) and all(abs(p['t']-q['t'])<1e-9 and abs(p['new_speed']-q['new_speed'])<1e-9
                                             and abs(p['new_steering']-q['new_steering'])<1e-9 for p,q in zip(prototype,actual))
    unchanged=all(sha256((ROOT/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
    output=dict(scope='ray_wall_continuity OFF 재현과 ON 명령 비교. 저장 입력이며 새 주행 예측 아님.',results=results,
                on_matches_prototype=same,source_sha256=hashes,source_unchanged=unchanged)
    (HERE/'compare.json').write_text(json.dumps(freeze(output),ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
    print(json.dumps({k:v['counts'] for k,v in results.items()},ensure_ascii=False))
    print('on_matches_prototype',same,'source_unchanged',unchanged)
    if not same or not unchanged:raise SystemExit(1)
