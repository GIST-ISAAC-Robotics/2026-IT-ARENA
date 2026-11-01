"""포화된 벽 탐색 예산만 한 번 보충하는 진단. 실제 제어 소스에는 연결하지 않는다."""
from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy import local_follow as module
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.lidar_motion import MotionConfig

original=module.local_path
config=None
activity=Counter()


def remaining(path):
    rear=np.asarray(path).copy();rear[:,0]+=config.wheelbase_m/2
    nearest=int(np.argmin(np.linalg.norm(rear,axis=1)))
    return float(np.linalg.norm(np.diff(rear[nearest:],axis=0),axis=1).sum())


def extended(points,side,offset=.425,horizon=1.8):
    base=original(points,side,offset,horizon)
    if horizon<=1.8 or base[0] is None:return base
    if base[1]['boundary_observed_ahead_m']<horizon-1e-8:
        activity['visible_wall_ends_before_budget']+=1;return base
    length=remaining(base[0]);deficit=horizon-length
    if deficit<=config.measurement_padding_m:
        activity['path_already_long_enough']+=1;return base
    extra=min(abs(offset)*math.pi/2,deficit)
    candidate=original(points,side,offset,horizon+extra)
    if candidate[0] is None:
        activity['extended_shape_failed']+=1;return base
    if not module.path_agrees_arc(base[0],candidate[0],config.boundary_agreement_m):
        activity['near_path_changed']+=1;return base
    if remaining(candidate[0])<=length+config.measurement_padding_m:
        activity['no_useful_path_growth']+=1;return base
    activity['extended_path_used']+=1
    return candidate


if __name__=='__main__':
    source=HERE.parent/'solo_corridor_arc_v1/run1/follow_decisions.jsonl'
    metrics=json.loads((HERE.parent/'solo_corridor_arc_v1/run1_metrics.json').read_bytes())
    begin=metrics['first_green_sample_s'];end=begin+metrics['lap_s_from_first_green_sample']
    hashes={p.relative_to(ROOT).as_posix():sha256(p.read_bytes()).hexdigest()
            for p in [Path(__file__).resolve(),*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]}
    input_hash=sha256(source.read_bytes()).hexdigest()
    counts=Counter();changes=[];last_seq=-1;footer=None
    try:
        with source.open(encoding='utf-8') as stream:
            for line in stream:
                e=json.loads(line);d=e['data'];kind=e['kind']
                if e['seq']!=last_seq+1 or footer is not None:raise ValueError('trace ordering')
                last_seq=e['seq']
                if kind=='header':
                    config=module.FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
                    old=module.LocalFollow(config);new=module.LocalFollow(config)
                elif kind=='reset':old.reset();new.reset()
                elif kind=='footer':footer=d
                elif kind=='call':
                    module.local_path=original;a=replay_call(old,d['input'],motion)
                    if not equal_output(comparable_decision(freeze(a)),comparable_decision(d['output'])):
                        raise ValueError('baseline mismatch '+str(last_seq))
                    counts['baseline_matched']+=1
                    module.local_path=extended;b=replay_call(new,d['input'],motion)
                    t=d['input']['reference_s'];dt=d['input']['dt_s']
                    if not begin<=t<=end:continue
                    counts['lap_calls']+=1
                    counts['baseline_target_integral_m']+=a[0]*dt;counts['candidate_target_integral_m']+=b[0]*dt
                    counts['baseline_target_ge_2p6_s']+=dt*(a[0]>=2.6);counts['candidate_target_ge_2p6_s']+=dt*(b[0]>=2.6)
                    counts['baseline_peak_mps']=max(counts['baseline_peak_mps'],a[0]);counts['candidate_peak_mps']=max(counts['candidate_peak_mps'],b[0])
                    if abs(a[0]-b[0])>1e-8 or abs(a[1]-b[1])>1e-8:
                        counts['commands_changed']+=1;counts['new_zero']+=int(a[0]>.001>=b[0]);counts['zero_released']+=int(b[0]>.001>=a[0])
                        changes.append(dict(seq=last_seq,t=t,measured=d['input']['speed_mps'],old_speed=a[0],new_speed=b[0],
                            old_steering=a[1],new_steering=b[1],old_reason=a[2].get('reason'),new_reason=b[2].get('reason')))
                else:raise ValueError('unknown event')
    finally:module.local_path=original
    if (footer is None or footer['written']!=last_seq or footer['accepted']!=last_seq or footer['worker_alive'] is not False
            or any(footer[k] for k in ('failed','dropped','capture_errors','pending'))):raise ValueError('incomplete trace')
    unchanged=all(sha256((ROOT/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
    result=dict(scope='직전 완주 입력을 고정한 벽 예산 보완 진단. 새 실제 경로/랩 예측이 아니며 운영 연결 없음.',
                window_s=[begin,end],counts=dict(counts),activity=dict(activity),changes=changes,
                source_sha256=hashes,source_unchanged=unchanged,input_sha256=input_hash,
                input_unchanged=input_hash==sha256(source.read_bytes()).hexdigest())
    with (HERE/'shadow.json').open('x',encoding='utf-8') as stream:json.dump(freeze(result),stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(freeze({k:result[k] for k in ('counts','activity','source_unchanged','input_unchanged')}),ensure_ascii=False))
    if not unchanged or not result['input_unchanged']:raise SystemExit(1)
