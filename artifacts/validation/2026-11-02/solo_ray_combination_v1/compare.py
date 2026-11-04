"""저장 입력 전용 기존 먼 벽 연결 조합 비교. 런타임 파일/현재 Gazebo를 변경하지 않는다."""
from pathlib import Path
from hashlib import sha256
import csv
import inspect
import json
import math
import sys
import textwrap
from zipfile import ZipFile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import arena_autonomy.local_follow as original
from arena_autonomy.follow_trace import replay_call, equal_output, comparable_decision, freeze
from arena_autonomy.lidar_motion import MotionConfig

INPUT=HERE.parent/'solo_alternative_speed_v1/run1'


def variant():
    from dataclasses import replace
    class Candidate(original.LocalFollow):
        def __init__(self, config):
            assert config.solo_route_only and config.preview_above_base_speed and config.boundary_bridge
            super().__init__(replace(config, ray_wall_continuity=True))
    return Candidate


def run():
    source=Path(original.__file__)
    before=source.read_bytes()
    with ZipFile(HERE/'before.zip','x',compression=8) as archive:
        archive.writestr(source.relative_to(ROOT).as_posix(),before)
    candidate_type=variant()
    report=json.loads((INPUT/'report.json').read_bytes())
    actuation=json.loads((INPUT/'actuation_trace.json').read_bytes())
    begin=report['start_time_sim_s']
    end=next(r['now_us']*1e-6 for r in actuation if r.get('reason')=='software_stop')
    rows=[];baseline_mismatch=0;all_calls=0;footer=None
    previous={(kind,accel):0. for kind in ('base','candidate') for accel in (1.5,2.,2.5)}
    for line in (INPUT/'follow_decisions.jsonl').open(encoding='utf-8'):
        event=json.loads(line);data=event['data']
        if event['kind']=='header':
            cfg=original.FollowConfig(**data['follow_config'])
            motion=MotionConfig(**data['motion_config'])
            base=original.LocalFollow(cfg);candidate=candidate_type(cfg)
        elif event['kind']=='reset':
            base.reset();candidate.reset()
        elif event['kind']=='call':
            all_calls+=1
            a=freeze(replay_call(base,data['input'],motion))
            b=freeze(replay_call(candidate,data['input'],motion))
            baseline_mismatch+=int(not equal_output(comparable_decision(a),comparable_decision(data['output'])))
            t=data['input']['reference_s']
            row=dict(t=t,base_speed=a[0],candidate_speed=b[0],base_steer=a[1],candidate_steer=b[1],
                     base_reason=a[2].get('reason'),candidate_reason=b[2].get('reason'),
                     base_ahead=a[2].get('observed_path_ahead_m'),candidate_ahead=b[2].get('observed_path_ahead_m'))
            for kind,out in [('base',a),('candidate',b)]:
                curvature=abs(math.tan(out[1])/cfg.wheelbase_m)
                capped=min(out[0],math.sqrt(cfg.lateral_acceleration_mps2/curvature)) if curvature>1e-6 else out[0]
                for accel in (1.5,2.,2.5):
                    key=(kind,accel)
                    previous[key]=min(capped,previous[key]+accel*data['input']['dt_s'])
                    row[f'{kind}_ramp_{accel}']=previous[key]
            if begin<=t<end:rows.append(row)
        elif event['kind']=='footer':footer=data
    assert footer is not None and footer['dropped']==footer['failed']==footer['capture_errors']==0
    assert source.read_bytes()==before
    with (HERE/'comparison.csv').open('x',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    metrics={}
    for key in ['base_speed','candidate_speed',*[f'{k}_ramp_{a}' for k in ('base','candidate') for a in (1.5,2.,2.5)]]:
        metrics[key]=dict(maximum_mps=max(r[key] for r in rows),zero_sample_hold_s=.02*sum(r[key]<=1e-9 for r in rows),
                         above_2p5_sample_hold_s=.02*sum(r[key]>=2.5 for r in rows))
    result=dict(candidate='현재 단독 preview/bridge 구성에 기존 ray_wall_continuity만 추가',scope='동일 저장 입력의 반사실 후보. 새 관측 생성/실제 가속/랩 시간/폐루프 성공 아님. 부모 램프2.0/2.5도 계산만 하며 설정 미변경.',
        input_sha256=sha256((INPUT/'follow_decisions.jsonl').read_bytes()).hexdigest(),
        source_sha256=sha256(before).hexdigest(),original_source_unchanged=True,
        all_calls=all_calls,window=[begin,end],window_calls=len(rows),baseline_mismatches=baseline_mismatch,
        new_zero_calls=sum(r['base_speed']>0 and r['candidate_speed']<=0 for r in rows),
        released_zero_calls=sum(r['base_speed']<=0 and r['candidate_speed']>0 for r in rows),
        maximum_steering_difference_rad=max(abs(r['base_steer']-r['candidate_steer']) for r in rows),metrics=metrics)
    with (HERE/'summary.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False))
    return 0 if baseline_mismatch==0 else 1


if __name__=='__main__':
    raise SystemExit(run())
