"""단독 선택 구성의 현재 관측 반대벽 속도 상한을 저장 입력에서만 비교한다."""
from pathlib import Path
import json
import sys
import types
import hashlib
import numpy as np

from arena_autonomy.follow_trace import replay_call, equal_output, comparable_decision, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow, FollowConfig

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
RUN=ROOT/'artifacts/validation/2026-11-02/official_facility_v1/run1'
SOURCE=ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py'


def main():
    source=SOURCE.read_text(encoding='utf-8')
    old="self.path_wall == side and front and all(r.status == 'return' for r in front)"
    new="(self.path_wall == side or cfg.solo_route_only) and front and all(r.status == 'return' for r in front)"
    assert source.count(old)==1
    candidate_source=source.replace(old,new)
    (OUT/'candidate_local_follow.py').write_text(candidate_source,encoding='utf-8')
    module=types.ModuleType('arena_autonomy._solo_alternative_candidate');sys.modules[module.__name__]=module
    exec(compile(candidate_source,str(OUT/'candidate_local_follow.py'),'exec'),module.__dict__)
    trajectory=[json.loads(line) for line in (RUN/'trajectory.jsonl').open()]
    t=np.array([r['time'] for r in trajectory]);s=np.array([r['progress_m'] for r in trajectory])
    baseline=candidate=motion=None;mismatch=[];changes=[];calls=0;by_segment={};reset_events=[]
    for line in (RUN/'follow_decisions.jsonl').open():
        event=json.loads(line);data=event['data']
        if event['kind']=='header':
            motion=MotionConfig(**data['motion_config'])
            baseline=LocalFollow(FollowConfig(**data['follow_config']))
            candidate=module.LocalFollow(module.FollowConfig(**data['follow_config']))
        elif event['kind']=='reset':
            baseline.reset();candidate.reset();reset_events.append(event['seq'])
        elif event['kind']=='call':
            calls+=1
            a=freeze(replay_call(baseline,data['input'],motion));b=freeze(replay_call(candidate,data['input'],motion))
            if not equal_output(comparable_decision(a),comparable_decision(data['output'])):mismatch.append(event['seq'])
            if a[:2]!=b[:2]:
                position=float(np.interp(data['input']['reference_s'],t,s))
                row=dict(seq=event['seq'],reference_s=data['input']['reference_s'],progress_m=position,
                    before=a[:2],after=b[:2],reason=b[2].get('reason'),preferred=data['input']['side'],
                    path_wall=b[2].get('path_wall'),boundary=b[2].get('boundary_selection'),
                    current=b[2].get('path_source_s')==b[2].get('source_first_s'),partial=b[2].get('partial_observation'),
                    clearance=b[2].get('swept_clearance_m'),remaining=b[2].get('observed_path_ahead_m'))
                changes.append(row)
                key=str(int(position)//2*2);by_segment[key]=by_segment.get(key,0)+1
    result=dict(scope='stored_input_only_not_closed_loop',calls=calls,baseline_mismatch=mismatch,
        output_changes=len(changes),by_2m_segment=by_segment,
        max_increase_mps=max((r['after'][0]-r['before'][0] for r in changes),default=0),
        before_hold_to_motion=sum(r['before'][0]==0 and r['after'][0]>0 for r in changes),
        steering_changed=sum(r['before'][1]!=r['after'][1] for r in changes),
        original_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),changes=changes)
    (OUT/'alternative.json').write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='changes'},ensure_ascii=False))
    return bool(mismatch)


if __name__=='__main__':raise SystemExit(main())
