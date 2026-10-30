"""실제 새 선택 정책과 오프라인 v2의 저장 입력 명령·선택 이유·확인 보존 비교."""
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
spec=importlib.util.spec_from_file_location('_offline_body_connection',BASE.parent/'fresh_confirmation_body_v2/candidate_state.py')
offline=importlib.util.module_from_spec(spec);spec.loader.exec_module(offline)


def compare(path):
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    calls=matched=0; footer=None; examples=[]
    with path.open(encoding='utf-8') as f:
        header=json.loads(next(f))['data']; motion=MotionConfig(**header['motion_config'])
        cfg=replace(FollowConfig(**header['follow_config']),rear_lane_keep_trial=True,rear_lane_keep_policy='observed_measured_v1')
        old=LocalFollow(cfg);old.parallel_trial=offline.FreshMemoryParallelTrial(cfg);old.reset()
        new=LocalFollow(replace(cfg,rear_lane_keep_policy='observed_fresh_v2'))
        for seq,line in enumerate(f,1):
            event=json.loads(line); assert event['seq']==seq and footer is None
            if event['kind']=='reset': old.reset();new.reset();continue
            if event['kind']=='footer':footer=event['data'];continue
            if event['kind']!='call':continue
            data=event['data']['input']
            a,b=[freeze(replay_call(t,data,motion)) for t in (old,new)]
            def decision(x):
                return [*x[:2],x[2].get('reason'),x[2].get('selector_choice'),
                        (x[2].get('fresh_confirmation_memory') or {}).get('kept')]
            same=equal_output(decision(a),decision(b));calls+=1;matched+=same
            if not same and len(examples)<4:examples.append(dict(t=data['reference_s'],offline=decision(a),runtime=decision(b)))
    return dict(input=str(path.relative_to(ROOT)),input_sha256=digest,calls=calls,matched=matched,examples=examples,
                passed=bool(footer) and not footer['capture_errors'] and calls==matched and digest==hashlib.sha256(path.read_bytes()).hexdigest())


if __name__=='__main__':
    paths=['rear_lane_measured_right_gazebo_v1/drive_right_on/follow_decisions.jsonl',
           'fresh_confirmation_body_v2/probe_combined/B_gazebo_right_trace.jsonl',
           'fresh_confirmation_body_v2/probe_combined/C_lead_stop_left_trace.jsonl']
    report=[compare(BASE.parent/p) for p in paths]
    with Path(sys.argv[1]).open('x',encoding='utf-8') as f:json.dump(report,f,indent=2)
    print(json.dumps(report));raise SystemExit(0 if all(x['passed'] for x in report) else 1)
