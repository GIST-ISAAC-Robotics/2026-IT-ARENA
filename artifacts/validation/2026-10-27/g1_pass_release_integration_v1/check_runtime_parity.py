"""선택형 실제 LocalFollow 연결 출력과 보존한 시제품 선택 출력의 수치/상태 대조."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
for part in ('src/arena_autonomy','src/arena_vehicle_interface','scripts'):
    sys.path.insert(0,str(ROOT/part))
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig,MotionUnavailable
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene


def run(name):
    raw=ROOT/'artifacts/validation/2026-10-27/g1_parallel_lane_v1'/name/'follow_decisions.jsonl'
    expected_path=ROOT/'artifacts/validation/2026-10-27/g1_pass_release_selector_v1/run3_replay'/f'{name}_rows.jsonl'
    expected={r['seq']:r for r in (json.loads(s) for s in expected_path.read_text().splitlines())}
    events=[json.loads(s) for s in raw.read_text().splitlines()]
    header=events[0]['data'];motion=MotionConfig(**header['motion_config'])
    cfg=replace(FollowConfig(**header['follow_config']),parallel_hold_trial=True,pass_release_trial=True)
    ctl=LocalFollow(cfg);mismatches=[];calls=0
    for event in events[1:-1]:
        if event['kind']=='reset':ctl.reset();continue
        data,scan,history=restore_call(event['data']['input'],motion)
        try:
            scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            out=ctl.command(scene,history,data['speed_mps'],data['side'],data['previous_steering_rad'],
                            data['dt_s'],raw_scan=scan)
        except (MotionUnavailable,ValueError) as error:
            mismatches.append({'seq':event['seq'],'error':str(error)});ctl.reset();continue
        calls+=1;e=expected[event['seq']];d=out[2]
        actual=(out[0],out[1],d.get('selector_choice'),d.get('selector_mode',d.get('selector_mode_before')),d.get('selector_reason'))
        target=(e['out']['speed'],e['out']['steer'],e['choice'],e['mode'],e['reason'])
        if abs(actual[0]-target[0])>1e-12 or abs(actual[1]-target[1])>1e-12 or actual[2:]!=target[2:]:
            mismatches.append({'seq':event['seq'],'actual':actual,'expected':target})
    return {'calls':calls,'mismatch_count':len(mismatches),'mismatches':mismatches,'passed':not mismatches}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    result={n:run(n) for n in ('run1_left','run2_right')}
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({n:{k:v for k,v in r.items() if k!='mismatches'} for n,r in result.items()}))
    sys.exit(0 if all(r['passed'] for r in result.values()) else 1)
