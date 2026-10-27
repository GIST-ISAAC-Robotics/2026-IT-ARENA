"""보존 입력의 새 선택 출력 비교. 실제 변경 출력대로 주행한 폐루프 증거가 아니다."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[4]
for folder in ('src/arena_autonomy','src/arena_vehicle_interface','scripts'):
    sys.path.insert(0,str(ROOT/folder))
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene


def compare(path, output):
    events=[json.loads(s) for s in path.read_text(encoding='utf-8').splitlines()]
    header=events[0]['data']
    cfg=replace(FollowConfig(**header['follow_config']),parallel_hold_trial=True,pass_release_trial=True)
    motion=MotionConfig(**header['motion_config']);ctl=LocalFollow(cfg)
    rows=[]
    for event in events[1:]:
        if event['kind']=='reset':ctl.reset();continue
        if event['kind']!='call':continue
        data,scan,history=restore_call(event['data']['input'],motion)
        try:
            scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            new=ctl.command(scene,history,data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'],raw_scan=scan)
        except (MotionUnavailable,ValueError) as error:
            ctl.reset();new=(0.,0.,{'reason':'follow_input_stop','input_error':str(error)})
        old=event['data']['output'];d=new[2]
        rows.append(dict(seq=event['seq'],t=data['reference_s'],old_speed=old[0],speed=new[0],
            old_steer=old[1],steer=new[1],old_reason=old[2].get('selector_reason'),reason=d.get('selector_reason'),
            choice=d.get('selector_choice'),mode=d.get('selector_mode'),
            region=d.get('selector_region'),parallel=d.get('parallel_hold_trial')))
    summary=dict(scope='fixed_saved_input_not_closed_loop',input=str(path.relative_to(ROOT)),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),calls=len(rows),
        changed_commands=sum(abs(r['speed']-r['old_speed'])>1e-9 or abs(r['steer']-r['old_steer'])>1e-9 for r in rows),
        old_positive=sum(r['old_speed']>0 for r in rows),new_positive=sum(r['speed']>0 for r in rows),
        reasons=dict(Counter(r['reason'] for r in rows)),choices=dict(Counter(r['choice'] for r in rows)))
    output.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    output.with_suffix('.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows),encoding='utf-8')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('input',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.jsonl').exists():raise FileExistsError(args.output)
    print(json.dumps(compare(args.input.resolve(),args.output),ensure_ascii=False))
