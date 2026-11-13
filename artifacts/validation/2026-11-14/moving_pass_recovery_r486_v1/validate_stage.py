"""r478 원 scan/운동과 원 tracker 관측을 이용하는 국소 guide 비교. 새 폐루프 주행 아님."""
from pathlib import Path
from types import SimpleNamespace
import copy
import json
import math
import sys

HERE=Path(__file__).resolve().parent
ORIGINAL=HERE.parent/'traffic_current_route_r478_v1'
FROZEN=ORIGINAL/'analysis_v1/frozen_source'
sys.path.insert(0,str(HERE/'stage/src/arena_autonomy'))
import arena_autonomy
arena_autonomy.__path__.append(str(FROZEN/'src/arena_autonomy/arena_autonomy'))
sys.path.insert(0,str(ORIGINAL/'analysis_v1'))
from inspect_failure import restore_scene
from arena_autonomy.follow_trace import freeze
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.moving_pass import MovingPassGuide,MovingTargetTrack
import numpy as np

ROWS=[json.loads(line) for line in (ORIGINAL/'run1/follow_decisions.jsonl').read_text(encoding='utf8').splitlines()]
HEADER=next(r['data']['header'] for r in ROWS if r['kind']=='runtime_evaluator_header')
CFG=FollowConfig(**HEADER['follow_config'])
MOTION=MotionConfig(**HEADER['motion_config'])


def captured(seq):
    row=next(r for r in ROWS if r['kind']=='call' and r['seq']==seq)
    d,scan,history,scene=restore_scene(row['data']['input'],MOTION)
    return row,d,scan,history,scene


def observed_tracker(guide,row,scene):
    detail=copy.deepcopy(row['data']['output'][2]['moving_pass']['target'])
    guide.tracker.detail=detail
    guide.tracker.lost_advantage_scans=detail.get('lost_advantage_scans',0)
    guide.tracker.speed_upper=detail.get('observed_speed_upper_mps')
    guide.tracker.speed_lower=detail.get('observed_speed_lower_mps')
    bounds=detail.get('observed_bounds_xy')
    if bounds:
        points=np.asarray(scene.occupied_endpoints_xy_m)
        selected=np.all((points>=np.asarray(bounds[0])-1e-10)&(points<=np.asarray(bounds[1])+1e-10),axis=1)
        guide.tracker.current=points[selected].copy()
        guide.tracker.reference=scene.t_ref_s
        guide.tracker.source=scene.source_first_s
        guide.tracker.epoch=scene.source_epoch
    guide.tracker.rear_count=detail.get('rear_confirmations',0)


def at_entry(seq):
    row,d,scan,history,scene=captured(seq)
    result=copy.deepcopy(row['data']['output'][2]['moving_pass']['entry'])
    guide=MovingPassGuide(CFG,MovingTargetTrack(CFG,'medium_v1'))
    guide.raw_scan=scan
    candidate=result['candidates'][0]
    guide.path=np.asarray(candidate['path_points'])
    guide.offset=candidate['offset_m'];guide.transition=candidate['transition_m']
    guide.side='left';guide.phase='entry'
    guide.reference_s=scene.t_ref_s;guide.last_source=scene.source_first_s;guide.epoch=scene.source_epoch
    wall=result['reference']['walls'][guide.side]
    x=np.asarray(wall['observed_x_m']);guide.wall_line=np.c_[x,wall['slope']*x+wall['intercept_m']]
    guide.wall_confirmations=result['outer_wall_fresh_confirmations']
    guide.intent.sign=result['intent_direction']
    observed_tracker(guide,row,scene)
    return guide,row,d,scan,history,scene,result


def run():
    records=[]
    for seq in (524,560,563):
        guide,row,d,scan,history,scene,result=at_entry(seq)
        output=guide._current_follow_return(scene,history,d['speed_mps'],d['previous_steering_rad'],d['dt_s'],result)
        record={'seq':seq,'actual_speed_mps':d['speed_mps'],'actual_previous_steering_rad':d['previous_steering_rad'],
            'changed':output is not result,'phase':guide.phase,'follow_headway':guide.follow_headway,'result':freeze(output)}
        records.append(record)
        print(seq,'changed',record['changed'],'phase',guide.phase,'selected',output.get('selected'),flush=True)
    guide,row,d,scan,history,scene,result=at_entry(524)
    first=guide._current_follow_return(scene,history,d['speed_mps'],d['previous_steering_rad'],d['dt_s'],result)
    sequence=[{'seq':524,'selected':first.get('selected'),'phase':guide.phase,'headway':guide.follow_headway}]
    for seq in (527,530,533,536):
        row,d,scan,history,scene=captured(seq);guide.raw_scan=scan
        observed_tracker(guide,row,scene)
        result=guide.evaluate(scene,history,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
        sequence.append({'seq':seq,'selected':result.get('selected'),'phase':guide.phase,
            'headway':guide.follow_headway,'result':freeze(result),'actual_speed_mps':d['speed_mps'],
            'actual_previous_steering_rad':d['previous_steering_rad']})
        print('sequence',seq,'phase',guide.phase,'headway',guide.follow_headway,'selected',result.get('selected'),
            'reason',result.get('reason'),flush=True)
    receipt={'scope':'원 실행의 guide 출력 상태와 tracker 현재 관측에서 국소 시작. 원 운동 입력은 바꾸지 않았으며 새 후보를 차량에 적용한 폐루프가 아님.',
        'representatives':records,'sequence':sequence}
    (HERE/'stage_probe_v2.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf8')


if __name__=='__main__':run()
