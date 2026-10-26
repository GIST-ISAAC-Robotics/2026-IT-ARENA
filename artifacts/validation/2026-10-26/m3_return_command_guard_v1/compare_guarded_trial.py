"""선택형 복귀 관문 OFF 동일성·ON 저장 입력 영향. 고정 입력이며 실제 궤적을 바꾸지 않는다."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]; OLD=HERE.parent/'m3_rear_residual_v1'
sys.path[:0]=[str(ROOT/p) for p in ('src/arena_autonomy','src/arena_vehicle_interface','tests')]
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig
from arena_autonomy.m3_guarded_return_trial import GuardedReturnTrial


def job(args):
    path_text,enabled=args;path=ROOT/path_text;raw=path.read_bytes()
    events=[json.loads(x) for x in raw.decode('utf-8').splitlines()];head=events[0]['data']
    ctl=GuardedReturnTrial(FollowConfig(**head['follow_config']),ExecutionConfig(**head['execution_config']),
                          enabled=enabled,allow_simulation_trial=True)
    motion=MotionConfig(**head['motion_config']);name=path.parent.parent.name
    expected=json.loads((OLD/'cachefix_runs/run7_premises_cachefix'/f'{name}__plain.json').read_text())
    differences=[];counts=Counter();rows=[]
    for i,event in enumerate(events[1:-1]):
        if event['kind']=='reset':
            ctl.reset();counts['reset_events']+=1
            if expected[i]!={'reset':True}:differences.append(i)
            continue
        data,scan,history=restore_call(event['data']['input'],motion)
        try:
            scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                               epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            out=ctl.command(scene,history,data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'])
        except (ValueError,MotionUnavailable) as error:
            ctl.reset();out=(0.,0.,{'reason':'follow_input_stop','behavior':'HOLD',
                                  'input_error':str(error),'mode':'m3_execution_trial'})
        counts['decision_calls']+=1
        digest=hashlib.sha256(json.dumps(freeze(out),sort_keys=True).encode()).hexdigest()
        if digest!=expected[i]['sha']:differences.append(i)
        lat=out[2].get('m3_lateral') or {};gate=lat.get('release_gate') or {}
        counts['actual_releases']+=bool(lat.get('released_to_baseline'))
        counts['baseline_command_rejected']+=gate.get('reason')=='baseline_command_unconfirmed'
        counts['post_release_guard_calls']+='m3_return_guard' in out[2]
        rows.append({'i':i,'t':data['reference_s'],'speed':out[0],'steering':out[1],
                     'reason':out[2]['reason'],'mode':lat.get('mode'),'sha':digest})
    return name+('__on' if enabled else '__off'),rows,{
        **dict(counts),'output_differences':len(differences),'first_difference':differences[:1],
        'input_sha256':hashlib.sha256(raw).hexdigest(),'input_unchanged':path.read_bytes()==raw}


def main(out,workers):
    if out.exists():raise FileExistsError(out)
    files=list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+[Path(__file__)]
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    paths=list(json.loads((OLD/'run2_plan.json').read_text()));results={};rows={}
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for name,data,summary in pool.map(job,[(p,e) for p in paths for e in (False,True)]):
            results[name],rows[name]=summary,data
            print(json.dumps({name:summary}),flush=True)
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    out.mkdir(parents=True)
    report={'scope':'same stored input, optional pure controller only; no ROS/Gazebo',
            'source_sha256':hashes,'source_unchanged':unchanged,'results':results,
            'limits':['ON changes are diagnostic; saved vehicle trajectory does not respond to them.',
                      'If no release is reached, this test does not prove guard intervention.']}
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    (out/'rows.json').write_text(json.dumps(rows)+'\n',encoding='utf-8')
    return 0 if unchanged and all(v['input_unchanged'] and (v['output_differences']==0 or k.endswith('__on')) for k,v in results.items()) else 1


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=5)
    a=p.parse_args();sys.exit(main(a.output,a.workers))
