"""같은 저장 센서 입력에서 경로만 평행 이동한 후보를 비교한다. 주행 허가는 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

from arena_autonomy.follow_trace import restore_call, replay_call
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.scene_motion import motion_scene

from probe_saved import CASES, ROOT, HERE
from hold_path_candidate import candidate


def one(name, relative):
    path=ROOT/relative
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    expected={r['seq']:r['bounded_local_v4'] for r in
              (json.loads(l) for l in (HERE/f'{name}_rows.jsonl').read_text().splitlines())}
    counts,threat_counts=Counter(),Counter()
    pairs=Counter()
    costs=[]
    calls=matched=0
    with path.open(encoding='utf-8') as stream,(HERE/f'run2_{name}_rows.jsonl').open('x',encoding='utf-8') as dest:
        for line in stream:
            e=json.loads(line);d=e['data']
            if e['kind']=='header':
                cfg=FollowConfig(**{**d['follow_config'],'partial_observation_policy':'bounded_local_v4'})
                ctl=LocalFollow(cfg);motion=MotionConfig(**d['motion_config'])
            elif e['kind']=='reset':ctl.reset()
            elif e['kind']=='call':
                base=replay_call(ctl,d['input'],motion)
                old=expected[e['seq']]
                matched+=int(base[:2]==tuple(old['command']) and base[2]['reason']==old['reason'])
                calls+=1
                data,scan,history=restore_call(d['input'],motion)
                try:
                    scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                        epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
                    start=time.perf_counter()
                    result=candidate(scene,base,data['speed_mps'],data['previous_steering_rad'],data['dt_s'],cfg)
                    costs.append((time.perf_counter()-start)*1000)
                except MotionUnavailable as error:
                    result={'eligible':False,'reason':'scene_'+str(error)}
                counts[result['reason']]+=1
                threats=(d['output'][2].get('m3_lateral') or {}).get('threats') or {}
                threat=any(v.get('present') for v in threats.values())
                if threat:
                    threat_counts[result['reason']]+=1
                    pairs[f'baseline_positive={base[0]>0},candidate={result["eligible"]}']+=1
                dest.write(json.dumps({'seq':e['seq'],'reference_s':data['reference_s'],
                    'stored_sensor_threat':threat,'baseline_command':base[:2],
                    'baseline_reason':base[2]['reason'],'candidate':result},allow_nan=False)+'\n')
    return {'calls':calls,'baseline_commands_matched':matched,'reasons':dict(counts),
        'threat_reasons':dict(threat_counts),'threat_pairs':dict(pairs),
        'maximum_candidate_ms':max(costs), 'input_sha256':digest,
        'input_unchanged':digest==hashlib.sha256(path.read_bytes()).hexdigest()}


if __name__=='__main__':
    target=HERE/'run2_compare.json'
    if target.exists():raise FileExistsError(target)
    files=sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+sorted(HERE.glob('*.py'))
    hashes=lambda:{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    before=hashes()
    results={k:one(k,v) for k,v in CASES.items()}
    result={'scope':'offline_path_choice_only_not_closed_loop_or_runtime_permission',
        'sources':before,'source_unchanged':before==hashes(),'results':results,
        'limitations':['current side occupancy and original padding/braking remain unchanged',
            'no persistent encounter state, independent protection, actuation or road/tire audit yet',
            '1 cm body-corner lateral budget is a trial classification, not rear visibility proof']}
    with target.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(results,indent=2))
    if not result['source_unchanged'] or any(r['calls']!=r['baseline_commands_matched'] or
                                            not r['input_unchanged'] for r in results.values()):
        raise SystemExit(1)
