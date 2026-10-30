"""기존 중앙 정체 입력. 낮은 속도의 비접근 기하 가설만 비교하고 출력하지 않는다."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import zipfile
import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from geometry import compare
from arena_autonomy.bypass_entry_trial import EntryGuide,EntryConfig,current_outer_wall_padding,entry_motion_region
from arena_autonomy.follow_trace import restore_call,freeze,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.pass_release_trial import lateral_config,region_blocked,SelectorConfig
from arena_autonomy.m3_execution_trial import envelope_distance,ExecutionConfig

OLD=BASE.parent/'g2_center_offsets_v1'
spec=importlib.util.spec_from_file_location('existing_trace',OLD/'run.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
INPUT=OLD/'comparison1/existing_027/input_output.jsonl'


def main():
    previous=json.loads((OLD/'comparison1/report.json').read_text())
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in previous['source_sha256'].items())
    output=BASE/'analysis2';output.mkdir(exist_ok=False)
    before=hashlib.sha256(INPUT.read_bytes()).hexdigest()
    source={p.relative_to(ROOT).as_posix():p.read_bytes() for p in BASE.glob('*.py')}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for n,data in source.items():archive.writestr(n,data)
    counts=Counter();rows=[];seq=-1;footer=False;durations=[]
    for line in INPUT.open(encoding='utf-8'):
        e=json.loads(line);d=e['data'];assert not footer and e['seq']==seq+1;seq=e['seq']
        if e['kind']=='header':
            assert d['schema']==907
            config=FollowConfig(**d['config']);motion=MotionConfig(**d['motion'])
            kwargs=dict(d['entry']);kwargs['offsets_m']=tuple(kwargs['offsets_m']);kwargs['transitions_m']=tuple(kwargs['transitions_m'])
            guide=EntryGuide(config,EntryConfig(**kwargs),**d['guide_kwargs']);duration=d['duration_s']
        elif e['kind']=='call':
            inp,scan,history=restore_call(d['input'],motion)
            scene=motion_scene(scan,history,inp['reference_s'],timing_verified=inp['timing_verified'],
                epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
            result=guide.evaluate(scene,history,inp['speed_mps'],inp['previous_steering_rad'],inp['dt_s'])
            assert equal_output(old.comparable(freeze(result)),old.comparable(d['output']))
            counts['baseline_matched']+=1
            if (scene.t_ref_s>=.5+duration or len(result['candidates'])!=1
                    or result['candidates'][0]['reason']!='known_occupied_command_hold'):continue
            row=result['candidates'][0];points=np.asarray(scene.occupied_endpoints_xy_m)
            padding,classification=current_outer_wall_padding(scene,config,guide.side,result['reference']['walls'][guide.side])
            previous_steer=inp['previous_steering_rad'];dt=inp['dt_s'];speed=inp['speed_mps'];target=.10
            lo=max(-config.max_steering_rad,previous_steer-3*dt);hi=min(config.max_steering_rad,previous_steer+3*dt)
            steers=sorted({lo,hi,previous_steer,float(np.clip(0.,lo,hi)),row['steering_rad']})
            age=scene.t_ref_s-scene.source_first_s
            reaction_braking=envelope_distance(max(speed,target),age,dt,ExecutionConfig())
            # local_path.braking_speed의 잔여 60 mm도 점유 비교에 보존한다.
            distance=reaction_braking+.06
            started=time.perf_counter()
            prev=compare(points,padding,previous_steer,distance,config)
            trials=[]
            for steer in steers:
                geometry=compare(points,padding,steer,distance,config)
                region=entry_motion_region(scene,speed,target,steer,previous_steer,dt,config,lateral_config(config),padding)
                blocked=region_blocked(region,SelectorConfig())
                eligibility=geometry['sampled_geometry_candidate'] and prev['sampled_geometry_candidate'] and blocked is None
                trials.append(dict(steering_rad=steer,geometry=geometry,region=region,region_blocked=blocked,
                    sampled_combined_candidate=eligibility,
                    current_command_only_candidate=geometry['sampled_geometry_candidate'] and blocked is None))
            durations.append((time.perf_counter()-started)*1000)
            counts['occupied_hold_calls']+=1
            counts['any_current_command_only']+=any(t['current_command_only_candidate'] for t in trials)
            counts['any_combined_candidate']+=any(t['sampled_combined_candidate'] for t in trials)
            counts['current_uncertainty_overlap_calls']+=prev['current_uncertainty_overlap_points']>0
            counts['old_steering_nonapproach_reject']+=not prev['sampled_geometry_candidate']
            counts['any_base_collision']+=any(t['geometry']['known_base_collision_points']>0 for t in trials)
            rows.append(dict(reference_s=scene.t_ref_s,source_first_s=scene.source_first_s,measured_speed_mps=speed,
                target_mps=target,occupied_distance_m=distance,braking_residual_m=.06,original_command=d['command'],original_safe=d['safe'],
                original_steering_rad=row['steering_rad'],old_steering=prev,trials=trials))
        elif e['kind']=='footer':footer=True;assert d['calls']==counts['baseline_matched']
        else:raise ValueError('unexpected record')
    assert footer
    (output/'rows.json').write_text(json.dumps(freeze(rows),indent=2,allow_nan=False)+'\n',encoding='utf-8')
    ordered=sorted(durations)
    report=dict(counts=dict(counts),input_sha256=before,input_unchanged=hashlib.sha256(INPUT.read_bytes()).hexdigest()==before,
        baseline_source_matched=True,source_sha256={n:hashlib.sha256(v).hexdigest() for n,v in source.items()},
        source_unchanged=all((ROOT/n).read_bytes()==v for n,v in source.items()),
        processing_ms=dict(median=ordered[len(ordered)//2],p95=ordered[int(.95*(len(ordered)-1))],max=max(ordered)),
        braking_residual_m=.06,scope='offline sampled geometry only; no controller output/no new actuation/no continuous sweep or future motion proof')
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
