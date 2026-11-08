"""첫 준비 시점의 명령 비교만 수행. ROS 발행·거짓 자유 공간·정답 위치 없음."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,pursue_path
from arena_autonomy.local_path import swept_limit,braking_speed
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.pass_release_trial import lateral_config,SelectorConfig,region_blocked,motion_region

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];PARENT=HERE.parent
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    intent=json.loads((PARENT/'p22_route_intent_v1/summary.json').read_bytes())
    allpaths=json.loads((PARENT/'p22_entry_arc_probe_v1/paths.json').read_bytes())
    result={}
    for name,record in allpaths.items():
        mode='strict' if name=='r11' else 'status_clock_proxy'
        goals=intent['runs'][name]['modes'][mode]['normal']['proposals']
        paths={r['t']:r for r in record['paths'] if r['t'] in goals and r['radius_m']==.36 and r['speed_mode']=='low'}
        assert len(paths)==1
        src=ROOT/record['source'];before=digest(src);assert before==record['sensor_sha256']
        results=[]
        for line in src.open():
            e=json.loads(line);d=e['data']
            if e['kind']=='header':mc=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config'])
            if e['kind']!='call' or d['input']['reference_s'] not in paths:continue
            raw,scan,h=restore_call(d['input'],mc)
            candidate=paths[raw['reference_s']]
            p=np.asarray(candidate['poses'])
            # Pure Pursuit가 사용하는 차체 중심 경로. 후륜 자세에서 정확히 바꾼다.
            path=p[:,:2]+cfg.wheelbase_m/2*np.column_stack((np.cos(p[:,2]),np.sin(p[:,2])))
            scene=motion_scene(scan,h,raw['reference_s'],timing_verified=raw['timing_verified'],epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
            age=scene.t_ref_s-scene.source_first_s
            cap,steer,remaining=pursue_path(path,raw['speed_mps'],cfg,age,speed_cap=.65)
            steer=float(np.clip(steer,raw['previous_steering_rad']-3*raw['dt_s'],raw['previous_steering_rad']+3*raw['dt_s']))
            points=np.asarray(scene.occupied_endpoints_xy_m)
            pad=per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
            def clear(pts,margin):
                return min(swept_limit(pts,a,cfg.wheelbase_m,horizon=2.5,margin=margin,length=cfg.length_m,width=cfg.width_m)
                           for a in (steer,raw['previous_steering_rad']))
            gap=clear(points,pad);target=min(.65,cap,braking_speed(gap,age))
            region=motion_region(scene,raw['speed_mps'],target,steer,raw['previous_steering_rad'],raw['dt_s'],cfg,lateral_config(cfg))
            reason=region_blocked(region,SelectorConfig()) or ('occupied' if target<=0 else 'candidate')
            obstacle_gap=clear(np.vstack((points,[.13,0.])),np.r_[pad,.025+cfg.measurement_padding_m])
            assert braking_speed(obstacle_gap,age)<=0
            results.append(dict(t=raw['reference_s'],mode=mode,measured_speed_mps=raw['speed_mps'],command_speed_mps=target,
                                steering_rad=steer,reason=reason,eligible=reason=='candidate',clearance_m=gap,remaining_m=remaining,
                                region=freeze(region),obstacle_counterexample_gap_m=obstacle_gap,
                                current_dynamic_motion_bound_mps=cfg.object_motion_bound_mps,current_fixed_padding_m=.025+cfg.measurement_padding_m))
        assert digest(src)==before and len(results)==1
        result[name]=dict(sensor_sha256=before,input_unchanged=True,commands=results)
    (HERE/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print(json.dumps(result))
if __name__=='__main__':main()
