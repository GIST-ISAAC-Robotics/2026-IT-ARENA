"""저장 scan의 차단점을 재구성하여 평가 전용 형상에 대조. 제어기 command 호출 없음."""
from pathlib import Path
import json
import math
import sys
import numpy as np
from shapely.affinity import rotate, translate
from shapely.geometry import Point, box

A = Path(__file__).resolve().parent.parent
ROOT = A.parents[3]
sys.path[:0] = [str(A), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.bypass_entry_trial import current_outer_wall_padding
from arena_autonomy.local_path import swept_limit
from build_experimental_track import obstacle_polygons
from facility_geometry import load, slab_projection


def lines(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def main():
    source = A/'run1/follow_decisions.jsonl'
    raw = source.read_bytes().splitlines(keepends=True)
    rows = [json.loads(line) for line in raw]
    header = rows[0]['data']
    motion, config = MotionConfig(**header['motion_config']), FollowConfig(**header['follow_config'])
    calls = [row for row in rows if row['kind']=='call' and row['data']['compute']['stage']=='plan']
    end = json.loads((A/'run1/report.json').read_bytes())['case_window']['end_sim_s']
    stall = [row for row in calls if row['data']['output'][2]['reason']=='moving_pass_current_path_unavailable'
             and row['data']['input']['reference_s'] <= end
             and row['data']['output'][2].get('moving_pass',{}).get('entry',{}).get('candidates')]
    # r513: 후보가 없는 정지(entry_guide_current_wall_too_short)는 점유 재구성 대상이 아니다.
    last = stall[-2:]
    ids = {row['seq'] for row in last}
    (A/'failure_inputs/last_stall_records.jsonl').write_bytes(b''.join(line for line,row in zip(raw,rows) if row['seq'] in ids))
    selected = [stall[0], *last]
    ego, actor = lines(A/'run1/ego_poses.jsonl'), lines(A/'run1/actor_poses.jsonl')
    fixture = ROOT/'artifacts/validation/2026-11-13/old_signal_610_r326_v1'
    locations = json.loads((fixture/'fixture.json').read_bytes())['locations']
    triangles = load(fixture/'support_frame_m.stl')[2]
    # C1 scan 높이 주변의 시설 단면만 평가에 사용한다.
    shape = slab_projection(triangles, .1345, .1347)
    facilities = [(p['name'],translate(rotate(shape,p['yaw'],origin=(0,0),use_radians=True),p['x'],p['y'])) for p in locations]
    walls = obstacle_polygons(ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf')
    result = []
    for row in selected:
        data = row['data']
        inp, scan, history = restore_call(data['input'], motion)
        t = inp['reference_s']
        scene = motion_scene(scan,history,t,timing_verified=inp['timing_verified'],epoch=inp['epoch'],
            rear_half_angle_deg=inp['rear_half_angle_deg'],timing_policy=inp.get('timing_policy','verified_only'))
        entry = data['output'][2]['moving_pass']['entry']
        candidate = entry['candidates'][0]
        side = candidate['current_outer_wall']
        padding, classification = current_outer_wall_padding(scene,config,side,entry['reference']['walls'][side])
        points = np.asarray(scene.occupied_endpoints_xy_m,dtype=float)
        ray_ids = [ray.source_index for ray in scene.rays if ray.status=='return']
        assert len(points)==candidate['occupied_points']
        steer = candidate['steering_rad']
        clearance = swept_limit(points,steer,config.wheelbase_m,horizon=2.5,margin=padding,length=config.length_m,width=config.width_m)
        assert abs(clearance-candidate['command_clearance_m'])<1e-12
        curvature = math.tan(steer)/config.wheelbase_m
        hit_ids, hit_distance = [], None
        for distance in np.arange(0.,2.501,.04):
            yaw = curvature*distance
            c,s = math.cos(yaw),math.sin(yaw)
            rx,ry = (distance-config.wheelbase_m/2,0.) if abs(curvature)<1e-5 else (s/curvature-config.wheelbase_m/2,(1-c)/curvature)
            dx,dy = points[:,0]-rx,points[:,1]-ry
            xx,yy = c*dx+s*dy-config.wheelbase_m/2,-s*dx+c*dy
            hits = np.flatnonzero((abs(xx)<=config.length_m/2+padding)&(abs(yy)<=config.width_m/2+padding))
            if len(hits):
                hit_ids=hits.tolist();hit_distance=float(distance);break
        ep=min(ego,key=lambda p:abs(p[0]-t));ap=min(actor,key=lambda p:abs(p[0]-t))
        actor_shape=translate(rotate(box(-.1,-.075,.1,.075),ap[3],origin=(0,0),use_radians=True),ap[1],ap[2])
        def point_record(index):
            p=points[index];c,s=math.cos(ep[3]),math.sin(ep[3])
            world=[ep[1]+c*p[0]-s*p[1],ep[2]+s*p[0]+c*p[1]]
            q=Point(world)
            distances={'actor':q.distance(actor_shape),'wall':min(q.distance(w) for w in walls),
                **{'facility_'+name:q.distance(shape) for name,shape in facilities}}
            return dict(occupied_index=index,source_ray_index=ray_ids[index],point_base_xy_m=p.tolist(),
                range_from_base_m=float(np.linalg.norm(p)),padding_m=float(padding[index]),world_xy_m=world,
                evaluator_distance_to_shape_m=distances,nearest_evaluator_shape=min(distances,key=distances.get))
        nearest_index=int(np.argmin(np.linalg.norm(points,axis=1)))
        result.append(dict(seq=row['seq'],reference_s=t,ego_pose=ep,actor_pose=ap,
            nearest_occupied_point=point_record(nearest_index),first_command_hit_distance_m=hit_distance,
            reproduced_command_clearance_m=clearance,recorded_command_clearance_m=candidate['command_clearance_m'],
            first_command_hit_count=len(hit_ids),first_command_hit_points=[point_record(i) for i in hit_ids[:8]],
            occupied_count=len(points),padding_policy=classification))
    output=dict(scope='stored scan/history motion_scene and recorded command occupancy reconstruction only; no controller command call or simulator',
        last_stall_sequences=sorted(ids),records=result,
        attribution_limit='Nearest evaluator shape is an association inference, not a sensor identity proof; points are motion-corrected with measured history and mapped using nearest truth pose.',
        facility_slice_m=[.1345,.1347],threshold_changes=False)
    with (A/'failure_inputs/occupied_attribution.json').open('x',encoding='utf-8') as f:json.dump(output,f,ensure_ascii=False,indent=2)
    print(json.dumps([dict(seq=r['seq'],reference_s=r['reference_s'],clearance=r['reproduced_command_clearance_m'],
        first_hit_count=r['first_command_hit_count'],first_hit_nearest_shapes=[p['nearest_evaluator_shape'] for p in r['first_command_hit_points']]) for r in result]))


if __name__=='__main__':main()
