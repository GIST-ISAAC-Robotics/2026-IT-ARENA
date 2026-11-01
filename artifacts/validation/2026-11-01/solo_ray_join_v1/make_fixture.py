"""회귀 고정본: 완주본의 실제 한 회전(16.92초) 끝점과 원시 광선 번호만 옮긴다."""
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene

source=HERE.parent/'solo_corridor_arc_v1/run1/follow_decisions.jsonl'
with source.open(encoding='utf-8') as stream:
    for line in stream:
        e=json.loads(line);d=e['data']
        if e['kind']=='header':motion=MotionConfig(**d['motion_config'])
        if e['kind']!='call' or round(d['input']['reference_s'],2)!=16.92:continue
        inp,scan,history=restore_call(d['input'],motion)
        scene=motion_scene(scan,history,inp['reference_s'],timing_verified=inp['timing_verified'],
                           epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
        hits=[r for r in scene.rays if r.status=='return']
        data=dict(scope='solo_corridor_arc_v1 run1 t=16.92의 실제 반환. 입력 복사본이며 정답을 포함하지 않는다.',
                  side=inp['side'],horizon_m=d['output'][2]['observed_search_horizon_m'],
                  points=[[round(v,6) for v in r.hit_xy_m] for r in hits],rays=[r.source_index for r in hits])
        break
path=ROOT/'tests/fixtures/solo_ray_wall_scan.json'
path.write_text(json.dumps(data,ensure_ascii=False,indent=0),encoding='utf-8',newline='\n')
print(len(data['points']),path.stat().st_size)
