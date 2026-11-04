"""다음 1랩 녹화의 외부 관찰 카메라 구도 사전 점검. 평가 자세는 제어에 사용하지 않는다."""
import math
import json
import hashlib
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from shapely.geometry import LineString
from shapely.prepared import prep
from surface_contact_eval import SurfaceMap

RUN=HERE.parent/'solo_post_free_v1/run1'


def evaluate(track, rows, x, z, pitch):
    width,height,hfov=960,540,1.35
    f=(width/2)/math.tan(hfov/2)
    half=math.atan(height/(2*f))
    def pixel(u,h):
        return height/2+f*math.tan(math.atan2(z-h,u-x)-pitch)
    ground=dict(near_x_m=x+z/math.tan(pitch+half),
                far_x_m=x+z/math.tan(pitch-half) if pitch>half else None,
                distance_3m_v_px=pixel(3.,0),distance_5m_v_px=pixel(5.,0),
                rear_tire_v_px=pixel(-.10,.01),front_deck_v_px=pixel(.10,.06))
    bad=0
    for r in rows:
        X,Y,theta=r['x'],r['y'],r['yaw']
        c,s=math.cos(theta),math.sin(theta)
        bad+=track.intersects(LineString([(X+x*c,Y+x*s),(X,Y)]))
    return dict(pose_in_model=[x,0,z,0,pitch,0],pitch_down_deg=math.degrees(pitch),
                ground=ground,wall_xy_hits_between_camera_and_car=bad,poses=len(rows))


def main():
    world=RUN/'runtime_world.sdf'
    rows=[json.loads(s) for s in (RUN/'trajectory.jsonl').open(encoding='utf8')]
    track=prep(SurfaceMap.from_world(world).walls)
    out=dict(previous=evaluate(track,rows,-.32,.38,.72),forward=evaluate(track,rows,-.52,.30,.30),
             input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [world,RUN/'trajectory.jsonl']},
             scope='수평면의 핀홀 투영과 기존 한 랩의 카메라-자차 중앙선 XY 벽 교차 표본. 표지판/상대차/차체 모서리/실제 픽셀 가림·연속 시각 완전성은 판정하지 않는다. far=null은 하늘선 포함이며 트랙이 무한히 보인다는 뜻이 아니다.')
    with (HERE/'chase_projection.json').open('x',encoding='utf8') as f:
        json.dump(out,f,indent=2,ensure_ascii=False,allow_nan=False)
    print(out['previous'],out['forward'])


if __name__=='__main__':
    main()
