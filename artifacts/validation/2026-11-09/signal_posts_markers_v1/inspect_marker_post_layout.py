"""마커 옆 시험 지주가 실제 도로/벽과 겹치는지 사전 기하만 검사한다."""
import json,math,sys
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path.insert(0,str(R/'scripts'))
from surface_contact_eval import SurfaceMap,rectangle
W=R/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
s=SurfaceMap.from_world(W);out=[]
for marker,x,y,yaw in [(20,6.155,6.,0.),(30,2.48,3.505,math.pi/2)]:
    for upstream in (0.,.3725):
        for side in (-1,1):
            cx=x+upstream*math.cos(yaw)-side*.3725*math.sin(yaw)
            cy=y+upstream*math.sin(yaw)+side*.3725*math.cos(yaw)
            post=rectangle(cx,cy,yaw,.01462,.015);foot=rectangle(cx,cy,yaw,.05,.04)
            out.append(dict(id=marker,upstream=upstream,side=side,center=[cx,cy],
                post_road_area=post.intersection(s.road).area,foot_road_area=foot.intersection(s.road).area,
                post_wall_area=post.intersection(s.walls).area,foot_wall_area=foot.intersection(s.walls).area,
                ground=s.point_class(cx,cy)))
(A/'marker_post_layout_audit.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=1))
