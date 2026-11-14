"""서로 다른 분기의 Y갈림길 두 관측. 표준 라이브러리만, 제어기 실행 없음."""
from pathlib import Path
import hashlib,json,math
A=Path(__file__).resolve().parent;ROOT=A.parents[4]
def dump(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
def cross(a,b):return a[0]*b[1]-a[1]*b[0]
def sub(a,b):return (a[0]-b[0],a[1]-b[1])

def sensor(segments):
    origin=(.06,0.);ranges=[]
    for i in range(500):
        angle=-math.pi+i*math.tau/500;direction=(math.cos(angle),math.sin(angle));hits=[]
        for start,end in segments:
            delta=sub(end,start);denom=cross(direction,delta)
            if abs(denom)<1e-12:continue
            rel=sub(start,origin);distance=cross(rel,delta)/denom;fraction=cross(rel,direction)/denom
            if .05<=distance<=12. and 0<=fraction<=1:hits.append(distance)
        ranges.append(min(hits) if hits else None)
    return dict(ranges=ranges,angle_min=-math.pi,angle_increment=math.tau/500,range_min=.05,range_max=12.,
        acquired_at_s=[.6+i*.0002 for i in range(500)],t_ref_s=.7,source_epoch=0,source_frame_id='laser_frame',
        sensor_xy_m=list(origin),sensor_yaw_rad=0.,rear_half_angle_deg=0.,
        null_range_meaning='UNKNOWN/no finite hit; never free')

def main():
    # Approach walls bend outward by45deg atx=-.15. A center island separates the
    # branches. The side policy is left; both branch outlines remain evaluation-only.
    bend=-.15;offset=.505
    left=lambda x:(x,offset+x-bend)
    right=lambda x:(x,-offset-x+bend)
    common=[((-2.,offset),(bend,offset)),((-2.,-offset),(bend,-offset)),
            (right(bend),right(2.5)),((.70,0.),(2.5,1.5)),((.70,0.),(2.5,-1.5))]
    cases=[('preferred_continuous',common+[(left(bend),left(2.5))]),
           ('preferred_fragmented',common+[(left(bend),left(.05)),(left(.70),left(2.5))])]
    geometry=[]
    for name,segments in cases:
        raw=sensor(segments);dump(A/(name+'_scan.json'),raw)
        geometry.append(dict(name=name,segments=segments,assumed_not_measured=True,
            selected_wall_side='left',left_branch='positive_y',opposite_branch='negative_y',
            finite_returns=sum(v is not None for v in raw['ranges']),unknown_indices=[i for i,v in enumerate(raw['ranges']) if v is None]))
    dump(A/'evaluation_geometry.json',geometry)
    config=json.loads((ROOT/'artifacts/validation/2026-11-14/asymmetric_field_wall_r548_v1/report.json').read_text())['full_follow_config']
    assert config['max_steering_rad']==.28475436568260193
    dump(A/'follow_config.json',config)
    files=['src/arena_autonomy/arena_autonomy/current_wall_reacquire.py','src/arena_autonomy/arena_autonomy/local_path.py','src/arena_autonomy/arena_autonomy/local_follow.py','src/arena_autonomy/arena_autonomy/piecewise_wall.py']
    dump(A/'prepared_source_manifest.json',{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in files})
    print(json.dumps([dict(case=g['name'],returns=g['finite_returns'],unknown=len(g['unknown_indices'])) for g in geometry]))
if __name__=='__main__':main()
