"""공식 본선 접근과 고정 마운트 후보의 마커 기하. 인식 성공률/가림 완전 증명 아님."""
import bisect
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

from occlusion_geometry import OUT, ROOT, ZIP, rot, mul, add, sub, camera, pixel, segment_box


def route_function(rows, length):
    pts = [(float(r['x_m']),float(r['y_m'])) for r in rows]
    cumulative = [float(r['s_m']) for r in rows]
    if cumulative[-1] < length:
        pts.append(pts[0])
        cumulative.append(length)
    def pose(s):
        s %= cumulative[-1]
        i = min(len(pts)-2,bisect.bisect_right(cumulative,s)-1)
        a,b = pts[i],pts[i+1]
        ratio = (s-cumulative[i])/(cumulative[i+1]-cumulative[i])
        return dict(x=a[0]+ratio*(b[0]-a[0]),y=a[1]+ratio*(b[1]-a[1]),
                    yaw_rad=math.atan2(b[1]-a[1],b[0]-a[0]))
    return pose,cumulative[-1]


def main():
    with zipfile.ZipFile(ZIP) as archive:
        raw = archive.read(next(n for n in archive.namelist() if n.endswith('world.sdf')))
        scene = json.loads(archive.read(next(n for n in archive.namelist() if n.endswith('scene.json'))))
        route = list(csv.DictReader(archive.read(next(n for n in archive.namelist() if n.endswith('centerline.csv'))).decode().splitlines()))
    model = ET.fromstring(raw).find("world/model[@name='it_arena_track_static']")
    mounts = json.loads((OUT/'occlusion_geometry.json').read_text())['mounts']
    at,total = route_function(route,scene['track']['lap_length_m'])
    assert total == scene['track']['lap_length_m']
    rows = []
    for marker,mount,distance in itertools.product(scene['aruco_markers']['markers'],mounts,(.5,1.,1.5,2.)):
        link = model.find(f"link[@name='aruco_{marker['id']}']")
        p = list(map(float,link.findtext('pose').split()))
        basis = rot(*p[3:])
        normal = mul(basis,(1.,0.,0.))
        targets = [add(p[:3],mul(basis,(.0025,y,z))) for y,z in itertools.product((-.05,.05),repeat=2)]
        s = marker['s_m']-distance
        ego = at(s)
        origin,cb = camera(ego,mount)
        uv = [pixel(q,origin,cb) for q in targets]
        facing = sum(a*b for a,b in zip(normal,sub(origin,p[:3])))>0
        visible = facing and all(q is not None and 0<=q[0]<1280 and 0<=q[1]<720 for q in uv)
        min_edge = min(math.dist(uv[a],uv[b]) for a,b in ((0,1),(0,2),(1,3),(2,3))) if all(q is not None for q in uv) else None
        center = tuple(sum(q[k] for q in targets)/4 for k in range(3))
        samples = targets+[center]
        # 40cm 앞차 중심: 특정 주행정책의 선택 거리가 아니라 가림 대조 가정이다.
        lead = at(s+.4)
        occlusion = {str(height):sum(segment_box(origin,q,lead,height) for q in samples) for height in (.12,.25,.40)}
        rows.append(dict(marker_id=marker['id'],mount=mount['name'],route_distance_before_m=distance,
            full_board_in_image=visible,face_toward_camera=facing,min_board_edge_px=min_edge,
            uv=uv,ego=ego,camera_origin=origin,board_center=center,lead_center_gap_m=.4,
            lead_blocked_sample_count=occlusion,sample_count=5))
    summary = []
    for mount,marker in itertools.product(mounts,scene['aruco_markers']['markers']):
        matches = [r for r in rows if r['mount']==mount['name'] and r['marker_id']==marker['id']]
        summary.append(dict(mount=mount['name'],marker_id=marker['id'],
            full_board_distances_m=[r['route_distance_before_m'] for r in matches if r['full_board_in_image']],
            blocked_by_25cm_lead_distances_m=[r['route_distance_before_m'] for r in matches if r['lead_blocked_sample_count']['0.25']]))
    result = dict(scope='actual_course_marker_projection_and_five_ray_box_occlusion_no_detection_or_driving',
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ZIP,Path(__file__),OUT/'occlusion_geometry.py',OUT/'occlusion_geometry.json')},
        source_centerline_length_m=total,rows=rows,summary=summary,
        limitations=['official dense CSV pose is evaluator only; vehicle may follow a different actual trajectory',
            'no wall or board housing occlusion; pinhole optics',
            'board edge pixels are not ArUco black code size or detection guarantee',
            'solid 20x15cm front car at .4m center gap is an explicit hypothetical'])
    with (OUT/'marker_route_geometry.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
