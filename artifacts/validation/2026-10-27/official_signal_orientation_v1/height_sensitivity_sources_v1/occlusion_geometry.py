"""공식 입력의 고정 카메라·앞차 가림 비교. 정지 기하이며 검출/주행 결과 아님."""
from pathlib import Path
import hashlib
import itertools
import json
import math
import xml.etree.ElementTree as ET
import zipfile

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
ZIP = ROOT/'assets/track/official/v2026.10.24/it_arena_track_v2026.10.24.zip'


def rot(roll, pitch, yaw):
    cr,sr,cp,sp,cy,sy = math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),(-sp,cp*sr,cp*cr))


def mul(m, p):
    return tuple(sum(a*b for a,b in zip(row,p)) for row in m)


def inverse(m):
    return tuple(zip(*m))


def add(a,b):
    return tuple(x+y for x,y in zip(a,b))


def sub(a,b):
    return tuple(x-y for x,y in zip(a,b))


def segment_box(origin, target, slot, height):
    """두 점 사이 선분과 지상 OBB 내부의 교차. 끝점만 닿는 경우는 가림에서 제외."""
    matrix = inverse(rot(0,0,slot['yaw_rad']))
    o = mul(matrix,sub(origin,(slot['x'],slot['y'],0)))
    t = mul(matrix,sub(target,(slot['x'],slot['y'],0)))
    low,high = (-.10,-.075,0.),(.10,.075,height)
    enter,leave = 0.,1.
    for k in range(3):
        delta = t[k]-o[k]
        if abs(delta)<1e-12:
            if not low[k] <= o[k] <= high[k]:
                return False
        else:
            a,b = sorted(((low[k]-o[k])/delta,(high[k]-o[k])/delta))
            enter,leave = max(enter,a),min(leave,b)
    return enter < leave and leave > 1e-6 and enter < 1-1e-6


def camera(slot, mount):
    return (add((slot['x'],slot['y'],0),mul(rot(0,0,slot['yaw_rad']),mount['xyz'])),
            rot(0,-math.radians(mount['up_deg']),slot['yaw_rad']))


def pixel(target, origin, basis):
    x,y,z = mul(inverse(basis),sub(target,origin))
    if x <= 0:
        return None
    f = 640/math.tan(math.radians(75)/2)
    return (640-f*y/x,360-f*z/x)


def lamp_samples(lamp, origin, basis):
    # 正面 투영 원반 9점. 면적 비율/완전 가림의 엄밀한 증명은 아니다.
    c,r = lamp['xyz'],lamp['radius']
    axes = inverse(basis)
    return [c]+[add(c,add(tuple(v*r*.8*math.cos(a) for v in axes[1]),
                          tuple(v*r*.8*math.sin(a) for v in axes[2])))
                for a in [i*math.tau/8 for i in range(8)]]


def main():
    # 가림 기하 자체의 의미 있는 세 대조. 장애물 뒤/옆과 회전 좌표 확인.
    box = dict(x=1.,y=0.,yaw_rad=0.)
    assert segment_box((0,0,.1),(2,0,.1),box,.2)
    assert not segment_box((0,0,.3),(2,0,.3),box,.2)
    assert not segment_box((0,0,.1),(.8,0,.1),box,.2)
    assert segment_box((0,0,.1),(2,0,.1),dict(box,yaw_rad=math.pi/2),.2)
    with zipfile.ZipFile(ZIP) as archive:
        sdf = archive.read(next(n for n in archive.namelist() if n.endswith('world.sdf')))
        scene = json.loads(archive.read(next(n for n in archive.namelist() if n.endswith('scene.json'))))
    model = ET.fromstring(sdf).find("world/model[@name='it_arena_track_static']")
    lamps = []
    for link in model.findall('link'):
        if not link.get('name','').startswith('lamp_'):
            continue
        xyzrpy = list(map(float,link.findtext('pose','0 0 0 0 0 0').split()))
        visual = link.find('visual')
        vp = list(map(float,visual.findtext('pose','0 0 0 0 0 0').split()))
        lamps.append(dict(name=link.get('name'),
            xyz=add(xyzrpy[:3],mul(rot(*xyzrpy[3:]),vp[:3])),
            radius=float(visual.findtext('geometry/sphere/radius'))))
    lamps.sort(key=lambda l: (l['name']=='lamp_green',l['name']))
    mounts = [dict(name='current_horizontal',xyz=[.055,0,.145],up_deg=0.),
              dict(name='rear_20cm',xyz=[-.075,0,.20],up_deg=25.328),
              dict(name='rear_30cm',xyz=[-.075,0,.30],up_deg=21.910)]
    slots = sorted(scene['starting_grid']['slots'],key=lambda s:-s['s_m'])
    rows = []
    for mount,slot,height in itertools.product(mounts,slots,(.12,.25,.40)):
        origin,basis = camera(slot,mount)
        # 선두순 5대 배치. 마지막 칸이 자차이면 그 칸을 포함한 대표 5대로 바꾼다.
        field = slots[:5] if slot in slots[:5] else slots[:4]+[slot]
        blockers = [s for s in field if s is not slot]
        entries = []
        for lamp in lamps:
            targets = lamp_samples(lamp,origin,basis)
            uv = [pixel(p,origin,basis) for p in targets]
            obscured = [any(segment_box(origin,p,b,height) for b in blockers) for p in targets]
            self_blocked = [segment_box(origin,p,slot,.12) for p in targets]
            center = pixel(lamp['xyz'],origin,basis)
            entries.append(dict(name=lamp['name'],center_uv=center,
                all_samples_in_image=all(p is not None and 0<=p[0]<1280 and 0<=p[1]<720 for p in uv),
                all_samples_in_roi=all(p is not None and 0<=p[0]<1280 and 0<=p[1]<720*.58 for p in uv),
                blocked_samples=sum(obscured),self_blocked_samples=sum(self_blocked),sample_count=len(targets)))
        rows.append(dict(mount=mount['name'],grid=slot['index'],actor_height_m=height,
            field_slots=[s['index'] for s in field],lamps=entries,
            any_occlusion=any(e['blocked_samples'] or e['self_blocked_samples'] for e in entries),
            all_in_image=all(e['all_samples_in_image'] for e in entries),
            all_in_roi=all(e['all_samples_in_roi'] for e in entries),
            red_to_green_left_to_right=all(a['center_uv'] is not None and b['center_uv'] is not None and
                a['center_uv'][0]<b['center_uv'][0] for a,b in zip(entries,entries[1:]))))
    summary = [dict(mount=m['name'],actor_height_m=h,
        grids_in_image=sum(r['all_in_image'] for r in rows if r['mount']==m['name'] and r['actor_height_m']==h),
        grids_with_sampled_occlusion=[r['grid'] for r in rows if r['mount']==m['name'] and r['actor_height_m']==h and r['any_occlusion']],
        grids_red_to_green_order=[r['grid'] for r in rows if r['mount']==m['name'] and r['actor_height_m']==h and r['red_to_green_left_to_right']])
        for m,h in itertools.product(mounts,(.12,.25,.40))]
    result = dict(scope='five_box_cars_static_pinhole_sampled_occlusion_not_detection',
        sources={ZIP.relative_to(ROOT).as_posix():hashlib.sha256(ZIP.read_bytes()).hexdigest(),
                 Path(__file__).relative_to(ROOT).as_posix():hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
        mounts=mounts,summary=summary,rows=rows,
        limitations=['nominal base height zero and no suspension',
            'five cars represented by solid 20x15cm boxes, heights are assumptions',
            'ego box height .12m; actual boards/cables/brackets not included',
            'nine samples do not prove continuous disk visibility or detector success',
            'no track housing occlusion, optical distortion, exposure or rolling shutter'])
    with (OUT/'occlusion_geometry.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
