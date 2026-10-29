"""가로 배열에서 설치 높이 불확실성·카메라 위치·상대차 가림의 정지 기하 비교."""
from copy import deepcopy
import hashlib
import itertools
import json
from pathlib import Path
import sys
import zipfile

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
PREVIOUS=OUT.parent/'camera_mount_preparation_v1'
sys.path[:0]=[str(PREVIOUS),str(ROOT/'scripts')]
from occlusion_geometry import camera,pixel,lamp_samples,segment_box
from audit_start_signal_geometry import lamps_from_sdf


def main():
    source=ROOT/'assets/track/official/v2026.10.24/it_arena_track_v2026.10.24.zip'
    with zipfile.ZipFile(source) as archive:
        scene=json.loads(archive.read(next(n for n in archive.namelist() if n.endswith('scene.json'))))
    sdf=OUT/'horizontal_red4/fixture.sdf'
    lamps=lamps_from_sdf(sdf.read_bytes())
    mounts=[dict(name='front_145_horizontal',xyz=[.055,0,.145],up_deg=0.),
            dict(name='rear_200_horizontal',xyz=[-.075,0,.20],up_deg=0.),
            dict(name='rear_200_up10',xyz=[-.075,0,.20],up_deg=10.),
            dict(name='rear_300_horizontal',xyz=[-.075,0,.30],up_deg=0.),
            dict(name='rear_300_up10',xyz=[-.075,0,.30],up_deg=10.)]
    slots=sorted(scene['starting_grid']['slots'],key=lambda s:-s['s_m'])
    rows=[]
    # .28/.40는 40cm 계획의 기준점 불명확성을 보는 가정값이며 확정 설치 치수가 아니다.
    for z,mount,slot,height in itertools.product((.28,.40,1.08),mounts,slots,(.12,.25,.40)):
        variant=deepcopy(lamps)
        for lamp in variant:lamp['xyz'][2]=z
        origin,basis=camera(slot,mount)
        field=slots[:5] if slot in slots[:5] else slots[:4]+[slot]
        blockers=[s for s in field if s is not slot]
        entries=[]
        for lamp in variant:
            points=lamp_samples(lamp,origin,basis)
            uv=[pixel(p,origin,basis) for p in points]
            entries.append(dict(name=lamp['name'],center_uv=pixel(lamp['xyz'],origin,basis),
                in_image=all(v is not None and 0<=v[0]<1280 and 0<=v[1]<720 for v in uv),
                in_roi=all(v is not None and 0<=v[0]<1280 and 0<=v[1]<720*.58 for v in uv),
                peer_blocked=sum(any(segment_box(origin,p,b,height) for b in blockers) for p in points),
                self_blocked=sum(segment_box(origin,p,slot,.12) for p in points),samples=len(points)))
        rows.append(dict(lamp_z_m=z,mount=mount['name'],grid=slot['index'],peer_height_m=height,
                         lamps=entries,all_in_image=all(e['in_image'] for e in entries),
                         all_in_roi=all(e['in_roi'] for e in entries),
                         any_occlusion=any(e['peer_blocked'] or e['self_blocked'] for e in entries)))
    summary=[]
    for z,mount,height in itertools.product((.28,.40,1.08),mounts,(.12,.25,.40)):
        chosen=[r for r in rows if (r['lamp_z_m'],r['mount'],r['peer_height_m'])==(z,mount['name'],height)]
        assert len(chosen)==6
        summary.append(dict(lamp_z_m=z,mount=mount['name'],peer_height_m=height,
            grids_in_image=sum(r['all_in_image'] for r in chosen),grids_in_roi=sum(r['all_in_roi'] for r in chosen),
            grids_with_occlusion=[r['grid'] for r in chosen if r['any_occlusion']],
            grids_roi_and_clear=[r['grid'] for r in chosen if r['all_in_roi'] and not r['any_occlusion']]))
    sources=[Path(__file__),sdf,source,PREVIOUS/'occlusion_geometry.py',ROOT/'scripts/audit_start_signal_geometry.py']
    result=dict(scope='pinhole nine-ray static comparison, not rendering/detection/driving or hardware',
        assumptions=['lamp heights .28/.40/1.08m are sensitivity cases, not adopted installation',
                     'peer boxes 20x15cm and three assumed heights; ego solid .12m box',
                     '1280x720 HFOV75, no lens distortion or real CSI crop',
                     'nine points do not certify continuous visible area',
                     'no walls, signal housing, suspension pitch or actual mount occlusion'],
        mounts=mounts,summary=summary,rows=rows,
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    with (OUT/'height_sensitivity.json').open('x',encoding='utf-8') as file:
        json.dump(result,file,ensure_ascii=False,indent=2)
    print(json.dumps(summary))


if __name__=='__main__':main()
