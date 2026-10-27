"""첫 완주 시험 대기 중 고정 배치의 기하 후보만 비교. 실행 설정 변경 없음."""
import copy
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_grid_camera_pitch import relative_targets, vertical_interval, intersect
from audit_start_signal_geometry import lamps_from_sdf, project, rotation

OUT=Path(__file__).parent
SOURCE=ROOT/'artifacts/validation/2026-10-13/race_start_geometry_v1/result'
cam_path=ROOT/'src/arena_description/config/b0183_c1.yaml'
camera=yaml.safe_load(cam_path.read_text())['camera']
lamps=lamps_from_sdf((SOURCE/'world.sdf').read_bytes())
slots=json.loads((SOURCE/'scene.json').read_text())['starting_grid']['slots']
focal=camera['width_px']/(2*math.tan(math.radians(camera['horizontal_fov_deg'])/2))
rows=[]
for x,z in itertools.product((-.075,0.,.055),(.145,.20,.25,.30,.35)):
    cam=copy.deepcopy(camera)
    cam['xyz_m']=[x,0.,z]
    bounds=[]
    for base_z,slot in itertools.product((0.,.01),slots):
        centers,corners=relative_targets(lamps,slot,cam,base_z)
        bounds.append(vertical_interval(corners,cam))
    interval=intersect(bounds)
    row=dict(x_m=x,z_m=z,all_grid_lamp_vertical_interval=interval,candidates=[])
    if interval['nonempty']:
        for pitch in (interval['min_up_deg']+.5,(interval['min_up_deg']+interval['max_up_deg'])/2):
            cam['rpy_rad']=[0.,-math.radians(pitch),0.]
            images=[project(lamps,slot,cam,base_z) for base_z,slot in itertools.product((0.,.01),slots)]
            markers=[]
            for forward in (.5,1.,1.5,2.):
                pts=np.array([[forward,y,h-z] for y,h in itertools.product((-.05,.05),(.35,.45))])
                rotated=(rotation(*cam['rpy_rad']).T@pts.T).T
                uv=np.column_stack((camera['width_px']/2-focal*rotated[:,1]/rotated[:,0],
                                    camera['height_px']/2-focal*rotated[:,2]/rotated[:,0]))
                visible=bool(np.all(rotated[:,0]>0) and np.all(uv>=0) and
                             np.all(uv[:,0]<camera['width_px']) and np.all(uv[:,1]<camera['height_px']))
                markers.append(dict(distance_m=forward,full_image=visible,uv=uv.tolist()))
            row['candidates'].append(dict(up_deg=pitch,lamps_full=sum(v['all_inside_image'] for v in images),
                lamps_detector_roi=sum(v['all_inside_detector_region'] for v in images),
                expected_color_order=sum(v['red_to_green_left_to_right'] for v in images),generic_markers=markers))
    rows.append(row)
sources=[Path(__file__),cam_path,SOURCE/'scene.json',SOURCE/'world.sdf',
         ROOT/'scripts/audit_grid_camera_pitch.py',ROOT/'scripts/audit_start_signal_geometry.py']
report=dict(scope='pinhole_mount_comparison_no_render_or_occlusion_no_mount_change',
    source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    official_source='b27f947 10/13 preserved geometry; latest official geometry not yet rechecked',rows=rows,
    limitations=['generic frontal marker is not actual route visibility','no self or other vehicle occlusion',
                 'actual CSI crop/optics and light construction unknown','camera center height is not assembled exterior height'])
with (OUT/'report.json').open('x') as stream:stream.write(json.dumps(report,indent=2)+'\n')
for r in rows:
    viable=[c for c in r['candidates'] if c['lamps_full']==12 and all(m['full_image'] for m in c['generic_markers'])]
    print(json.dumps(dict(x_m=r['x_m'],z_m=r['z_m'],joint_candidates=[dict(up_deg=c['up_deg'],roi=c['lamps_detector_roi'],order=c['expected_color_order']) for c in viable])))
