"""보존한 최신 공식 ZIP에서 램프/그리드를 대조하고 후보의 피치 여유를 계산한다."""
import copy
import hashlib
import itertools
import json
from pathlib import Path
import sys
import zipfile

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_start_signal_geometry import lamps_from_sdf, project
from audit_grid_camera_pitch import relative_targets, vertical_interval, intersect

OUT=Path(__file__).parent
archive_path=ROOT/'assets/track/official/v2026.10.24/it_arena_track_v2026.10.24.zip'
old=ROOT/'artifacts/validation/2026-10-13/race_start_geometry_v1/result'
with zipfile.ZipFile(archive_path) as archive:
    worlds=[n for n in archive.namelist() if n.endswith('world.sdf')]
    scenes=[n for n in archive.namelist() if n.endswith('scene.json')]
    assert len(worlds)==len(scenes)==1
    lamps=lamps_from_sdf(archive.read(worlds[0]))
    slots=json.loads(archive.read(scenes[0]))['starting_grid']['slots']
assert lamps==lamps_from_sdf((old/'world.sdf').read_bytes())
assert slots==json.loads((old/'scene.json').read_text())['starting_grid']['slots']
camera=yaml.safe_load((ROOT/'src/arena_description/config/b0183_c1.yaml').read_text())['camera']
rows=[]
for x,z in itertools.product((-.075,0.,.055),(.145,.20,.25,.30,.35)):
    cam=copy.deepcopy(camera);cam['xyz_m']=[x,0.,z]
    intervals=[]
    for base_z,slot in itertools.product((0.,.01),slots):
        _,corners=relative_targets(lamps,slot,cam,base_z)
        intervals.append(vertical_interval(corners,cam))
    for forward,base_z in itertools.product((.5,1.,1.5,2.),(0.,.01)):
        points=[[forward,y,h-z-base_z] for y,h in itertools.product((-.05,.05),(.35,.45))]
        intervals.append(vertical_interval(points,cam))
    interval=intersect(intervals)
    rows.append(dict(x_m=x,z_m=z,joint_vertical_interval=interval,
        pitch_tolerance_deg=(interval['max_up_deg']-interval['min_up_deg'])/2 if interval['nonempty'] else None))
report=dict(scope='latest_official_equal_geometry_and_generic_marker_pitch_margin_not_detection',
    official_zip_sha256=hashlib.sha256(archive_path.read_bytes()).hexdigest(),lamps_equal=True,grid_equal=True,
    marker_distance_scope='generic front-facing .10m board, heights .35-.45m, distances .5-2m from camera',rows=rows)
with (OUT/'latest_geometry_check.json').open('x') as stream:stream.write(json.dumps(report,indent=2)+'\n')
print(json.dumps({'lamps_equal':True,'grid_equal':True,'rows':[r for r in rows if r['joint_vertical_interval']['nonempty']]}))
