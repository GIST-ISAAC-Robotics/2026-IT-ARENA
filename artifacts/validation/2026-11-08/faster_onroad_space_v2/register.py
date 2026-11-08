"""빠른 상대의 측방 위치만 달리한 시험. 정적 도로 검사 뒤 1회 실행할 소스를 고정한다."""
from copy import deepcopy
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import zipfile

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
import validate_g1_traffic as ev

R32=HERE.parent/'moving_pass_wall_mask_v1'
case='straight_moving_lead'
old=json.loads((R32/case/'registered.json').read_bytes())
args=list(old['args']); i=args.index('--config')+1
assert args[i]==(R32/'new_track_course.json').relative_to(ROOT).as_posix()
args[i]=(HERE/'new_track_course.json').relative_to(ROOT).as_posix()
original=ev.load_config(R32/'new_track_course.json')
current=ev.load_config(HERE/'new_track_course.json')
normalized=deepcopy(current)
for name in ['scope','registered_at']:normalized[name]=original[name]
normalized['cases'][0]['expected']=original['cases'][0]['expected']
assert normalized['cases'][0]['actor_lateral_offset_m']==-.20
normalized['cases'][0]['actor_lateral_offset_m']=-.12
assert normalized==original
assert args[args.index('--moving-pass-profile')+1]=='faster_v1'
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
assert sha(HERE/'assess.py')==sha(R32/'assess.py')
exact=json.loads((HERE.parent/'faster_follow_return_probe_v1/summary.json').read_bytes())
assert exact['baseline_command_mismatches']==0
world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
line,scene=ev.load_centerline(world); road=ev.road_polygon(world/'world.sdf'); walls=ev.obstacle_polygons(world/'world.sdf')
geometry={}
for cfg,key in [(original,'r32'),(current,'right20')]:
    c=cfg['cases'][0];s0=ev.actor_start_s(cfg,scene,0,c)
    geometry[key]=ev.check_route(cfg,line,c,s0,road,walls)
assert geometry['right20']['minimum_sampled_road_edge_distance_m']>.02
assert abs(geometry['right20']['minimum_sampled_road_edge_distance_m']
           -geometry['r32']['minimum_sampled_road_edge_distance_m']+.08)<1e-4
if '--check' in sys.argv:
    print(json.dumps(geometry));raise SystemExit(0)
d=HERE/case;d.mkdir(exist_ok=False)
(HERE/'geometry.json').write_text(json.dumps(geometry,indent=2)+'\n',encoding='utf8')
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
helpers=['register.py','new_track_course.json','assess.py','run.py','run.sh','check_runtime.py','PLAN.md','geometry.json']
files=sorted({f for f in files if (ROOT/f).is_file()}|{(HERE/f).relative_to(ROOT).as_posix() for f in helpers})
with zipfile.ZipFile(d/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
    for f in files:archive.write(ROOT/f,f)
record=dict(registered_utc=datetime.now(timezone.utc).isoformat(),scope='one current-code 0.70m/s lead at right20cm onroad; no new control',
            args=args,sources={f:sha(ROOT/f) for f in files},snapshot_sha256=sha(d/'source_snapshot.zip'),
            r32_evaluator_sha256=sha(R32/'assess.py'),r32_args_identical_except_config=True)
(d/'registered.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf8')
print('registered',len(files),geometry['right20'])
