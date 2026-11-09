"""r58 변경 근거와 C4 실행 인자를 보존한다. 과거 평가기는 수정하지 않는다."""
import hashlib,json,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sha=lambda b:hashlib.sha256(b).hexdigest()
target='src/arena_autonomy/arena_autonomy/bypass_wall_fit.py'
parent='config/tests/nearby_long_window_r57_sha256.json'
contract='config/tests/tracked_wall_occluder_r58_sha256.json'
snapshot=A/'before_r58.zip'
with zipfile.ZipFile(snapshot) as z:before=z.read(target)
assert not (R/contract).exists()
data=dict(schema_version=1,scope='r58: apply the selected narrow-return bridge to current tracked wall reference; all obstacle returns, unknown rays and default remain unchanged',
    parent_contract=parent,parent_contract_sha256=sha((R/parent).read_bytes()),
    before_snapshot=snapshot.relative_to(R).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
    approved_changes={target:dict(before=sha(before),after=sha((R/target).read_bytes()),
        reason='Join only observed wall returns on both sides of a small continuous foreground cluster; never removes occupancy')},
    approved_evaluation_scripts={},new_dependencies={})
(R/contract).write_text(json.dumps(data,indent=2)+'\n',encoding='utf8',newline='\n')
p=R/'tests/test_source_provenance_r0_plus_b3.py';raw=p.read_bytes()
needle=b"M3_SUCCESSORS += ('config/tests/nearby_long_window_r57_sha256.json',)"
assert raw.count(needle)==1
newline=b'\r\n' if b'\r\n' in raw else b'\n'
p.write_bytes(raw.replace(needle,needle+newline+b"M3_SUCCESSORS += ('config/tests/tracked_wall_occluder_r58_sha256.json',)"))
original=R/'artifacts/validation/2026-11-08/rear_face_motion_connected_v1/goal_eval.py'
out=A/'stationary_goal_eval.py';assert not out.exists()
s=original.read_text(encoding='utf8')
assert s.count("run.parent/'surface_run1.json'")==1
s=s.replace("run.parent/'surface_run1.json'","run.parent/f'surface_{run.name}.json'")
s=s.replace("f'{run.name}_goal.json'","f'{run.name}_goal_surface_corrected.json'")
out.write_text(s,encoding='utf8',newline='\n')
(A/'stationary_goal_eval_source.json').write_text(json.dumps(dict(
    parent=str(original.relative_to(R)),parent_sha256=sha(original.read_bytes()),
    corrected_sha256=sha(out.read_bytes()),
    changes=['surface path selected by run name','separate corrected output path'],
    limits_unchanged=True),indent=2)+'\n')
for ext in ('py','sh'):
    out=A/f'c4_run.{ext}';assert not out.exists()
    s=(A/f'c3_run.{ext}').read_text(encoding='utf8').replace('run3','run4').replace('c3_run.py','c4_run.py')
    if ext=='py':
        s=s.replace("'signal_posts:=stl_61cm_provisional']","'signal_posts:=stl_61cm_provisional', 'g1_narrow_occluder_bridge:=true']")
        s=s.replace("added_launch_argument='signal_posts:=stl_61cm_provisional'", "added_launch_arguments=['signal_posts:=stl_61cm_provisional','g1_narrow_occluder_bridge:=true']")
    else:
        s=s.replace('src/arena_autonomy/arena_autonomy/nearby_face_motion.py scripts/',
                    'src/arena_autonomy/arena_autonomy/nearby_face_motion.py src/arena_autonomy/arena_autonomy/bypass_wall_fit.py scripts/')
        start=s.index('  bash "$ENV" /usr/bin/python3 -B -c ')
        end=s.index('\nfi',start)
        s=s[:start]+'  bash "$ENV" /usr/bin/python3 -B "$A/stationary_goal_eval.py" "$D/run4" src/arena_gazebo/worlds/it_arena_official_v2026_11_06 > "$D/goal_run4.log" 2>&1; goal=$?'+s[end:]
        s=s.replace('sha256sum -c',
            'bash "$ENV" /usr/bin/python3 -B "$A/evaluate_traffic_posts.py" "$D/run4" > "$D/posts_run4.log" 2>&1; posts=$?\nsha256sum -c')
        s+= '\nexit "$harness"\n'
    out.write_text(s,encoding='utf8',newline='\n')
print('r58 source chain, corrected evaluator and C4 prepared; no simulation executed')
