"""동결 stage 원본을 보존하고 임시 길이 API만 제거한 최종 후보 생성."""
from pathlib import Path
import difflib, hashlib, json
A=Path(__file__).resolve().parent; REL=Path('src/arena_autonomy/arena_autonomy')
manifest={}
for name in ('local_path.py','current_wall_reacquire.py','local_follow.py'):
    source=A/'stage'/REL/name
    text=source.read_text(encoding='utf8')
    if name=='local_path.py':
        text=text.replace('narrow_occluder_bridge=False, minimum_ahead_m=.28, bearings_rad=None):',
                          'narrow_occluder_bridge=False, bearings_rad=None):')
        text=text.replace('    minimum_ahead_m은 벽 앵커 앞 최소 관측 호길이다. 기본0.28m는 유지하며,\n'
                          '    저속 재획득 호출만 차체 길이 기준을 전달한다. 경로의 전방 길이 검사는 별도다.\n','')
        text=text.replace('if last-anchor_s >= minimum_ahead_m:', 'if last-anchor_s >= .28:')
    elif name=='current_wall_reacquire.py':
        text=text.replace('    try:\n        minimum_ahead = .28\n','    try:\n')
        text=text.replace('minimum_ahead_m=minimum_ahead, bearings_rad=bearings)', 'bearings_rad=bearings)')
        text=text.replace('minimum_boundary_ahead_m=minimum_ahead,','minimum_boundary_ahead_m=.28,')
    out=A/'stage2'/REL/name
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(text,encoding='utf8',newline='\n')
    original=(A/'original'/REL/name).read_text(encoding='utf8')
    patch=''.join(difflib.unified_diff(original.splitlines(True),text.splitlines(True),
        fromfile='a/'+(REL/name).as_posix(),tofile='b/'+(REL/name).as_posix()))
    (A/(name+'.stage2.diff')).write_text(patch,encoding='utf8',newline='\n')
    manifest[name]=dict(original_sha256=hashlib.sha256((A/'original'/REL/name).read_bytes()).hexdigest(),
        stage_sha256=hashlib.sha256(out.read_bytes()).hexdigest())
(A/'manifest_stage2.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
print(json.dumps(manifest,indent=2))
