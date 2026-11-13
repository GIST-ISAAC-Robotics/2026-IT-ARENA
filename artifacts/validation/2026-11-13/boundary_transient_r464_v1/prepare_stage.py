"""r457 동결본의 local_follow 한 파일만 r464 stage에 파생한다."""
from pathlib import Path
import ast
import difflib
import hashlib
import json

HERE=Path(__file__).resolve().parent
REL='src/arena_autonomy/arena_autonomy/local_follow.py'
old=(HERE/'original'/REL).read_text(encoding='utf-8')
needle="""                        if match == 'shared_arc':
                            detail['boundary_history_agreement'] = 'current_corner_arc'
                    else:
                        eligible = eligible and path_agrees(candidate, carried, cfg.boundary_agreement_m)
"""
replacement="""                        if match == 'shared_arc':
                            detail['boundary_history_agreement'] = 'current_corner_arc'
                    elif (eligible and cfg.junction_continuity and cfg.expiry_route_reference_trial
                            and cfg.piecewise_wall_trial and self.path_wall == side
                            and info.get('wall_model') == 'piecewise_corner'):
                        # 동일 벽의 충분한 현재 관측과 유효 이력만 비교한다. 코너 출구의
                        # x 창 길이 차이로 갱신을 놓치지 않게 기존 호길이 비교를 재사용한다.
                        # 초기/만료 경로 획득, 벽 전환, TTL과 이후 점유 검사는 그대로다.
                        eligible = path_agrees(candidate, carried, cfg.boundary_agreement_m)
                        if not eligible and path_agrees_arc(candidate, carried, cfg.boundary_agreement_m):
                            eligible = True
                            detail['boundary_history_agreement'] = 'current_same_wall_corner_arc'
                    else:
                        eligible = eligible and path_agrees(candidate, carried, cfg.boundary_agreement_m)
"""
assert old.count(needle)==1
new=old.replace(needle,replacement)
target=HERE/'stage'/REL
target.parent.mkdir(parents=True,exist_ok=True)
target.write_text(new,encoding='utf-8',newline='\n')
ast.parse(new,filename=REL)
(HERE/'changes.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
    fromfile='a/'+REL,tofile='b/'+REL)),encoding='utf-8')
changes={'source_archive_sha256':json.loads((HERE/'original_manifest.json').read_text())['archive_sha256'],
    'source_changes':[{'path':REL,
        'original_sha256':hashlib.sha256((HERE/'original'/REL).read_bytes()).hexdigest(),
        'stage_sha256':hashlib.sha256(target.read_bytes()).hexdigest()}],
    'common_modified':False,'current_recheck_modified':False,'static_ast_passed':True,
    'tests_executed':False,'native_replay_executed':False,'drive_executed':False}
(HERE/'changes.json').write_text(json.dumps(changes,indent=2),encoding='utf-8')
print(json.dumps(changes,indent=2))
