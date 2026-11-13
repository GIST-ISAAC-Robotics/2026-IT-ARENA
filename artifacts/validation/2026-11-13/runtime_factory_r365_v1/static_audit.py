"""실행 없이 정책 중복/차이와 초안 구문·원본 해시만 확인한다."""
import ast
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent.parent/'2026-11-12'
SRC = ROOT/'src/arena_autonomy/arena_autonomy'
prior = OLD/'retained_route_execution_r266_v1/candidate_recheck'
rows = []
for path in sorted(prior.glob('*.py')):
    target = SRC/'current_path_recheck'/path.name
    equal = ast.dump(ast.parse(path.read_text(encoding='utf-8'))) == ast.dump(ast.parse(target.read_text(encoding='utf-8')))
    rows.append(dict(file=path.name, ast_equal=equal))
files = [OLD/'terminal_boundary_r278_v1/candidate.py',
         OLD/'current_region_policy_r281_v1/candidate.py',
         OLD/'retained_route_execution_r266_v1/candidate_policy.py',
         A.parent/'current_solo_low_r343_v1/candidate.py',
         A.parent/'current_solo_gap_r346_v1/candidate.py',
         A.parent/'solo_crawl_retry_r352_v1/candidate.py',
         SRC/'g1_g2_node.py', SRC/'local_follow.py', SRC/'guard_observation_policy.py']
files += sorted(prior.glob('*.py'))
files += sorted(A.glob('*.py'))
for path in files:
    ast.parse(path.read_text(encoding='utf-8'))
report = dict(status='static_draft_only_no_execution', candidate_recheck_comparison=rows,
              common_files_modified=False, wsl_jetson_ros_gazebo_executed=False,
              sha256={str(path.relative_to(ROOT)).replace('\\', '/'):hashlib.sha256(path.read_bytes()).hexdigest() for path in files})
(A/'static_audit.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
print(json.dumps(dict(ast_equal=sum(r['ast_equal'] for r in rows), compared=len(rows), hashed_files=len(files), status=report['status'])))
