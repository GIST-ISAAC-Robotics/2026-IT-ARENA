"""실행 없이 추출 원문/산출물 해시와 그대로 유지한 메서드 AST를 기록한다."""
import ast
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent.parent/'2026-11-12'
SRC = ROOT/'src/arena_autonomy/arena_autonomy'


def method(path, cls, name):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    body = tree.body if cls is None else next(n for n in tree.body if getattr(n, 'name', None) == cls).body
    return next(n for n in body if getattr(n, 'name', None) == name)


checks = []
for name in ('application_reason', 'prepared', 'commit_published', 'discard_ready'):
    prior = method(OLD/'retained_route_execution_r266_v1/route_delivery.py', 'RouteProcess', name)
    current = method(SRC/'runtime_dual_state.py', 'RuntimeDualProcess', name)
    checks.append(dict(method=name, equal=ast.dump(prior) == ast.dump(current)))
for name in ('acknowledge_publication', 'offer_recheck'):
    prior = method(A.parent/'current_ros_bridge_r336_v1/ros_delivery.py', 'Delivery', name)
    current = method(SRC/'runtime_dual_delivery.py', 'RuntimeDualDelivery', name)
    checks.append(dict(method=name, equal=ast.dump(prior) == ast.dump(current)))
for name, prior_path in [('same_positive_intent', A.parent/'current_deceleration_r334_v1/deceleration.py'),
                         ('may_finish_current', OLD/'current_priority_r286_v1/priority.py')]:
    prior = method(prior_path, None, name)
    current = method(SRC/'runtime_dual_state.py', None, name)
    checks.append(dict(method=name, equal=ast.dump(prior) == ast.dump(current)))

originals = [OLD/'retained_route_execution_r266_v1/overlap.py',
             OLD/'retained_route_execution_r266_v1/route_delivery.py',
             OLD/'current_priority_r286_v1/priority.py',
             OLD/'latest_scan_submission_r273_v1/submission.py',
             OLD/'current_region_runtime_r282_v1/delivery.py',
             A.parent/'current_phase_r323_v1/phase.py',
             A.parent/'current_deadline_r325_v1/deadline.py',
             A.parent/'current_deceleration_r334_v1/deceleration.py',
             A.parent/'current_ros_bridge_r336_v1/ros_delivery.py']
current = sorted(SRC.glob('runtime_dual_*.py'))+sorted((ROOT/'tests').glob('test_runtime_dual_*.py'))
imports = {}
test_cases = {}
for path in current:
    tree = ast.parse(path.read_text(encoding='utf-8'))
    imports[str(path.relative_to(ROOT)).replace('\\', '/')] = sorted({n.module for n in ast.walk(tree)
        if isinstance(n, ast.ImportFrom) and n.module})
    if path in list(SRC.glob('runtime_dual_*.py')):
        assert all(not x.startswith(('artifacts', 'candidate_', 'route_', 'overlap', 'deceleration'))
                   for x in imports[str(path.relative_to(ROOT)).replace('\\', '/')])
    else:
        count = 0
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name.startswith('test_'):
                cases = 1
                for decorator in node.decorator_list:
                    if (isinstance(decorator, ast.Call) and isinstance(decorator.func, ast.Attribute)
                            and decorator.func.attr == 'parametrize'):
                        cases *= len(decorator.args[1].elts)
                count += cases
        test_cases[path.name] = count
report = dict(passed=all(row['equal'] for row in checks), ast_same_methods=checks, imports=imports,
    static_test_cases=test_cases,
    sha256={str(path.relative_to(ROOT)).replace('\\', '/'):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in originals+current}, scope='static extraction audit, not runtime validation')
(A/'source_audit.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
print(json.dumps(dict(passed=report['passed'], same_methods=len(checks), hashed_files=len(report['sha256']),
                      static_test_cases=test_cases)))
if not report['passed']:
    raise SystemExit(1)
