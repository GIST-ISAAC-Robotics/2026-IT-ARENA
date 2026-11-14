"""함수 한정 이관과 기존 default 연산 AST를 기록한다. 검사는 재실행하지 않는다."""
import ast
import copy
import difflib
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SOURCE = ROOT / 'src/arena_autonomy/arena_autonomy/local_path.py'
TEST = ROOT / 'tests/test_swept_radial_motion.py'
before = (HERE / 'local_path.py.before').read_bytes()
after = SOURCE.read_bytes()
old_tree = ast.parse(before)
new_tree = ast.parse(after)
old_function = next(n for n in old_tree.body if isinstance(n, ast.FunctionDef) and n.name == 'swept_limit')
new_function = next(n for n in new_tree.body if isinstance(n, ast.FunctionDef) and n.name == 'swept_limit')
old_tree.body.remove(old_function)
new_tree.body.remove(new_function)
assert ast.dump(old_tree, include_attributes=False) == ast.dump(new_tree, include_attributes=False)

default_function = copy.deepcopy(new_function)
default_function.args.kwonlyargs = []
default_function.args.kw_defaults = []
default_function.body[0] = copy.deepcopy(old_function.body[0])
class DefaultPath(ast.NodeTransformer):
    def visit_If(self, node):
        test = ast.unparse(node.test)
        if test == 'radial_margin is not None':
            return None
        if test == 'radial_margin is None':
            return node.body
        return self.generic_visit(node)
default_function = DefaultPath().visit(default_function)
assert ast.dump(default_function, include_attributes=False) == ast.dump(old_function, include_attributes=False)

(HERE / 'local_path.py.after').write_bytes(after)
(HERE / TEST.name).write_bytes(TEST.read_bytes())
patch = ''.join(difflib.unified_diff(before.decode('utf-8').splitlines(True),
    after.decode('utf-8').splitlines(True), fromfile='a/'+SOURCE.relative_to(ROOT).as_posix(),
    tofile='b/'+SOURCE.relative_to(ROOT).as_posix()))
patch += ''.join(difflib.unified_diff([], TEST.read_text(encoding='utf-8').splitlines(True),
    fromfile='/dev/null', tofile='b/'+TEST.relative_to(ROOT).as_posix()))
(HERE / 'r552_swept_radial.patch').write_text(patch, encoding='utf-8')
sha = lambda raw: hashlib.sha256(raw).hexdigest()
suite = next(ET.parse(HERE / 'pytest.xml').getroot().iter('testsuite'))
assert int(suite.attrib['tests']) == 35
assert all(int(suite.attrib.get(key, 0)) == 0 for key in ['errors', 'failures', 'skipped'])
result = dict(source=str(SOURCE.relative_to(ROOT)).replace('\\', '/'),
    before_sha256=sha(before), after_sha256=sha(after), test_sha256=sha(TEST.read_bytes()),
    outside_swept_limit_ast_unchanged=True,
    default_branch_ast_matches_original=True,
    default_note='None 경로의 추가 판정은 선택값 분기뿐이며 기존 수치 연산 AST는 동일',
    pytest=suite.attrib,
    scope='Windows NumPy geometry only; no WSL, ROS, Gazebo, Jetson or physical device execution')
(HERE / 'manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(result, ensure_ascii=False, indent=2))
