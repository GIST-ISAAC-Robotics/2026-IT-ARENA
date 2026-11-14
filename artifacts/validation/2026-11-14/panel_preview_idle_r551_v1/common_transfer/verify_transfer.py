"""공통 이관 범위·원본 보존·실행 결과를 확인한다. 시험을 재실행하지 않는다."""
from pathlib import Path
import ast
import hashlib
import json
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
manifest = json.loads((HERE / 'manifest.json').read_text(encoding='utf-8'))
for relative, row in manifest.items():
    assert hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() == row['after_sha256'], relative
source_matches_stage = {}
for name in ['race_ops_page.py', 'race_ops_node.py']:
    source = (ROOT / 'src/arena_autonomy/arena_autonomy' / name).read_text(encoding='utf-8')
    staged = (HERE.parent / 'stage' / name).read_text(encoding='utf-8')
    assert source == staged
    source_matches_stage[name] = True

before = ast.parse((HERE / 'race_ops_node.py.before').read_text(encoding='utf-8'))
after = ast.parse((ROOT / 'src/arena_autonomy/arena_autonomy/race_ops_node.py').read_text(encoding='utf-8'))
def omit_get(tree):
    handler = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Handler')
    handler.body = [n for n in handler.body if not isinstance(n, ast.FunctionDef) or n.name != 'do_GET']
    return ast.dump(tree, include_attributes=False)
assert omit_get(before) == omit_get(after)

old_js = (HERE / 'operator_panel_page_checks.js.before').read_text(encoding='utf-8')
new_js = (ROOT / 'tests/operator_panel_page_checks.js').read_text(encoding='utf-8')
runner = '(async()=>{const name=process.argv[3]'
assert new_js[:new_js.index('checks.hidden_tab_keeps_ops_poll_without_preview=')] == old_js[:old_js.index(runner)]
assert new_js[new_js.index(runner):] == old_js[old_js.index(runner):]

suite = next(ET.parse(HERE / 'pytest.xml').getroot().iter('testsuite'))
assert int(suite.attrib['tests']) == 17
assert all(int(suite.attrib.get(k, 0)) == 0 for k in ['errors', 'failures', 'skipped'])
result = dict(source_matches_reviewed_stage=source_matches_stage,
    node_ast_outside_handler_get_unchanged=True,
    existing_js_checks_and_runner_unchanged=True,
    files_sha256_verified=len(manifest),
    pytest=suite.attrib,
    scope='Windows DOM + real common Handler AST / fake ROS; no server, WSL, ROS, Gazebo, Jetson or device execution')
(HERE / 'verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(result, ensure_ascii=False, indent=2))
