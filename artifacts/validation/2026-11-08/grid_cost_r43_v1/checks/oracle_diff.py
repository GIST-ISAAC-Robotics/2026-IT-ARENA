"""before.zip 원본 함수와 tests/r43_reference_oracle.py 사본의 AST 비교. 차이는 의도한 이름·계측 줄뿐이어야 한다."""
import ast, difflib, json, sys, zipfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[5]
Z = ROOT/'artifacts/validation/2026-11-08/grid_cost_r43_v1/before.zip'
P = 'src/arena_autonomy/arena_autonomy/'
oracle = ast.parse((ROOT/'tests/r43_reference_oracle.py').read_text(encoding='utf-8'))
with zipfile.ZipFile(Z) as z:
    before = {n: ast.parse(z.read(P+n+'.py').decode('utf-8')) for n in ('local_scene', 'lateral_intent', 'pass_release_trial')}


def find(tree, name, cls=None):
    nodes = tree.body
    if cls:
        nodes = next(n for n in nodes if isinstance(n, ast.ClassDef) and n.name == cls).body
    node = next(n for n in nodes if isinstance(n, ast.FunctionDef) and n.name == name)
    body = [s for s in node.body if not (isinstance(s, ast.Expr) and isinstance(s.value, ast.Constant))]
    return [ast.unparse(s) for s in body]


pairs = {'blind_spec': (find(before['local_scene'], 'blind_spec'), find(oracle, 'blind_spec')),
         'rear_masked': (find(before['local_scene'], 'rear_masked'), find(oracle, 'rear_masked')),
         'scene_from_scan': (find(before['local_scene'], 'scene_from_scan'), find(oracle, 'scene_from_scan')),
         'ObservationGrid.__init__': (find(before['lateral_intent'], '__init__', 'ObservationGrid'), find(oracle, 'reference_grid')),
         'threat_view': (find(before['pass_release_trial'], 'threat_view'), find(oracle, 'threat_view'))}
out = {}
for name, (a, b) in pairs.items():
    a = '\n'.join(a).splitlines(); b = '\n'.join(b).splitlines()
    out[name] = [l for l in difflib.unified_diff(a, b, 'before', 'oracle', n=0, lineterm='') if not l.startswith('@@')]
print(json.dumps(out, ensure_ascii=False, indent=1))
