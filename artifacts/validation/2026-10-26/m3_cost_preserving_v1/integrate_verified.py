"""전체 저장 비교가 통과한 뒤만 실행하는 일회성 이관. 기존 바이트를 별도 ZIP에 보존한다."""
import ast
import hashlib
import json
from pathlib import Path
import re
import textwrap
import zipfile

ROOT = Path.cwd()
BASE = Path(__file__).resolve().parent
summary = json.loads((BASE/'run6_full_returned/data/summary.json').read_text())
assert summary['passed'] and summary['source_unchanged']
assert sum(x['rounds'][0]['timing']['count'] for x in summary['results'].values()) == 6165
for name, value in summary['source_sha256'].items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == value, name

paths = ['src/arena_autonomy/arena_autonomy/lateral_intent.py',
         'src/arena_autonomy/arena_autonomy/m3_execution_trial.py']
before = {p: (ROOT/p).read_bytes() for p in paths}
snapshot = BASE/'before_integration_sources.zip'
if snapshot.exists():
    with zipfile.ZipFile(snapshot) as archive:
        assert set(archive.namelist()) == set(before)
        assert all(archive.read(path) == data for path, data in before.items())
else:
    with zipfile.ZipFile(snapshot, 'x', zipfile.ZIP_DEFLATED) as archive:
        for path, data in before.items():
            archive.writestr(path, data)


def function(source, name, cls=None):
    tree = ast.parse(source)
    nodes = tree.body if cls is None else next(n for n in tree.body if getattr(n, 'name', None) == cls).body
    node = next(n for n in nodes if getattr(n, 'name', None) == name)
    lines = source.splitlines(keepends=True)
    return ''.join(lines[node.lineno-1:node.end_lineno])


old_project_source = before[paths[0]].decode()
old_project = function(old_project_source, 'project', 'PathFrame')
new_project = function((BASE/'projection_candidate.py').read_text(encoding='utf-8'), 'project_xy')
new_project = re.sub(r'\bframe\b', 'self', new_project).replace('def project_xy(', 'def project(')
new_project = new_project.replace('    q =', '''    """모든 선분의 정확 투영. 128점씩 x/y 배열을 계산해 임시 배열 비용만 줄인다.

    양 끝 연장·sqrt·첫 argmin 동률·곱셈/덧셈 순서를 보존한다.
    경로/점 생략이나 이전 시각의 결과 캐시는 사용하지 않는다.
    """
    q =''', 1)
(ROOT/paths[0]).write_text(old_project_source.replace(old_project, textwrap.indent(new_project, '    ')), encoding='utf-8', newline='\n')

old_returned_source = before[paths[1]].decode()
old_returned = function(old_returned_source, '_both_sides_returned')
new_returned = function((BASE/'returned_candidate.py').read_text(encoding='utf-8'), 'both_returned_window')
new_returned = new_returned.replace('def both_returned_window(', 'def _both_sides_returned(')
new_returned = new_returned.replace('    cycle_count =', '''    """양옆 광선의 반환 조건은 그대로 두고 가능한 방향만 정확 검사한다.

    모든 광선 원점을 감싸는 원 바깥 질의에서 방향 변화의 상계는
    asin(radius/distance)다. 여기에 기존 최대 광선 간격 .025와 수치
    여유를 더한다. 원 안/근처는 전광선 검사하며 원래 순서로 동률을
    결정한다. unknown/마스크, source 인접성, 취득 시각은 바꾸지 않는다.
    """
    cycle_count =''', 1)
(ROOT/paths[1]).write_text(old_returned_source.replace(old_returned, new_returned), encoding='utf-8', newline='\n')

# 회귀용 독립 전광선/전선분 기준식을 보존한다. 실행 모듈의 빠른 함수를 기준식으로 재호출하지 않는다.
projection_test = (BASE/'test_projection_candidate.py').read_text(encoding='utf-8').replace(
    'from projection_candidate import project_xy',
    textwrap.dedent(old_project).replace('def project(', 'def reference_project(')+'\nproject_xy = PathFrame.project')
projection_test = projection_test.replace('frame.project(q)', 'reference_project(frame, q)')
(ROOT/'tests/test_path_projection_exact.py').write_text(
    '"""전선분 기준식과 투영 결과 비트 일치. 경로 후보/동률/끝 연장 보존."""\n'+projection_test,
    encoding='utf-8', newline='\n')
returned_test = (BASE/'test_returned_candidate.py').read_text(encoding='utf-8').replace(
    'from arena_autonomy.m3_execution_trial import _both_sides_returned',
    'from arena_autonomy.scan_topology import cyclic_count, adjacent_indices\n'+old_returned)
returned_test = returned_test.replace('from returned_candidate import both_returned_window',
    'from arena_autonomy.m3_execution_trial import _both_sides_returned as both_returned_window')
(ROOT/'tests/test_returned_ray_window.py').write_text(
    '"""전광선 기준식 대조. 원점/동률/가림/무응답/순환 증거를 보존한다."""\n'+returned_test,
    encoding='utf-8', newline='\n')

parent = 'config/tests/m2_merge_r0_sha256.json'
digest = lambda p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
contract = dict(schema_version=1,
    scope='M3 출력 보존 계산 개선. 전선분 투영·전광선 반환 조건 동일. 기본 활성화·주행 안전 승인 아님',
    parent_contract=parent, parent_contract_sha256=digest(parent),
    before_snapshot=str(snapshot.relative_to(ROOT)).replace('\\', '/'),
    before_snapshot_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
    approved_changes={p: dict(before=hashlib.sha256(before[p]).hexdigest(), after=digest(p),
        reason='6,165 저장 판단과 순수 기하 반례에서 출력 동일한 계산만 이관. 정책·관측·간격·기본값 불변') for p in paths},
    approved_evaluation_scripts={}, new_dependencies={})
(ROOT/'config/tests/m3_exact_cost_r0_sha256.json').write_text(
    json.dumps(contract, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')
for name in ('test_source_provenance_r0_plus_b3.py', 'test_corner_follow.py',
             'test_g1_dense_stop.py', 'test_physics_step_option.py'):
    path = ROOT/'tests'/name
    s = path.read_text(encoding='utf-8')
    if name == 'test_source_provenance_r0_plus_b3.py':
        s = s.replace("'config/tests/m2_merge_r0_sha256.json')", "'config/tests/m2_merge_r0_sha256.json', 'config/tests/m3_exact_cost_r0_sha256.json')")
        s = s.replace("[10, 11, 12]", "[10, 11, 12, 13]")
    else:
        s = s.replace("'m2_merge_r0_sha256.json')", "'m2_merge_r0_sha256.json', 'm3_exact_cost_r0_sha256.json')")
    path.write_text(s, encoding='utf-8', newline='\n')
print(json.dumps(contract, ensure_ascii=False, indent=2))
