"""준비 파일·원 조건·동일 공통 SHA 대조. 실행 등록/동결·장치 접근 없음."""
import ast
import hashlib
import json
from pathlib import Path
import re

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'traffic_current_geometry_r496_v1'
CASE = ROOT / 'artifacts/validation/2026-11-08/right_side_newcourse_v1'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    cfg, original = read(A / 'config.json'), read(CASE / 'new_track_course.json')
    assert cfg['cases'] == original['cases']
    assert cfg['acceptance']['right_side_start'] == original['acceptance']['slow_lead']
    assert cfg['actor']['command_hz'] == original['actor']['command_hz'] == 50
    assert cfg['actor']['height_m'] == .22 and original['actor']['height_m'] == .12
    actor = dict(cfg['actor']); actor.pop('visual_outline')
    actor['height_m'] = original['actor']['height_m']
    actor['visible_section'] = original['actor']['visible_section']
    assert actor == original['actor']
    current = read(OLD / 'config.json')
    assert cfg['acceptance']['stop'] == current['acceptance']['stop']
    launch = dict(cfg['launch']); launch['entry_g1'] = current['launch']['entry_g1']
    assert launch == current['launch']
    goal = (A / 'goal_eval.py').read_text(encoding='utf-8')
    goal = goal.replace('EGO_LENGTH_M = .207\nEGO_WIDTH_M = .15\nACTOR_LENGTH_M = .20\n', '')
    goal = goal.replace("rectangle(row['x'], row['y'], row['yaw'], EGO_LENGTH_M, EGO_WIDTH_M)", "rectangle(row['x'], row['y'], row['yaw'], .20, .15)")
    goal = goal.replace('body = a_s-ego_s-(EGO_LENGTH_M+ACTOR_LENGTH_M)/2', 'body = a_s-ego_s-.20')
    goal = goal.replace('ego_s-EGO_LENGTH_M/2 > a_s+ACTOR_LENGTH_M/2', 'ego_s-.10 > a_s+.10')
    goal = goal.replace('(np.abs(ego_s-a_s)<=(EGO_LENGTH_M+ACTOR_LENGTH_M)/2)', '(np.abs(ego_s-a_s)<=.20)')
    assert goal == (CASE / 'goal_eval.py').read_text(encoding='utf-8')
    prepared = read(OLD / 'prepared.json')
    common = {p:h for p,h in prepared['frozen_inputs'].items() if p.startswith(('src/','scripts/','tests/','config/'))}
    changed = [p for p,h in common.items() if sha(ROOT/p) != h]
    assert not changed, changed
    native = read(OLD / 'native_v1/receipt.json')
    for p,h in native['production_source_hashes'].items():
        assert sha(ROOT/p) == h
    library = ROOT/'src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'
    assert sha(library) == native['native_sha256']
    for p in A.rglob('*.py'):
        ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'), *A.rglob('*.bash')]:
        assert b'\r' not in p.read_bytes(), p
    missing = [target for target in re.findall(r'\]\(([^)]+)\)', (A/'PLAN.md').read_text(encoding='utf-8')) if not (A/target).exists()]
    assert not missing, missing
    assert not any((A/n).exists() for n in ('prepared.json','sources.zip','registered.json','run1'))
    result = dict(passed=True, common_files_compared=len(common), changed_common=[],
        original_case_exact=True, original_actor_except_visible_height_and_explicit_box_exact=True,
        raw_acceptance_alias_exact=True, existing_goal_only_current_body_dimensions_changed=True,
        new_thresholds=False, reused_native_tests=166, native_library_sha256=sha(library),
        geometry_passed=read(A/'geometry_right_side.json')['passed'], rejected_rear_overlap_samples=read(A/'geometry.json')['facility_collision_sample_count'],
        script_syntax_passed=True, plan_links_present=True, wsl_used_for_this_check=False,
        common_frozen=False, actual_simulation_started=False,
        artifacts={p.name:sha(p) for p in [A/'PLAN.md', A/'config.json', A/'arguments.json', A/'goal_eval.py', A/'geometry_right_side.json', A/'recipe_provenance.json']})
    with (A/'preparation_receipt.json').open('x', encoding='utf-8') as f:
        json.dump(result,f,ensure_ascii=False,indent=2); f.write('\n')
    print(json.dumps({k:result[k] for k in ('passed','common_files_compared','changed_common','geometry_passed','common_frozen','actual_simulation_started')}))


if __name__ == '__main__':
    main()
