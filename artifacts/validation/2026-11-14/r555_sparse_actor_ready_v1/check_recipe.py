"""JSON·해시·AST만 검사한다. import/ROS·WSL·동결·actor 생성·주행은 하지 않는다."""
import ast
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'sparse_actor_measured_steering_r546_v1'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    old = read(OLD / 'config.json')
    cfg = read(A / 'config.json')
    assert cfg == read(A / 'expected_config.json')
    expected = dict(old)
    for key in ('scope', 'registered_at'):
        expected[key] = cfg[key]
    expected['launch'] = dict(old['launch'])
    expected['launch']['entry_g1'] = old['launch']['entry_g1'].replace(OLD.name, A.name)
    assert cfg == expected
    args = [x.replace(OLD.name, A.name) for x in read(OLD / 'arguments.json')]
    assert args == read(A / 'arguments.json')
    recipe = read(A / 'recipe.json')
    case = cfg['cases'][0]
    assert case['name'] == 'right_side_start' and case['duration_s'] == 12.
    assert (case['actor_start_center_gap_m'], case['actor_lateral_offset_m']) == (.5, -.23)
    assert case['schedule'] == [dict(from_s=0., to_s=12., speed_mps=.5)]
    assert recipe['marker']['assistance'] == 'OFF' and recipe['marker']['value'] == ''
    assert (recipe['marker']['id'], recipe['marker']['board_m'], recipe['marker']['black_code_m']) == (1, .05, .0348)
    assert recipe['marker']['white_border_each_side_m'] == .0076
    assert recipe['marker']['same_marked_actor_for_off_on']
    assert cfg['actor']['visual_outline'] == 'sparse_upper_posts'
    assert sha(ROOT / recipe['vehicle_config']) == recipe['vehicle_config_sha256']
    for name in ('actor.sdf', 'actor_fixture.py', 'marker_source.png', 'field_world.sdf', 'facilities_only.sdf',
                 'traffic_runner.py', 'collect_evidence.py', 'collect_result.py', 'assess.py', 'analyze_controller_guard.py', 'expected_header.json'):
        assert sha(A / name) == sha(OLD / name), name
    source = read(A / 'source_selection.json')
    assert source['source_baseline'] == recipe['intended_source_baseline']
    assert source['source_baseline'].endswith('measured_steering_radial_lap_r553_v1')
    assert not any(source[k] for k in ('freeze_authorized', 'drive_authorized', 'wsl_exclusive_owner'))
    assert source['final_production_source_selection_pending']
    assert source['included_integrations'] == [] and source['status_detail'] == 'full'
    assert source['additional_sources'] == ['scripts/experimental_facilities.py']
    for name, digest in source['additional_source_hashes'].items():
        assert sha(ROOT / name) == digest
    freeze = (A / 'prepare.py').read_text(encoding='utf-8')
    assert "source_base=ROOT/source['source_baseline']" in freeze
    assert "common={p:h for p,h in baseline['frozen_inputs'].items()" in freeze
    assert "source_base/'prepared.json',source_base/'sources.zip'" in freeze
    assert "*A.glob('*.png')" in freeze
    marker = (A / 'marker_course.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('r537_marker_profile',default_value='')" in marker
    assert "SetParameter(name='vehicle_marker_observer_enabled',value=False)" in marker
    for name in ('prepared.json', 'sources.zip', 'registered.json', 'run1', 'preflight', 'root_execution_review.json'):
        assert not (A / name).exists(), name
    provenance = read(A / 'recipe_provenance.json')
    for name, digest in provenance['predecessor_files_sha256'].items():
        assert sha(OLD / name) == digest, name
    for row in provenance['files']:
        assert sha(A / row['path']) == row['after_sha256'], row['path']
    for path in A.rglob('*.py'):
        ast.parse(path.read_bytes())
    for path in [*A.rglob('*.sh'), *A.rglob('*.bash')]:
        assert b'\r' not in path.read_bytes(), path
    result = dict(passed=True, preparation_only=True, frozen=False, WSL_used=False, ROS_used=False,
        Gazebo_used=False, Jetson_used=False, dynamic_drive_runs=0, render_detection_runs=0,
        actor_world_and_evaluator_byte_identical=True, original_acceptance_and_stop_unchanged=True,
        r546_original_files_preserved=len(provenance['predecessor_files_sha256']),
        final_r553_source_selection_pending=True, marker_assistance='OFF', marker_on_run_prepared=False,
        actor_sdf_sha256=sha(A / 'actor.sdf'), world_sha256=sha(A / 'field_world.sdf'),
        recipe_sha256=sha(A / 'recipe.json'), source_selection_sha256=sha(A / 'source_selection.json'))
    (A / 'preparation_receipt.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result))

if __name__ == '__main__':
    main()
