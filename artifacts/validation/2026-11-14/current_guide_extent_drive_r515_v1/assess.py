"""기존 별도 경기 목표를 현재 CAD 치수로 기록한다. 원 판정·임계값 변경 없음."""
from pathlib import Path
import sys

import yaml

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(A), str(ROOT / 'scripts')]
from prepare import read, sha, write_new
from surface_contact_eval import VehicleGeometry, evaluate_run
import goal_eval


def main():
    run = A / 'run1'
    report = read(run / 'report.json')
    original_report_hash = sha(run / 'report.json')
    profile = ROOT / 'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'
    assert sha(profile) == report['vehicle_config']['sha256']
    cfg = yaml.safe_load(profile.read_text(encoding='utf-8'))['vehicle']
    footprint, drive = cfg['footprint'], cfg['drivetrain']
    geometry = VehicleGeometry(body_length_m=footprint['length_m'], body_width_m=footprint['width_m'],
        wheelbase_m=drive['wheelbase_m'], track_width_m=drive['track_width_m'],
        wheel_radius_m=drive['wheel_radius_m'], wheel_width_m=drive['wheel_width_m'],
        max_wheel_steering_rad=drive['max_steering_angle_rad'])
    assert goal_eval.EGO_LENGTH_M == geometry.body_length_m
    assert goal_eval.EGO_WIDTH_M == geometry.body_width_m
    surface = evaluate_run(run, geometry=geometry)
    surface['geometry_status'] = dict(source=str(profile.relative_to(ROOT)), sha256=sha(profile),
        basis='current CAD dimensions; same old evaluator contact-patch10mm and Ackermann steering approximation; not actual tire contact or referee judgement')
    write_new(A / 'surface_run1.json', surface)
    world = ROOT / 'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
    goal_eval.evaluate(run, world)
    goal = read(A / 'run1_goal.json')
    assert sha(run / 'report.json') == original_report_hash
    write_new(A / 'assessment_summary.json', dict(
        case='right_side_start', scope='ego passes right-front actor and continues; not rear overtaking',
        raw_original_passed=report['passed'], raw_original_gates=report['gates'],
        existing_supplement_passed=goal['passed'], existing_supplement_gates=goal['gates'],
        scene_established=goal['established'], scene_conditions=goal['scene_conditions'], metrics=goal['metrics'],
        geometry=surface['geometry'], geometry_status=surface['geometry_status'],
        original_report_sha256=original_report_hash, original_report_unchanged=True,
        threshold_changes=False, actual_run_count=1, repeat_on_failure=False,
        interpretation='legacy15cm/following and existing80mm/two-wheel/return goals remain separate; failed gates are retained, no retroactive pass'))


if __name__ == '__main__':
    main()
