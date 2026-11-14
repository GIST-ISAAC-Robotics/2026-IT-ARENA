"""종료 후 원 평가·실제 간격·지속 주행·현 기하 노면을 병렬 기록한다. 원 판정 수정 없음."""
import collections
import json
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from prepare import read, write_new, sha
from audit_local_pursuit_road import road_polygon
from g1_course_eval import wheel_center_surface


def main():
    run = A/'run1'
    report = read(run/'report.json')
    if not report.get('case_window') or report['case_window'].get('green_sim_s') is None:
        write_new(A/'execution_summary.json', dict(summary_available=False,
            unavailable_reason='no_started_case_window', original_report_passed=report.get('passed'),
            original_gates=report.get('gates'), original_verdict_changed=False,
            run_valid=report.get('run_valid'), harness_valid=report.get('harness_valid'),
            shutdown_clean=report.get('shutdown_clean'), stop_checks=report.get('stop_checks'),
            original_report_sha256=sha(run/'report.json'), actual_drive_trial_completed=False))
        return
    first, last = (report['case_window'][k] for k in ('green_sim_s','end_sim_s'))
    rows = [r for r in read(run/'trace.json') if first <= r['t'] <= last]
    poses = [json.loads(line) for line in (run/'ego_poses.jsonl').open(encoding='utf-8')]
    poses = [r for r in poses if first <= r[0]]  # r475와 동일하게 STOP 종료까지 포함
    geometry = report['vehicle_config']['ego_geometry']
    wheels = wheel_center_surface(road_polygon(run/'runtime_world.sdf'), poses,
        wheelbase=geometry['wheelbase_m'], track=geometry['track_width_m'])
    actuation = read(run/'actuation_trace.json')
    evaluation = report['evaluation']
    write_new(A/'execution_summary.json', dict(
        scope='one 12sim-second right-side visible-actor scene; not rear overtaking; not full lap, low-height detection or physical driving proof',
        original_report_passed=report['passed'], original_gates=report['gates'],
        original_verdict_changed=False, run_valid=report['run_valid'], harness_valid=report['harness_valid'],
        case_window=report['case_window'], progress=evaluation['progress'], case_metrics=evaluation['case_metrics'],
        actual_gap=evaluation['gap'], wall_collision_samples=report['wall_collision_samples'],
        wheel_centers_current_geometry=wheels, peak_ground_speed_mps=max(r['ground_speed_mps'] for r in rows),
        command_expired_samples=sum(r.get('reason')=='command_expired' for r in actuation),
        mcu_reasons=dict(collections.Counter(r.get('reason') for r in actuation)),
        stop_checks=report['stop_checks'], stop=report['stop'], shutdown_clean=report['shutdown_clean'],
        shutdown_audit=report['shutdown_audit'], remaining_test_pids=report['remaining_test_pids'],
        source_unchanged=report['source_unchanged'], integrity=read(A/'integrity.json'),
        desired_final_selector_mode=None, expected_contact_samples=0,
        progress_and_stall_existing_rules=read(A/'config.json')['acceptance']['right_side_start'],
        interpretation='report original gap/final-gap/behind/road gates even if passing; do not require a particular mode or rewrite raw passed'))


if __name__ == '__main__':
    main()
