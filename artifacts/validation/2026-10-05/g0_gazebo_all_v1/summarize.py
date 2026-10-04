"""Gazebo G0 report.json의 사례별 판정·핵심 수치만 표로 출력한다. 원시 jsonl은 읽지 않는다."""
import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
print('passed', report.get('passed'), 'failures', [f['message'] for f in report.get('failures', [])])
for case in report['cases']:
    checks = case.get('checks') or {}
    failed = [name for name, ok in checks.items() if not ok]
    actor = case.get('actor') or {}
    summary = case.get('observation_summary') or {}
    startup = case.get('startup') or {}
    print(json.dumps({
        'case': case['case'], 'passed': case['rendered_observation_passed'],
        'failures': [f"{f['source']}:{f['type']}:{f['message']}" for f in case.get('failures', [])],
        'failed_checks': failed, 'geometry_possible': case['geometry']['cross_section_possible'],
        'active_scans': case.get('active_scan_count'), 'raw_frames': case.get('raw_frame_count'),
        'raw_hz': case.get('raw_rate_sim_hz'), 'scan_hz': case.get('sequential_rate_sim_hz'),
        'max_gap_s': case.get('raw_max_source_gap_s'), 'discarded': case.get('discarded_source_gaps'),
        'max_capture_error_s': case.get('max_capture_time_error_s'),
        'visible_min_max': [summary.get('visible_target_rays_min'), summary.get('visible_target_rays_max')],
        'actor_cmds': actor.get('command_count'), 'actor_err_m': actor.get('max_observed_error_to_latest_command_m'),
        'actor_cmd_step_s': actor.get('max_commanded_step_s'),
        'actor_obs_step_s': actor.get('max_observed_position_update_interval_s'),
        'phases': [(p['segment']['speed_mps'], p['sensor_range_delta_m'], p['observed_pose_x_delta_m'], p['passed'])
                   for p in (case.get('dynamic_phases') or {}).get('phases', [])],
        'startup_missing': startup.get('missing_streams'), 'startup_ready_wall_s': startup.get('ready_wall_s'),
        'renderer': case['environment'].get('actual_renderer_verified'),
        'shutdown_clean': case['shutdown'].get('clean')}, ensure_ascii=False, default=str))
