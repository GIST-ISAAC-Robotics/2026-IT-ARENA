def finalize_verdict(report, mode):
    """실행 유효성(하네스·무결성·종료)과 행동 판정을 나눠 기록한다."""
    evaluation = report.get('evaluation', {})
    harness = evaluation.get('harness', {})
    controller = evaluation.get('controller_mode', {})
    if mode not in ('g1', 'r0', 'r0plus', 'm3', 'g1g2'):
        raise ValueError('unsupported controller mode')
    rows = controller.get('behavior_rows', 0)
    mode_matches = ((rows > 0 and controller.get('g1_rows') == rows) if mode == 'g1' else
                    (rows > 0 and controller.get('g1g2_rows') == rows) if mode == 'g1g2' else
                    (rows > 0 and controller.get('r0plus_rows') == rows) if mode == 'r0plus' else
                    (rows > 0 and controller.get('m3_rows') == rows) if mode == 'm3' else
                    controller.get('g1_rows', 0) == 0 and controller.get('r0plus_rows', 0) == 0
                    and controller.get('m3_rows', 0) == 0 and controller.get('g1g2_rows', 0) == 0)
    integrity = {
        'no_runtime_error': 'error' not in report and 'error' not in evaluation,
        'evaluation_readers_valid': report.get('evaluation_readers', {}).get('valid') is True,
        'no_traffic_actor_in_world': evaluation.get('no_actor', {}).get('passed') is True,
        'sensor_only_controller_inputs': report.get('sensor_only_inputs') is True,
        'sensor_only_safety_inputs': report.get('safety_sensor_only_inputs') is True,
        'actuation_sensor_only_inputs': report.get('actuation', {}).get('sensor_only_inputs') is True,
        'no_actor_pose_publishers': report.get('actor_pose_publishers') == {'eval_g1b_actor': [], 'eval_second_lead': []},
        'drive_publisher_controller_only': report.get('drive_publishers') == ['local_pursuit'],
        'safe_drive_publisher_safety_only': report.get('safe_drive_publishers') == ['lidar_safety'],
        'sequential_acquisition': report.get('sequential_acquisition_verified') is True,
        'lidar_interface': report.get('lidar_nominal_interface_passed') is True,
        'source_unchanged': report.get('source_unchanged') is True,
        'signal_sequence': (report['team_signal_evidence'].get('passed') is True if 'team_signal_evidence' in report
                            else report['official_signal_evidence'].get('passed') is True if 'official_signal_evidence' in report
                            else {'red', 'yellow', 'green'}.issubset(report.get('signal_states', []))),
        'no_false_start': report.get('false_start') is False,
        'controller_mode_matches': mode_matches,
    }
    stop = report.get('stop', {})
    actuation = report.get('actuation', {})
    stop_checks = {
        'model_stop_stable': stop.get('stable') is True,
        'dense_pose_stop_evidence': (stop.get('pose_evidence') or {}).get('passed') is True,
        'explicit_mcu_stop': report.get('actuation_stop_passed') is True,
        'no_mcu_latch_before_stop': ((actuation.get('stop_latch_timing') or {}).get('valid') is True
                                     and actuation.get('latched_before_stop_samples', 1) == 0 and report.get('abort') is None),
        'no_deadline_diagnostic': report.get('actuation_diagnostics', {}).get('control_deadline_samples', 1) == 0,
        'no_wall_collision': report.get('wall_collision_samples') == 0 and stop.get('wall_collision_samples') == 0,
    }
    behavior = dict(evaluation.get('gates', {}))
    report['harness_valid'] = harness.get('valid') is True
    report['integrity'] = integrity
    report['stop_checks'] = stop_checks
    report['gates'] = behavior
    report['run_valid'] = report['harness_valid'] and all(integrity.values()) and report.get('shutdown_clean') is True
    report['behavior_gates_passed'] = bool(behavior) and all(behavior.values()) and all(stop_checks.values())
    if mode in ('g1', 'r0plus', 'm3', 'g1g2'):
        report['passed'] = report['run_valid'] and report['behavior_gates_passed']
    else:
        # R0는 비교 기준이다. 실행 유효성만 통과 조건이며 G1 판정선 충족 여부는 따로 남긴다.
        report['passed'] = report['run_valid']
        report['r0_meets_g1_gates'] = report['behavior_gates_passed']
    return report
