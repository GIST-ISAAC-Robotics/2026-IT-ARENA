"""G1 부분 관측 후보의 교통 시험 범위와 실행 증거. 제어 입력을 만들지 않는다."""

POLICIES = ('strict', 'bounded_local_v2', 'bounded_local_v3', 'bounded_local_v4')


def pass_release_arguments(enabled, parallel):
    if enabled and not parallel:
        raise ValueError('pass release requires explicit parallel hold trial')
    return ['g1_pass_release_trial:=true'] if enabled else []


def guard_curve_arguments(enabled, pass_release):
    if enabled and not pass_release:
        raise ValueError('guard curve trial requires explicit pass release trial')
    return ['g1_guard_curve_trial:=true'] if enabled else []


def parallel_trial_arguments(enabled, policy, mode, recorded, boundary, occupancy):
    if not enabled:
        return []
    if (policy != 'bounded_local_v4' or mode != 'g1' or not recorded or
            boundary != 'corner_distance_candidate' or occupancy != 'per_ray_candidate'):
        raise ValueError('parallel hold requires recorded G1 v4 with corner-distance and per-ray candidates')
    return ['g1_parallel_hold_trial:=true']


def partial_trial_arguments(policy, mode, recorded, launch):
    """기본 실행은 그대로 두고, 명시 후보만 검증한 저속 구성으로 제한한다."""
    if policy not in POLICIES:
        raise ValueError('unsupported partial observation traffic policy')
    if policy == 'strict':
        return []
    required = {
        'autonomy_mode': 'local_pursuit', 'lidar_acquisition': 'sequential',
        'lidar_rate_hz': 10., 'lidar_compensation': 'both',
        'speed_profile': 'g1_compare_1mps', 'actuation_mode': 'virtual_mcu',
    }
    if mode != 'g1' or not recorded or any(launch.get(k) != v for k, v in required.items()):
        raise ValueError('partial observation requires recorded G1, sequential 10 Hz, both compensation, '
                         'g1_compare_1mps and virtual_mcu')
    return [f'partial_observation_policy:={policy}']


def partial_trace_fields(policy, autonomy, safety, *, parallel=False, pass_release=False):
    """기본 trace 형식은 유지하고 후보에서만 양쪽 원시 진단을 보존한다."""
    if policy == 'strict':
        return {}
    if policy not in POLICIES:
        raise ValueError('unsupported partial observation traffic policy')
    result = {'partial_observation_trial': {
        'controller_policy': autonomy.get('partial_observation_policy'),
        'controller': autonomy.get('partial_observation'),
        'safety': safety.get('partial_observation'),
    }}
    if parallel:
        result['parallel_hold_trial'] = autonomy.get('parallel_hold_trial')
        result['parallel_hold_selected'] = autonomy.get('parallel_hold_selected', False)
    if pass_release:
        result['pass_release_trial'] = {k:autonomy.get(k) for k in (
            'pass_release_trial', 'selector_mode', 'selector_mode_before', 'selector_choice', 'selector_reason',
            'selector_sides', 'selector_evidence', 'selector_region', 'selector_excursion_m',
            'selector_unknown_filled', 'selector_hidden_rear_risk_unresolved', 'selector_guard_candidate')}
    return result


def pass_release_evidence(trace, guard_curve=False):
    rows=[r['pass_release_trial'] for r in trace if isinstance(r.get('pass_release_trial'),dict)]
    active=[r for r in rows if r.get('pass_release_trial') is True]
    choices=('baseline','parallel','hold_fault')+(('guard_baseline',) if guard_curve else ())
    bad=[r for r in active if r.get('selector_unknown_filled') is not False
         or (r.get('selector_mode') or r.get('selector_mode_before')) not in ('SOLO','GUARD','RELEASED')
         or r.get('selector_choice') not in choices
         or (r.get('selector_choice')=='guard_baseline' and (r.get('selector_mode')!='GUARD'
             or not isinstance(r.get('selector_guard_candidate'),dict)
             or r['selector_guard_candidate'].get('accepted') is not True))]
    result={'diagnostic_rows':len(active), 'invalid_rows':len(bad),
            'modes':sorted({r.get('selector_mode') or r.get('selector_mode_before') or 'missing' for r in active}),
            'baseline_rows':sum(r.get('selector_choice')=='baseline' for r in active),
            'parallel_rows':sum(r.get('selector_choice')=='parallel' for r in active),
            'passed':bool(active) and not bad,
            'scope':'explicit selector diagnostics, not closed-loop success or unseen obstacle absence'}
    if guard_curve:
        result['guard_baseline_rows']=sum(r.get('selector_choice')=='guard_baseline' for r in active)
    return result


def parallel_trial_evidence(trace):
    rows = [r for r in trace if isinstance(r.get('parallel_hold_trial'), dict)]
    selected = [r for r in rows if r.get('parallel_hold_selected')]
    invalid = [r for r in rows if r['parallel_hold_trial'].get('unknown_filled') is not False]
    return {'diagnostic_rows': len(rows), 'selected_rows': len(selected), 'invalid_rows': len(invalid),
            'passed': bool(rows) and bool(selected) and not invalid,
            'scope': 'explicit straight trial activation, not side clearance or closed-loop success'}


def partial_trial_evidence(policy, trace):
    """평가 trace의 양쪽 판단을 확인한다. 설정 인자·주행 성과와 구별한다."""
    if policy not in POLICIES or policy == 'strict':
        raise ValueError('partial traffic evidence requires an explicit candidate')
    choices, safety_count, missing_count, filled_count = [], 0, 0, 0
    for row in trace:
        data = row.get('partial_observation_trial') or {}
        choice = data.get('controller_policy')
        if choice is not None:
            choices.append(choice)
        for name in ('controller', 'safety'):
            result = data.get(name)
            if isinstance(result, dict):
                if name == 'safety':
                    safety_count += 1
                missing_count += int(result.get('unknown_samples', 0) > 0)
                filled_count += int(result.get('unknown_filled') is not False)
    return {
        'requested_policy': policy, 'controller_policies': sorted(set(choices)),
        'controller_rows': len(choices), 'safety_rows': safety_count,
        'partial_diagnostic_rows': missing_count, 'invalid_unknown_fill_rows': filled_count,
        'passed': bool(choices) and all(p == policy for p in choices) and safety_count > 0 and filled_count == 0,
        'scope': 'recorded_policy_and_independent_safety_diagnostics_not_closed_loop_success',
    }
