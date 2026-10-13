"""solver 제한 대조의 입력 관문과 수치 결과를 구분한다. 기본 채택 판정은 하지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

from analyze_evaluator_gc import source_snapshot_valid, sha
from analyze_scheduled_apply import analyze_run as scheduled_run, compare_envelopes
from analyze_initial_startup import normalized_xml
from analyze_scheduled_phase import compare_metrics
from analyze_scheduled_surface import (REFERENCE_REPORT, REFERENCE_SHA256, frozen_sources_match,
    braking_contacts, rows)
from analyze_contact_model import analyze_run as physics_audit, read_contacts, model_params, exact_sample
from probe_solver_compare import SOLVERS
from analyze_apply_trace import divergence

ROOT = Path(__file__).resolve().parents[1]


def actual_solver(log, solver):
    """단순 요청 문자열/ECM 값이 아닌 엔진의 실제 primary getter 출력만 허용한다."""
    plain = re.sub(r'\x1b\[[0-9;]*m', '', log)
    found = re.findall(r'Using \[(DantzigBoxedLcpSolver|PgsBoxedLcpSolver)\] solver\.', plain)
    errors = [line for line in plain.splitlines() if any(x in line.lower() for x in
        ('failed to cast constraint solver', 'defaulting to [', 'solverfeature', 'failed to solve', 'lcp solver failed'))]
    return {'primary_types': sorted(set(found)), 'getter_log_count': len(found),
            'solver_error_lines': errors, 'passed': bool(found) and set(found) == {SOLVERS[solver]} and not errors,
            'scope': 'gz-physics actual primary getter. Per-step fallback not instrumented.'}


def world_without_solver(path, expected=None):
    root = ET.parse(path).getroot()
    physics = root.findall('world/physics')
    if len(physics) != 1 or physics[0].findtext('max_step_size') != '0.001':
        raise ValueError('world/physics step changed')
    dart = physics[0].find('dart')
    solvers = [] if dart is None else dart.findall('solver')
    if expected is None:
        if solvers:
            raise ValueError('reference already specifies solver')
    else:
        if len(solvers) != 1 or len(solvers[0]) != 1 or solvers[0].findtext('solver_type') != expected:
            raise ValueError('unexpected solver configuration')
        if solvers[0].attrib or solvers[0][0].attrib:
            raise ValueError('unexpected solver attributes')
        dart.remove(solvers[0])
        if len(dart) == 0 and not dart.attrib and not (dart.text or '').strip():
            physics[0].remove(dart)
    return hashlib.sha256(normalized_xml(root)).hexdigest()


def force_audit(run, zero):
    """방향 마찰 감사는 full RPY; 평면 운동량은 기존 엄격한 전제 충족 때만 표시한다."""
    data = rows(run/'braking_diagnostics.jsonl')
    selected = [r for r in data if zero <= r['sim_time_s'] <= zero+.22]
    if not selected:
        raise ValueError('no post-zero diagnostics')
    contacts, duplicates = read_contacts(run/'wheel_contacts.jsonl')
    params = model_params(run/'runtime_vehicle.sdf')
    timeline, missing = [], []
    for row in selected:
        ns = round(row['sim_time_s']*1e9)
        absent = [w for w in ('rear_left_wheel', 'rear_right_wheel') if ns not in contacts[w]]
        if absent:
            missing.append({'sim_time_s': row['sim_time_s'], 'missing_wheels': absent,
                'body_mps': row['truth_longitudinal_mps'], 'mean_wheel_mps': row['wheel_surface_speed_mps'],
                'z_m': row['truth_z_m'], 'roll_rad': row['truth_roll_rad'], 'pitch_rad': row['truth_pitch_rad']})
        else:
            timeline.append(exact_sample(row, contacts, params))
    tilt = max(abs(r[f'truth_{a}_rad']) for r in selected for a in ('roll', 'pitch'))
    impulse, unavailable = None, None
    if tilt > 1e-5:
        unavailable = 'roll/pitch exceeds unchanged near-planar precondition'
    elif missing:
        unavailable = 'missing exact same-step rear contact; not zero-filled'
    else:
        try:
            impulse = physics_audit(run, zero)
        except ValueError as exc:
            unavailable = str(exc)
    return {'point_bound_ratio_max': max((v['point_bound_ratio_max'] for r in timeline for v in r['rear'].values()), default=None),
            'max_abs_roll_pitch_rad': tilt, 'planar_precondition': tilt <= 1e-5,
            'planar_audit': impulse, 'planar_audit_unavailable': unavailable,
            'exact_contact_coverage_complete': not missing, 'missing_exact_contacts': missing,
            'selected_diagnostic_samples': len(selected), 'timeline': timeline, 'duplicates_removed': duplicates,
            'scope': 'rear contacts at recorded ~20 ms drivetrain diagnostics, first 0.22 s after applied zero; not all 1 ms samples'}


def input_integrity(summary):
    """정지 결과는 별도로 보존하고 입력/출처/종료 관문은 하나도 완화하지 않는다."""
    detailed = summary['evidence']['checks']
    traced = summary['trace']['validity']
    checks = {'evidence_except_stop': all(v is True for k, v in detailed.items() if k != 'stopped'),
        'trace_except_outcome': all(v is True for k, v in traced.items() if k not in ('report_passed', 'stopped', 'passed')),
        'other_checks': all(v is True for k, v in summary['checks'].items() if k not in ('evidence', 'posthoc_trace_run')),
        'schedule_checks': all(not v for v in summary['evidence']['failed_schedule_checks'].values())}
    return {'checks': checks, 'passed': all(checks.values()),
            'physical_run_passed': summary['passed'], 'stopped': detailed['stopped']}


def analyze(base, dantzig_run='d1_dantzig'):
    if dantzig_run not in ('d1_dantzig', 'd2_dantzig'):
        raise ValueError('unregistered run')
    registration = json.loads((base/'registered_manifest.json').read_text())
    contract_path = ROOT/'config/tests/scheduled_command_r7_sha256.json'
    if sha(contract_path) != registration['contract_sha256'] or sha(REFERENCE_REPORT) != REFERENCE_SHA256:
        raise ValueError('prior contract/report changed')
    contract = json.loads(contract_path.read_text())
    reference = json.loads(REFERENCE_REPORT.read_text())['source_sha256']
    wrapper = 'scripts/probe_solver_compare.py'
    reference[wrapper] = registration['sources'][wrapper]
    checks = {'registered_source_zip': source_snapshot_valid(base/'registered_source.zip', registration['sources']),
              'registered_sources_still_same': all(sha(ROOT/p) == digest for p, digest in registration['sources'].items())}
    bundles = [scheduled_run(base/name, contract, registration['plugin_sha256']) for name in (dantzig_run, 'p1_pgs')]
    a, b = [r[0] for r in bundles]
    start = a['start_ns']//1000
    envelopes = compare_envelopes(bundles[0][2], bundles[1][2], start, start+round(a['duration_s']*1e6)+200000)
    checks.update(both_valid=a['passed'] and b['passed'], both_scheduled=a['scheduled'] and b['scheduled'],
                  same_start=a['start_ns'] == b['start_ns'], same_duration=a['duration_s'] == b['duration_s'],
                  same_initial_step=a['trace']['trace']['spawn'] == b['trace']['trace']['spawn'],
                  step_1ms=a['trace']['step_s'] == b['trace']['step_s'] == .001,
                  same_normal_envelopes=envelopes['same_all_normal_applied_envelopes'])
    for key in ('source_sha256', 'runtime_vehicle_sha256', 'meshes_sha256', 'profile', 'spawn_pose'):
        checks['same_'+key] = a['evidence'][key] == b['evidence'][key]
    old = ROOT/'artifacts/validation/2026-10-12/scheduled_surface_v1/f1_flat'
    expected_world = world_without_solver(old/'runtime_world.sdf')
    results = {}
    for (summary, data, events), solver in zip(bundles, SOLVERS):
        run = base/summary['name']
        report = json.loads((run/'report.json').read_text())
        selected = actual_solver((run/'simulation.log').read_text(errors='replace'), solver)
        checks[run.name+'_actual_solver'] = selected['passed']
        checks[run.name+'_frozen_sources'] = frozen_sources_match(report['source_sha256'], registration['sources'], reference)
        checks[run.name+'_flat_passive'] = report['surface'] == 'flat' and report['zero_speed_mode'] == 'passive_damping'
        checks[run.name+'_only_solver_changed'] = world_without_solver(run/'runtime_world.sdf', solver) == expected_world
        checks[run.name+'_same_prior_vehicle'] = sha(run/'runtime_vehicle.sdf') == sha(old/'runtime_vehicle.sdf')
        contact = braking_contacts(data, events, rows(run/'wheel_contacts.jsonl'), summary['start_ns'], summary['duration_s'],
                                   report['command_profile']['brake_window_s'], .0325)
        results[run.name] = {'evidence': summary, 'actual_solver': selected, 'contacts': contact,
            'force_audit': force_audit(run, contact['zero_sim_s']), 'input_integrity': input_integrity(summary),
            'last_body_diagnostics': data[-1],
            'state_samples': [min(data, key=lambda r: abs(r['sim_time_s']-t)) for t in (1.8, 2.01, 3., 3.8, 4.311, 6.51, 7.51)]}
    metrics, within, secondary = compare_metrics(a['trace'], b['trace'])
    sources = {Path(m.__file__).resolve().relative_to(ROOT).as_posix(): sha(Path(m.__file__).resolve())
               for m in list(sys.modules.values()) if getattr(m, '__file__', None)
               and Path(m.__file__).resolve().is_relative_to(ROOT/'scripts')}
    sources[Path(__file__).relative_to(ROOT).as_posix()] = sha(Path(__file__))
    input_checks = {k: v for k, v in checks.items() if k != 'both_valid'}
    input_checks['both_input_integrity'] = all(r['input_integrity']['passed'] for r in results.values())
    return {'revision': 'v3_input_comparison_separate_from_stop_outcome',
            'checks': checks, 'comparison_valid': all(checks.values()), 'envelopes': envelopes, 'runs': results,
            'input_comparison_checks': input_checks, 'input_comparison_valid': all(input_checks.values()),
            'divergence': divergence(bundles[0][1], bundles[1][1], a['start_ns']),
            'metrics': metrics, 'metrics_within_existing_tolerance': within, 'secondary_metrics': secondary,
            'analysis_source_sha256': sources,
            'limits': ['one paired run; neither physical convergence nor real tire/MD20A validation',
                       'primary solver verified; internal per-step fallback not instrumented',
                       'passive_damping and flat support remain evaluation-only; no automatic default adoption']}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--dantzig-run', choices=('d1_dantzig', 'd2_dantzig'), default='d1_dantzig')
    args = p.parse_args()
    result = analyze(args.base.resolve(), args.dantzig_run)
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False, allow_nan=False)
    print(json.dumps({'checks': result['checks'], 'metrics': result['metrics'], 'force':
        {k: {name: v['force_audit'][name] for name in ('point_bound_ratio_max', 'max_abs_roll_pitch_rad', 'planar_precondition')}
         for k, v in result['runs'].items()}}, indent=2))
    raise SystemExit(0 if result['comparison_valid'] else 1)
