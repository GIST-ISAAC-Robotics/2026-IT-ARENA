"""평가기 GC 대조: 원시/소스 무결성, 지연·메모리·정지 판정을 분리한다(물리 비교 아님)."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile

from analyze_apply_trace_v2 import registered_sources
from analyze_evaluator_timing import analyze, read_jsonl
from audit_schedule_gate_v2 import evidence_checks, WINDOW_CHECKS, ALL_TICK_CHECKS, checks_dict

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as f:
        for part in iter(lambda: f.read(1024*1024), b''):
            digest.update(part)
    return digest.hexdigest()


def source_snapshot_valid(path, sources):
    if not isinstance(sources, dict) or not sources:
        return False
    with zipfile.ZipFile(path) as z:
        return len(z.namelist()) == len(sources) and set(z.namelist()) == set(sources) and all(
            hashlib.sha256(z.read(name)).hexdigest() == digest for name, digest in sources.items())


def run_result(run, contract, plugin_sha):
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    status = json.loads((run.parent/f'status_{run.name}.json').read_text(encoding='utf-8'))
    entry = status['cases'][0]
    sources = report['source_sha256']
    raw_names = ('braking_diagnostics', 'wheel_contacts', 'requested_commands', 'safe_commands',
                 'actuation_status', 'output_status', 'gc_events', 'slow_callbacks', 'plugin_command_events')
    integrity = {f'raw_{name}': isinstance(report.get(name), dict) and (run/f'{name}.jsonl').is_file()
                 and sha(run/f'{name}.jsonl') == report[name].get('sha256') for name in raw_names}
    integrity.update(raw_timing=sha(run/'evaluator_timing.i64') == report['evaluator_timing']['sha256'],
                     registered_sources=registered_sources(sources, contract),
                     source_zip=source_snapshot_valid(run/'source_snapshot.zip', sources),
                     plugin_binary=report.get('plugin_binary_sha256') == plugin_sha,
                     residual_empty=entry.get('residual_after') == [],
                     no_injection=report.get('evaluation_fault_injection_requested', 'missing') is None)
    common = evidence_checks(report)
    failed = {}
    for name, keys in [('request_audit', WINDOW_CHECKS), ('request_audit_all_ticks', ALL_TICK_CHECKS)]:
        checks = checks_dict(report.get(name), keys)
        failed[name] = [k for k, v in checks.items() if v is not True] if checks is not None else ['missing_checks']
    natural_late_only = (set(failed['request_audit']) <= {'lateness_within_limit'}
                         and set(failed['request_audit_all_ticks']) <= {'all_ticks_lateness_within_limit'}
                         and bool(failed['request_audit'] or failed['request_audit_all_ticks']))
    complete = all(common.values()) and all(integrity.values())
    valid = (complete and entry.get('rc') == 0 and report.get('passed') is True and report.get('input_valid') is True
             and not any(failed.values()))
    late_invalid = (complete and natural_late_only and entry.get('rc') == 1
                    and report.get('passed') is False and report.get('input_valid') is False)
    policy = report['evaluation_gc']
    is_defer = policy['mode'] == 'defer'
    timing = analyze(run)
    lateness = sorted(r['observed_ns']-r['scheduled_ns'] for r in read_jsonl(run/'requested_commands.jsonl'))
    gc_delta = [b-a for a, b in zip(policy['collections_before'], policy['collections_after'])]
    policy_ok = (policy.get('restored') is True and policy.get('enabled_before') is True
                 and policy.get('enabled_after_restore') is True
                 and policy.get('peak_rss_after_kib', float('inf')) < 2*1024*1024
                 and (not is_defer or (policy.get('enabled_during') is False
                                      and policy.get('enabled_before_restore') is False and gc_delta == [0, 0, 0])))
    return {'name': run.name, 'mode': policy['mode'], 'valid': valid, 'natural_schedule_failure_only': late_invalid,
            'checks': {**common, **integrity, 'gc_policy': policy_ok}, 'failed_schedule_checks': failed,
            'outcome_accepted_for_gc_comparison': policy_ok and (valid or (not is_defer and late_invalid)),
            'stopped': report.get('stopped'), 'shutdown_clean': report.get('shutdown_clean'),
            'lateness_ms': {'max': max(lateness)/1e6, 'p99': lateness[min(len(lateness)-1, int(.99*len(lateness)))]/1e6,
                            'ge_5ms': sum(v >= 5_000_000 for v in lateness), 'ticks': len(lateness)},
            'gc_policy': policy, 'gc_collection_delta': gc_delta, 'timing': timing,
            'runtime_vehicle_sha256': sha(run/'runtime_vehicle.sdf'),
            'world_normalized_sha256': hashlib.sha256(re.sub(r'/tmp/arena_world_[^/]+/', '/tmp/RUNTIME/',
                (run/'runtime_world.sdf').read_text(encoding='utf-8')).encode()).hexdigest(),
            'meshes_sha256': {p.name: sha(p) for p in sorted(run.glob('batched_*.obj'))},
            'source_sha256': sources, 'profile': report.get('command_profile'), 'spawn_pose': report.get('spawn')}


def comparison(runs):
    both = {mode: [r for r in runs if r['mode'] == mode] for mode in ('native', 'defer')}
    common_fields = ('source_sha256', 'runtime_vehicle_sha256', 'world_normalized_sha256',
                     'meshes_sha256', 'profile', 'spawn_pose')
    same = bool(runs) and all(all(r[k] == runs[0][k] for r in runs) for k in common_fields)
    accepted = same and bool(both['native']) and bool(both['defer']) and all(
        r['outcome_accepted_for_gc_comparison'] for r in runs)
    return {'same_workload': same, 'accepted_bounded_comparison': accepted,
            'replicates': {k: len(v) for k, v in both.items()},
            'max_lateness_ms_by_mode': {k: [r['lateness_ms']['max'] for r in v] for k, v in both.items()},
            'peak_rss_mib_by_mode': {k: [round(r['gc_policy']['peak_rss_after_kib']/1024, 1) for r in v]
                                    for k, v in both.items()},
            'limits': ['각 조건 2회 이하. 통계적 분포나 실차 연산 성능 검증이 아니다.',
                       '생성/명령 적용 시점은 고정하지 않았다. 제동 개선 또는 물리 재현성의 증거가 아니다.',
                       'RSS는 평가 프로세스의 최대값이며 Gazebo/ROS 자식 프로세스 사용량은 포함하지 않는다.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, action='append', required=True)
    parser.add_argument('--plugin-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--contract', type=Path, default=ROOT/'config/tests/scheduled_command_r3_sha256.json')
    args = parser.parse_args()
    contract = json.loads(args.contract.read_text())
    runs = [run_result(p, contract, args.plugin_sha256) for p in args.run]
    result = {'runs': runs, 'comparison': comparison(runs)}
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(json.dumps({r['name']: {k: r[k] for k in ('valid', 'natural_schedule_failure_only',
                      'outcome_accepted_for_gc_comparison', 'lateness_ms', 'gc_collection_delta')} for r in runs}, indent=2))
