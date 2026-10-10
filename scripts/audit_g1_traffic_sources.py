#!/usr/bin/env python3
"""G1-b 고정 소스/ZIP/수신 산출물 SHA와 실행 판정의 독립 요약.

원본 report/postcoverage를 수정하지 않으며 정답 입력의 graph 경계는 report의
검사값과 실제 등록 토픽 목록으로 대조한다. 런타임 SOURCE_FILES에 속하지 않는다.
"""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def audit_run(run, freeze, post_path=None):
    raw_report = (run/'report.json').read_bytes()
    report = json.loads(raw_report)
    registered = report.get('source_sha256', {})
    zipped, current, freeze_checks = {}, {}, {}
    with zipfile.ZipFile(run/'source_snapshot.zip') as archive:
        names = archive.namelist()
        for name, expected in registered.items():
            zipped[name] = names.count(name) == 1 and sha(archive.read(name)) == expected
            current[name] = (ROOT/name).is_file() and sha((ROOT/name).read_bytes()) == expected
        actor_expected = report.get('actor_sdf_sha256')
        actor_ok = (names.count('evaluation_actor.sdf') == 1 and actor_expected is not None
                    and sha(archive.read('evaluation_actor.sdf')) == actor_expected
                    and sha((run/'evaluation_actor.sdf').read_bytes()) == actor_expected)
    for name, expected in freeze['sha256'].items():
        # 회귀 tests는 런타임 소스 ZIP 외 범위이므로 freeze를 현재 파일과 검사한다.
        freeze_checks[name] = (ROOT/name).is_file() and sha((ROOT/name).read_bytes()) == expected
        if name in registered:
            freeze_checks[name] &= registered[name] == expected
    evidence = {}
    for field, name in (('scans_received_sha256', 'scans_received.jsonl'),
                        ('autonomy_status_sha256', 'autonomy_status.jsonl'),
                        ('ego_poses_sha256', 'ego_poses.jsonl'), ('actor_poses_sha256', 'actor_poses.jsonl'),
                        ('actor_contacts_sha256', 'actor_contacts.jsonl')):
        evidence[name] = (run/name).is_file() and sha((run/name).read_bytes()) == report.get(field)
    post_checks, post = {}, None
    if post_path is not None:
        post = json.loads(post_path.read_text(encoding='utf-8'))
        post_checks = {name: (run/name).is_file() and sha((run/name).read_bytes()) == expected
                       for name, expected in post.get('input_sha256', {}).items()}
    controller_inputs = set(report.get('autonomy_subscriptions', []))
    allowed = {'/camera/color/image_raw', '/scan', '/drive_motor/encoder', '/imu/data', '/clock', '/parameter_events'}
    checks = {'registered_runtime_sources': bool(registered), 'source_zip_matches': all(zipped.values()) and bool(zipped),
        'current_sources_match': all(current.values()) and bool(current),
        'frozen_sources_match': all(freeze_checks.values()) and len(freeze_checks) == len(freeze['sha256']),
        'actor_sdf_matches': actor_ok, 'received_artifact_hashes_match': all(evidence.values()),
        'controller_topic_allowlist': bool(controller_inputs) and controller_inputs.issubset(allowed),
        'actor_pose_only_evaluator': report.get('actor_pose_subscribers') == ['g1b_traffic_validator'],
        'drive_controller_only': report.get('drive_publishers') == ['local_pursuit'],
        'safe_drive_safety_only': report.get('safe_drive_publishers') == ['lidar_safety']}
    if post_path is not None:
        checks.update(postcoverage_inputs_match=bool(post_checks) and all(post_checks.values()),
                      postcoverage_passed=post.get('passed') is True)
    local_raw = {name: sha((run/name).read_bytes()) for name in (
        'scans_received.jsonl', 'autonomy_status.jsonl', 'ego_poses.jsonl', 'actor_poses.jsonl',
        'actor_contacts.jsonl', 'trace.json', 'trajectory.jsonl', 'actuation_trace.json', 'acquisition.json')
        if (run/name).is_file()}
    return {'audit_version': 'g1b_source_collection_v2_raw_hashes', 'run': str(run), 'checks': checks,
        'passed': all(checks.values()), 'report_sha256': sha(raw_report),
        'source_snapshot_zip_sha256': sha((run/'source_snapshot.zip').read_bytes()),
        'runtime_source_files': len(registered), 'source_freeze_files': len(freeze_checks),
        'source_zip_mismatches': [p for p, ok in zipped.items() if not ok],
        'current_source_mismatches': [p for p, ok in current.items() if not ok],
        'freeze_mismatches': [p for p, ok in freeze_checks.items() if not ok],
        'artifact_mismatches': [p for p, ok in evidence.items() if not ok],
        'local_raw_artifact_sha256': local_raw,
        'local_raw_policy': 'detailed received sensor/state/pose/actuation files remain local; preserve summary and hashes',
        'postcoverage_input_mismatches': [p for p, ok in post_checks.items() if not ok],
        'preserved_verdict': {k: report.get(k) for k in ('case', 'seed', 'mode', 'passed', 'run_valid',
            'behavior_gates_passed', 'harness_valid', 'shutdown_clean', 'source_unchanged')},
        'metrics': {'min_sampled_gap_m': report.get('evaluation', {}).get('gap', {}).get('min_sampled_gap_m'),
                    'min_swept_lower_bound_m': report.get('evaluation', {}).get('gap', {}).get('min_swept_lower_bound_m'),
                    'case': report.get('evaluation', {}).get('case_metrics'), 'stop': report.get('stop')},
        'postcoverage': {'path': str(post_path), 'version': post.get('audit_version'),
                        'passed': post.get('passed')} if post is not None else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--freeze', type=Path, default=ROOT/'artifacts/validation/2026-10-07/g1b_unit_v4/source_freeze.json')
    parser.add_argument('--postcoverage', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('source audit output exists')
    result = audit_run(args.run, json.loads(args.freeze.read_text(encoding='utf-8')), args.postcoverage)
    result['source_audit_sha256'] = sha(Path(__file__).read_bytes())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
