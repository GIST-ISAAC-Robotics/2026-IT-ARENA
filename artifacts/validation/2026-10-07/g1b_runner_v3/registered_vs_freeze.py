#!/usr/bin/env python3
"""실행 report의 등록 런타임 소스·ZIP을 그 실행 때 쓴 freeze와 대조하고 사후 감사의 사례/모드 적용성을 나눈다.

`audit_g1_traffic_sources.py`는 현재 작업 트리와도 비교하므로, 뒤에 평가기가 바뀐 이전 실행에서는
`current/frozen_sources_match`가 거짓이 된다. 이 보조 검사는 그 실행 당시 freeze만 기준으로 삼는다.
`audit_g1_traffic_coverage.py` v2의 경계 진행·HOLD 자세 검사는 G1 상태(`mode=g1_follow`, `behavior`)
전용이라 R0에는 적용되지 않는다. 판정·원본 파일은 바꾸지 않는다(읽기 전용).
"""
import hashlib
import json
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
G1_ONLY = ('boundary_reference_progresses', 'boundary_source_progresses', 'hold_matches_dense_pose')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def check(name, freeze_path):
    run = ROOT/'artifacts/validation/2026-10-07'/name
    report = json.loads((run/'report.json').read_text())
    freeze = json.loads((ROOT/freeze_path).read_text(encoding='utf-8'))['sha256']
    registered = report['source_sha256']
    with zipfile.ZipFile(run/'source_snapshot.zip') as archive:
        names = archive.namelist()
        zip_ok = all(names.count(n) == 1 and sha(archive.read(n)) == h for n, h in registered.items())
    shared = sorted(set(freeze) & set(registered))
    post = json.loads((run/'postcoverage_v2.json').read_text(encoding='utf-8'))
    failed = [k for k, v in post['checks'].items() if not v]
    mode = report.get('mode')
    applicable_failed = [k for k in failed if not (mode == 'r0' and k in G1_ONLY)]
    return {'run': name, 'mode': mode, 'evaluator_version': report.get('evaluation', {}).get('evaluator_version'),
            'freeze': freeze_path, 'registered_runtime_files': len(registered), 'source_zip_matches_registered': zip_ok,
            'shared_files_with_freeze': len(shared),
            'registered_matches_freeze': all(registered[f] == freeze[f] for f in shared) and bool(shared),
            'freeze_mismatches': [f for f in shared if registered[f] != freeze[f]],
            'postcoverage_failed_checks': failed,
            'postcoverage_not_applicable_to_mode': [k for k in failed if mode == 'r0' and k in G1_ONLY],
            'postcoverage_applicable_checks_passed': not applicable_failed,
            'report_sha256': sha((run/'report.json').read_bytes()),
            'source_snapshot_zip_sha256': sha((run/'source_snapshot.zip').read_bytes())}


def main():
    freeze_path, out_dir, names = sys.argv[1], ROOT/sys.argv[2], sys.argv[3:]
    out_dir.mkdir(parents=True, exist_ok=True)
    for name in names:
        result = check(name, freeze_path)
        (out_dir/f'{name}.registered_vs_freeze.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps({k: result[k] for k in ('run', 'mode', 'evaluator_version', 'source_zip_matches_registered',
                                                 'registered_matches_freeze', 'postcoverage_not_applicable_to_mode',
                                                 'postcoverage_applicable_checks_passed')}))


if __name__ == '__main__':
    main()
