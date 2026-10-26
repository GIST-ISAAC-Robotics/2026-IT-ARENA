"""run13 실행 바이트→기본 G1 복귀 비교 옵션 출처. 이전 계약/원시 증거는 불변."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
PARENT = 'config/tests/m3_command_selection_r0_sha256.json'
SNAPSHOT = 'artifacts/validation/2026-10-25/m3_execution_review_v1/run13_selection_closed_loop/run/source_snapshot.zip'
OUTPUT = 'config/tests/m3_baseline_release_r0_sha256.json'
REASONS = {
    'src/arena_autonomy/arena_autonomy/m3_execution_trial.py':
        '기본 false 옆차 앞 통과 뒤 기본 G1 복귀 옵션, 측방 반환 위치 함수 분리(개수 동일)',
    'src/arena_autonomy/arena_autonomy/local_pursuit.py': '명시 M3 release_to_baseline 파라미터 연결·단독 선택 거절',
    'src/arena_bringup/launch/g1_follow.launch.py': '기본 false release_to_baseline 전달',
    'scripts/validate_g1_traffic.py': '선택형 실행 인자 추가·새 출처 계약 사본 포함, 판정선 유지',
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parent = json.loads((ROOT/PARENT).read_text(encoding='utf-8'))
    parent_rows = {**parent['approved_changes'], **parent['approved_evaluation_scripts']}
    with zipfile.ZipFile(ROOT/SNAPSHOT) as bundle:
        changes = {p: dict(before=sha(bundle.read(p)), after=sha((ROOT/p).read_bytes()), reason=r)
                   for p, r in REASONS.items()}
    for p, row in changes.items():
        assert row['before'] == parent_rows[p]['after'] != row['after'], p
    result = dict(schema_version=1, scope='M3 비교 옵션의 출처 기록. 기본 채택·안전 승인 아님',
                  parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
                  before_snapshot=SNAPSHOT, before_snapshot_sha256=sha((ROOT/SNAPSHOT).read_bytes()),
                  approved_changes={p: v for p, v in changes.items() if p.startswith('src/')},
                  approved_evaluation_scripts={p: v for p, v in changes.items() if p.startswith('scripts/')})
    out = ROOT/OUTPUT
    out.write_bytes((json.dumps(result, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))
    print('prepared', len(changes), 'linked files ->', OUTPUT)


if __name__ == '__main__':
    main()
