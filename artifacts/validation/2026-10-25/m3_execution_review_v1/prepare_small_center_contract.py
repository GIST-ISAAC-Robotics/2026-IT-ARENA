"""run19 실행 사본 바이트→작은 중앙 보정 비교 옵션 출처. 이전 계약/원시 증거는 불변. 결과 파일이 있으면 거절."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
PARENT = 'config/tests/m3_release_gate_r0_sha256.json'
SNAPSHOT = 'artifacts/validation/2026-10-25/m3_execution_review_v1/run19_stop_restart/run/source_snapshot.zip'
OUTPUT = 'config/tests/m3_small_center_r0_sha256.json'
REASONS = {
    'src/arena_autonomy/arena_autonomy/m3_execution_trial.py':
        '기본 false 작은 중앙 보정 생략 비교 옵션(직진 예측 편차·곡률 조건, 검사 순서만 변경)',
    'src/arena_autonomy/arena_autonomy/local_pursuit.py': '명시 M3 small_center_straight 파라미터 연결·단독 선택 거절',
    'src/arena_bringup/launch/g1_follow.launch.py': '기본 false small_center_straight 전달',
    'scripts/validate_g1_traffic.py': '선택형 실행 인자(관측 복귀 필수)·새 출처 계약 사본 포함, 판정선 유지',
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    out = ROOT/OUTPUT
    if out.exists():
        raise FileExistsError(out)
    parent = json.loads((ROOT/PARENT).read_text(encoding='utf-8'))
    parent_rows = {**parent['approved_changes'], **parent['approved_evaluation_scripts']}
    with zipfile.ZipFile(ROOT/SNAPSHOT) as bundle:
        changes = {p: dict(before=sha(bundle.read(p)), after=sha((ROOT/p).read_bytes()), reason=r)
                   for p, r in REASONS.items()}
    for p, row in changes.items():
        assert row['before'] != row['after'], p
        if p in parent_rows:
            assert row['before'] == parent_rows[p]['after'], p
    result = dict(schema_version=1, scope='M3 비교 옵션의 출처 기록. 기본 채택·안전 승인 아님',
                  parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
                  before_snapshot=SNAPSHOT, before_snapshot_sha256=sha((ROOT/SNAPSHOT).read_bytes()),
                  approved_changes={p: v for p, v in changes.items() if p.startswith('src/')},
                  approved_evaluation_scripts={p: v for p, v in changes.items() if p.startswith('scripts/')})
    out.write_bytes((json.dumps(result, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))
    print('prepared', len(changes), 'linked files ->', OUTPUT)


if __name__ == '__main__':
    main()
