"""run9 원본 실행 바이트→명령 선택 개선 출처. 이전 계약/원시 증거는 불변."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
PARENT = 'config/tests/m3_execution_r0_sha256.json'
SNAPSHOT = 'artifacts/validation/2026-10-25/m3_execution_review_v1/run9_closed_loop_left/run/source_snapshot.zip'
REASONS = {
    'src/arena_autonomy/arena_autonomy/m3_execution_trial.py': '기본 false 실제 명령 후보 재선택·검사와 재개 상태 분리',
    'src/arena_autonomy/arena_autonomy/local_pursuit.py': '명시 M3 command_fallback 파라미터 연결·단독 선택 거절',
    'src/arena_bringup/launch/g1_follow.launch.py': '기본 false command_fallback 전달',
    'scripts/validate_g1_traffic.py': '선택형 실행 인자·상태 요약의 M3 표식 누락 수정',
    'scripts/g1_traffic_eval.py': '실제 모드 표식 보존 함수·평가 함수 분리, 판정선 유지',
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parent = json.loads((ROOT/PARENT).read_text(encoding='utf-8'))
    with zipfile.ZipFile(ROOT/SNAPSHOT) as bundle:
        changes = {p: dict(before=sha(bundle.read(p)), after=sha((ROOT/p).read_bytes()), reason=r)
                   for p,r in REASONS.items()}
    for p,row in changes.items():
        previous = parent['changes'][p]['after'] if p in parent['changes'] else parent['dependencies'][p]
        assert row['before'] == previous
    result = dict(schema_version=1, scope='M3 비교 옵션의 출처 기록. 기본 채택·안전 승인 아님',
        parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=SNAPSHOT, before_snapshot_sha256=sha((ROOT/SNAPSHOT).read_bytes()),
        approved_changes={p:v for p,v in changes.items() if p.startswith('src/')},
        approved_evaluation_scripts={p:v for p,v in changes.items() if p.startswith('scripts/')})
    (ROOT/'config/tests/m3_command_selection_r0_sha256.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('prepared', len(changes), 'linked files')


if __name__ == '__main__':
    main()
