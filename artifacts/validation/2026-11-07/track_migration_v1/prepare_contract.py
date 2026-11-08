"""r16 뒤에 v2026.11.06 코스 이관 출처 계약을 연결한다. 등록 후 덮어쓰지 않는다.

before_tree/는 `git cat-file --filters HEAD:<path>`로 복원한 변경 전 작업 트리 바이트이며
before/sha256_before.txt(편집 전 기록)와 일치를 확인한 뒤 before_r1.zip으로 묶는다.
"""
from pathlib import Path
import hashlib
import json
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PARENT = 'config/tests/moving_pass_handoff_r16_sha256.json'
TARGET = ROOT / 'config/tests/track_migration_v2026_11_06_r1_sha256.json'
CHANGES = {
    'src/arena_bringup/launch/simulation.launch.py':
        '명시 track:=official_v2026_11_06 선택과 2×5 신호 공식 월드 목록 추가. 기본 official과 제어기 동작 유지',
    'src/arena_bringup/launch/demo.launch.py': 'track 인자 전달(기본 official). 다른 인자·자율주행 구성 유지',
    'scripts/build_official_track.py':
        '릴리스별 파일 수·공식 폭·실행 월드 경로 등록과 v2026.11.06 추가. v2026.11.01 출력은 재생성 바이트 동일',
}
EVALUATION = {
    'scripts/validate_basic_autonomy.py': '--track 명시 시 launch와 평가 월드(중심선·scene·충돌)를 같은 경로로 선택. 기본 official 유지',
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    recorded = {}
    for line in (HERE / 'before/sha256_before.txt').read_text(encoding='utf-8').splitlines():
        digest, path = line.split(' *', 1)
        recorded[path] = digest
    snapshot = HERE / 'before_r1.zip'
    paths = sorted({**CHANGES, **EVALUATION})
    if not snapshot.exists():
        with zipfile.ZipFile(snapshot, 'w', zipfile.ZIP_DEFLATED) as archive:
            for path in paths:
                data = (HERE / 'before_tree' / path).read_bytes()
                assert sha(data) == recorded[path], path
                info = zipfile.ZipInfo(path, date_time=(2026, 11, 7, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(info, data)
    with zipfile.ZipFile(snapshot) as archive:
        rows = lambda table: {p: dict(before=sha(archive.read(p)), after=sha((ROOT / p).read_bytes()), reason=reason)
                              for p, reason in table.items()}
        changes, evaluation = rows(CHANGES), rows(EVALUATION)
    for path, row in {**changes, **evaluation}.items():
        assert row['before'] == recorded[path] != row['after'], path
    result = dict(schema_version=1,
                  scope='공식 v2026.11.06 별도 실행 월드 등록·명시 선택. 기본 월드·제어기·카메라 변경 아님, 주행 검증 아님.',
                  parent_contract=PARENT, parent_contract_sha256=sha((ROOT / PARENT).read_bytes()),
                  before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
                  approved_changes=changes, approved_evaluation_scripts=evaluation, new_dependencies={})
    content = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if TARGET.exists():
        assert TARGET.read_text(encoding='utf8') == content, '등록 이후 바이트 변경'
    else:
        TARGET.write_text(content, encoding='utf8', newline='\n')
    print(TARGET.relative_to(ROOT))


if __name__ == '__main__':
    main()
