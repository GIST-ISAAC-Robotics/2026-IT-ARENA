"""검출 영역 수정과 launch 줄바꿈 복원의 소스 계약. official_facility_r1과 run1 기록은 다시 쓰지 않는다."""
from hashlib import sha256
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PARENT = 'config/tests/official_facility_r1_sha256.json'
OUT = ROOT/'config/tests/official_start_region_r1_sha256.json'
REASONS = {
    'src/arena_autonomy/arena_autonomy/race_start.py': '고정 상단 58%와 초록 탐색 사각형 전체 포함 조건을 관측 배열 기준으로 교체. 전체 빨강이 온전한 배열 하나일 때만 상단 밖을 인정하고 잘린 광원은 판독하지 않는다.',
    'src/arena_autonomy/arena_autonomy/race_start_local.py': 'bank_window를 기반 검출기로 이동. 판단 내용 변경 없음.',
    'src/arena_bringup/launch/simulation.launch.py': '저장소 줄바꿈(CRLF) 복원. 내용 변경 없음. run1 실행 바이트는 before.zip과 run1 기록에 보존.',
}


def digest(data):
    return sha256(data).hexdigest()


def main():
    before = HERE/'before.zip'
    rows = {}
    with zipfile.ZipFile(before) as archive:
        for path, reason in REASONS.items():
            rows[path] = dict(before=digest(archive.read(path)), after=digest((ROOT/path).read_bytes()), reason=reason)
    contract = dict(schema_version=1, scope='출발 판독 영역을 관측 배열 기준으로 교정하고 launch 줄바꿈을 복원한다. 시험 초록·소등·배열·시간 조건과 차량 장착은 유지한다.',
                    parent_contract=PARENT, parent_contract_sha256=digest((ROOT/PARENT).read_bytes()),
                    before_snapshot=before.relative_to(ROOT).as_posix(), before_snapshot_sha256=digest(before.read_bytes()),
                    approved_changes=rows, approved_evaluation_scripts={}, new_dependencies={})
    OUT.write_bytes((json.dumps(contract, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))


if __name__ == '__main__':
    main()
