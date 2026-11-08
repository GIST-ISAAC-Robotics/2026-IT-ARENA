"""r6 후속 계약: G1/G2 launch 가드의 track 조건만 두 공식 2×5 파생 track으로 확장(부모 r5).

사용: python3 -B prepare_contract.py   # config/tests/handoff_track_guard_r6_sha256.json + M3_SUCCESSORS 한 줄
이전 계약·before.zip은 바꾸지 않는다. 이미 있으면 같은 내용인지만 확인한다.
"""
import hashlib
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
PARENT = 'config/tests/traffic_course_track_r5_sha256.json'
TARGET = 'config/tests/handoff_track_guard_r6_sha256.json'
LAUNCH = 'src/arena_bringup/launch/simulation.launch.py'
TEST = 'tests/test_source_provenance_r0_plus_b3.py'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    snapshot = HERE/'before.zip'
    with zipfile.ZipFile(snapshot) as archive:
        before = sha(archive.read(LAUNCH))
    after = sha((ROOT/LAUNCH).read_bytes())
    assert before != after
    contract = dict(
        schema_version=1,
        scope='G1/G2 handoff launch 가드의 track 조건만 OFFICIAL_2X5_TRACKS(official, official_v2026_11_06)로 확장. '
              'guarded/actor/evidence/mode/reserve 조건·제어 알고리즘·판정 불변, 지도·정답 제어 입력 없음',
        parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
        approved_changes={LAUNCH: dict(before=before, after=after,
                                       reason="_autonomy_executable: track != 'official' → track not in OFFICIAL_2X5_TRACKS")},
        approved_evaluation_scripts={}, new_dependencies={})
    text = json.dumps(contract, ensure_ascii=False, indent=2)+'\n'
    target = ROOT/TARGET
    if target.exists():
        assert target.read_text(encoding='utf-8') == text, '기존 계약과 다름: 덮어쓰지 않음'
    else:
        target.write_text(text, encoding='utf-8', newline='\n')
    test = ROOT/TEST
    data = test.read_bytes()
    anchor = f"M3_SUCCESSORS += ('{PARENT}',)\r\n".encode()
    line = (f"# 11/7 G1/G2 launch 가드 track 조건을 두 공식 2×5 파생 track으로 확장(나머지 조건 불변).\r\n"
            f"M3_SUCCESSORS += ('{TARGET}',)\r\n").encode()
    if line not in data:
        assert data.count(anchor) == 1 and data.rfind(b"M3_SUCCESSORS += ('") == data.index(anchor), '부모가 마지막 후속이 아님'
        test.write_bytes(data.replace(anchor, anchor+line))
    print(json.dumps(dict(target=TARGET, before=before, after=after)))


if __name__ == '__main__':
    main()
