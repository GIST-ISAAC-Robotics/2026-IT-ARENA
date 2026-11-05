"""이동 앞차 추종 연속 전진율 선택 옵션을 r7 뒤 후속 출처 계약 r8로 기록한다. 이전 계약은 바꾸지 않는다.

before 바이트는 continuity_before.zip(수정 직전 HEAD와 같은 바이트)이다.
"""
import hashlib
import json
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PARENT = 'config/tests/lidar_mount_structure_gate_r7_sha256.json'
NAME = 'config/tests/continuous_forward_pace_r8_sha256.json'
CHANGES = {
    'src/arena_autonomy/arena_autonomy/bypass_handoff.py':
        '선택 필드 continuous_forward_pace(기본 OFF, observed_forward_pace 필요). 고정 우회 여유만 남는 분기에서 이번 새 스캔의 '
        'ObservedForwardPace 전진율이 양수일 때만 그 값을 여유 상한에 더한다. G1 출력이 상한, 정지·감속·측방·튐에서는 0',
    'src/arena_autonomy/arena_autonomy/g1_g2_node.py': 'g1_g2_continuous_forward_pace 파라미터(기본 false) 전달',
    'src/arena_bringup/launch/simulation.launch.py':
        'evaluation_handoff_continuous_pace 인자(기본 false). paced 예약+관측 이동에서만 허용해 노드에 전달',
}
SCRIPTS = {
    'scripts/g1_handoff_eval.py': 'continuous_pace 선택 인자 전달과 헤더 확인 항목. 미지정 시 기존 인자와 같음',
    'scripts/validate_g1_traffic.py': '--handoff-continuous-pace 선택 인자(기본 꺼짐) 전달',
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def rows(paths, archive):
    out = {}
    for path, reason in paths.items():
        before, after = archive.read(path), (ROOT/path).read_bytes()
        assert sha(before) != sha(after), path
        out[path] = dict(before=sha(before), after=sha(after), reason=reason)
    return out


def main():
    snapshot = HERE/'continuity_before.zip'
    with zipfile.ZipFile(snapshot) as archive:
        changes, scripts = rows(CHANGES, archive), rows(SCRIPTS, archive)
    contract = dict(
        schema_version=1,
        scope='이동 앞차 추종 속도 톱니 보완 선택 시험. 기본 동작·기록 재생 불변, 추월 기능 아님, 실차 승인 아님',
        parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
        approved_changes=changes, approved_evaluation_scripts=scripts, new_dependencies={})
    text = json.dumps(contract, ensure_ascii=False, indent=2) + '\n'
    target = ROOT/NAME
    if target.exists():
        assert target.read_text(encoding='utf-8') == text, 'existing r8 differs; do not rewrite silently'
    else:
        target.write_text(text, encoding='utf-8', newline='\n')
    print(NAME, {p: r['after'][:12] for p, r in {**changes, **scripts}.items()})


if __name__ == '__main__':
    main()
