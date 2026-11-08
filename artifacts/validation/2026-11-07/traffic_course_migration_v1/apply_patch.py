"""교통·운영 평가 도구 명시 track 선택 패치의 적용과 출처 후속 계약 작성.

사용(진행 중 Gazebo 주행과 그 사후 무결성 기록이 끝난 뒤):
  python3 -B apply_patch.py --check [--root R]          # 기준 해시·부모 계약만 확인, 쓰기 없음
  python3 -B apply_patch.py --apply [--root R] [--parent P] [--name N]
--root 기본값은 이 저장소다. 사전 검증은 scratch 사본을 --root로 주어 수행한다.
적용은 before.zip(적용 직전 4개 스크립트 바이트)을 만들고 patched/ 사본을 복사한 뒤
config/tests/<N>과 M3_SUCCESSORS 한 줄을 추가한다. 이전 계약·등록 증거는 바꾸지 않는다.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
SCRIPTS = ('scripts/validate_g1_traffic.py', 'scripts/official_traffic_course.py',
           'scripts/race_ops_eval.py', 'scripts/summarize_motion_lap.py')
NEW_TEST = 'tests/test_traffic_course_track.py'
PROVENANCE_TEST = 'tests/test_source_provenance_r0_plus_b3.py'
DEFAULT_PARENT = 'config/tests/piecewise_corner_r4_sha256.json'
DEFAULT_NAME = 'config/tests/traffic_course_track_r5_sha256.json'
REASON = ('평가 전용 명시 track 선택: launch track·센서 프로필·평가 중심선/다각형·입력 목록에 같은 월드 전달. '
          '기본 official 인자·출력 유지, 새 코스는 같은 track으로 재등록한 각본만 실행. 제어 입력 변경 없음')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def last_successor(root):
    text = (root/PROVENANCE_TEST).read_text(encoding='utf-8')
    names = re.findall(r"^M3_SUCCESSORS \+= \('([^']+)',\)\s*$", text, flags=re.M)
    return names[-1] if names else None


def check(root, parent):
    base = {rel: sha((HERE/'base'/rel).read_bytes()) for rel in SCRIPTS}
    current = {rel: sha((root/rel).read_bytes()) for rel in SCRIPTS}
    changed = sorted(rel for rel in SCRIPTS if base[rel] != current[rel])
    problems = []
    if changed:
        problems.append(f'기준 사본 이후 바뀐 파일(패치 재작성 필요): {changed}')
    if last_successor(root) != parent:
        problems.append(f'M3_SUCCESSORS 마지막이 {last_successor(root)}이다. --parent를 그 계약으로 맞추고 재검토')
    if not (root/parent).is_file():
        problems.append(f'부모 계약 없음: {parent}')
    if (root/NEW_TEST).exists():
        problems.append(f'{NEW_TEST}가 이미 있다')
    return problems


def apply(root, parent, name):
    target = root/name
    snapshot = root/HERE.relative_to(REPO)/'before.zip'
    if target.exists() or snapshot.exists():
        raise FileExistsError('기존 계약/before.zip 덮어쓰기 금지')
    snapshot.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(snapshot, 'x', zipfile.ZIP_DEFLATED) as archive:
        for rel in SCRIPTS:
            archive.write(root/rel, rel)
    before = {rel: sha((root/rel).read_bytes()) for rel in SCRIPTS}
    for rel in SCRIPTS:
        (root/rel).write_bytes((HERE/'patched'/rel).read_bytes())
    (root/NEW_TEST).write_bytes((HERE/'patched'/NEW_TEST).read_bytes())
    contract = dict(
        schema_version=1,
        scope='교통·운영 평가 도구의 명시 track 선택. 지도·정답 제어 입력 없음, 독립 보호·판정 유지, 옛 각본 자동 이동 없음',
        parent_contract=parent, parent_contract_sha256=sha((root/parent).read_bytes()),
        before_snapshot=snapshot.relative_to(root).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
        approved_changes={},
        approved_evaluation_scripts={rel: dict(before=before[rel], after=sha((root/rel).read_bytes()), reason=REASON)
                                     for rel in SCRIPTS},
        new_dependencies={})
    target.write_text(json.dumps(contract, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')
    test = root/PROVENANCE_TEST
    data = test.read_bytes()
    anchor = f"M3_SUCCESSORS += ('{parent}',)\r\n".encode()
    assert data.count(anchor) == 1, '부모 계약 줄을 찾지 못함'
    line = (f"# 11/7 교통·운영 평가 도구 명시 track 선택(기본 official·제어기 유지).\r\n"
            f"M3_SUCCESSORS += ('{name}',)\r\n").encode()
    test.write_bytes(data.replace(anchor, anchor+line))
    return contract


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--apply', action='store_true')
    parser.add_argument('--root', type=Path, default=REPO)
    parser.add_argument('--parent', default=DEFAULT_PARENT)
    parser.add_argument('--name', default=DEFAULT_NAME)
    args = parser.parse_args()
    root = args.root.resolve()
    problems = check(root, args.parent)
    print(json.dumps(dict(root=str(root), parent=args.parent, name=args.name, problems=problems), ensure_ascii=False))
    if problems:
        return 1
    if args.apply:
        contract = apply(root, args.parent, args.name)
        print(json.dumps({rel: row['after'] for rel, row in contract['approved_evaluation_scripts'].items()}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
