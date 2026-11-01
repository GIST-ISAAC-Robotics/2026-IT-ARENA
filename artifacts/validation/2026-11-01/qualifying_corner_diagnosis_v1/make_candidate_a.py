"""스크래치 사본에만 후보 A를 적용한다. 저장소 src는 건드리지 않는다.

실행: python3 -I make_candidate_a.py <snap_combined> <snap_candA> <out.patch>
"""
import difflib
import shutil
import sys
from pathlib import Path

src, dst, patch = map(Path, sys.argv[1:4])
if dst.exists():
    shutil.rmtree(dst)
shutil.copytree(src, dst)
target = dst / 'src/arena_autonomy/arena_autonomy/local_follow.py'
raw = target.read_bytes().decode('utf-8')
crlf = '\r\n' in raw
old = raw.replace('\r\n', '\n')     # 실행 스냅숏의 줄바꿈을 유지한 채 내용만 고친다.
new = old
HELPER = '''def path_agrees_arc(candidate, reference, tolerance, lo=.05, hi=1.):
    """차체 x 창이 비는 급커브에서 두 경로의 같은 호길이 구간을 같은 기준으로 비교한다.

    path_agrees와 마찬가지로 공통 구간 0.25 m·각 5점을 요구하며 외삽하지 않는다.
    """
    def ahead(path):
        path = np.asarray(path, dtype=float)
        path = path[int(np.argmin(np.linalg.norm(path, axis=1))):]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        keep = (arc >= lo) & (arc <= hi)
        return path[keep], arc[keep]
    (first, first_arc), (second, second_arc) = ahead(candidate), ahead(reference)
    if len(first) < 5 or len(second) < 5:
        return False
    start, end = max(first_arc[0], second_arc[0]), min(first_arc[-1], second_arc[-1])
    if end-start < .25:
        return False
    first = first[(first_arc >= start-1e-12) & (first_arc <= end+1e-12)]
    second = second[(second_arc >= start-1e-12) & (second_arc <= end+1e-12)]
    if len(first) < 5 or len(second) < 5:
        return False
    distances = np.linalg.norm(first[:, None, :]-second[None, :, :], axis=2)
    return max(np.percentile(np.min(distances, axis=0), 80),
               np.percentile(np.min(distances, axis=1), 80)) <= tolerance


'''
BRANCH = '''                    if (not eligible and cfg.low_speed_arc_reacquire and abs(speed) <= .10
                            and sufficient(opposite, other)
                            and path_agrees_arc(candidate, opposite, cfg.boundary_agreement_m)):
                        # 측정 평균 속도가 낮을 때만 쓴다. 단일 축이라 실제 정지를 증명하지 않으며
                        # 이후 확인 횟수·점유·재개 여유·독립 보호는 기존 판정을 그대로 따른다.
                        eligible = True
                        detail['bilateral_reference'] = 'low_speed_arc'
'''
FIELD = '    preview_turn_speed: bool = False\n'
AGREE = '                    eligible = sufficient(opposite, other) and path_agrees(candidate, opposite, cfg.boundary_agreement_m)\n'
SUPPORT = 'def boundary_support(points, path, side, config):\n'
EXEMPT = "'preview_turn_speed', 'boundary_evidence_mode',"
GAP = "                return self._hold('follow_gap_hold', detail)\n"
STEER = '''                if cfg.low_speed_arc_reacquire and abs(speed) <= .10:
                    # 속도 0을 유지한 채 조향만 경로 쪽으로 돌린다. 전진은 재개 여유와 독립 보호가 정한다.
                    # 스캔 말미의 시간 여유 증가로 생긴 낮은 값이 새 스캔의 재개 계수를 지우지 않게 한다.
                    self.held = True
                    return 0., steering, {**detail, 'reason': 'follow_steer_in_place', 'behavior': 'HOLD'}
'''
edits = [(FIELD, FIELD + '    low_speed_arc_reacquire: bool = False  # 시험 선택. 낮은 측정 속도의 양측 경로 재획득만.\n'),
         (EXEMPT, "'preview_turn_speed', 'low_speed_arc_reacquire', 'boundary_evidence_mode',"),
         (SUPPORT, HELPER + SUPPORT), (AGREE, AGREE + BRANCH), (GAP, STEER + GAP)]
for a, b in edits:
    assert new.count(a) == 1, a
    new = new.replace(a, b)
target.write_bytes((new.replace('\n', '\r\n') if crlf else new).encode('utf-8'))
text = ''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                                    'a/src/arena_autonomy/arena_autonomy/local_follow.py',
                                    'b/src/arena_autonomy/arena_autonomy/local_follow.py', n=2))
patch.write_bytes(text.encode('utf-8'))
print('patched', len(text.splitlines()), 'diff lines')
