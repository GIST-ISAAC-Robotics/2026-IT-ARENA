"""교통 선택기의 현재 SOLO 구간에 한정한 기존 저속 공백 정책 선택.

무반환을 자유 공간으로 바꾸지 않는다. 실제 허가는 별도 partial 정책과 독립
보호층이 검사한다. 상태/소실 이력은 지우지 않고 현재 반환의 옆차도 대조한다.
"""
import math
import numpy as np
from arena_autonomy.scan_input import scan_angles


def eligible(selector, scene, baseline, scan, yaw, explained):
    if (selector is None or selector.mode != 'SOLO' or set(selector.sides) != {'left', 'right'}
            or any(s.get('seen') is not False or s.get('present') is not False
                   for s in selector.sides.values())):
        return False
    # 알려진 구조만 가린 일반 스캔에는 추가 경로 질의를 만들지 않는다.
    values = np.asarray(scan.ranges, dtype=float)
    missing = ~(np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max))
    if explained:
        missing[list(explained)] = False
    angles = (scan_angles(scan) + yaw + math.pi) % math.tau - math.pi
    if not np.any(missing & (np.abs(angles) <= math.radians(100)+1e-6)):
        return False
    from arena_autonomy.pass_release_trial import threat_view, SIDES
    view = threat_view(scene, baseline[2].get('path_points'), selector.lat)
    return view is not None and all(view[name]['present'] is False for name, _ in SIDES)
