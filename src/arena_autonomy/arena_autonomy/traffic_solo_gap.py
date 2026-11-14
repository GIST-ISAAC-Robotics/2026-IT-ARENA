"""교통 선택기의 현재 SOLO 구간에 한정한 기존 저속 공백 정책 선택.

무반환을 자유 공간으로 바꾸지 않는다. 실제 허가는 별도 partial 정책과 독립
보호층이 검사한다. 상태/소실 이력은 지우지 않고 현재 반환의 옆차도 대조한다.
"""
import math
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
import numpy as np
from arena_autonomy.scan_input import scan_angles


@dataclass(frozen=True)
class ThreatViewTransfer:
    """이번 eligible 계산만 select에 전달한다. 다른 scene/경로에는 쓰지 않는다."""
    scene: object
    path: object
    config: object
    view: object

    def matches(self, scene, path, config):
        return (scene is self.scene and config == self.config
                and np.array_equal(self.path, np.asarray(
                    path if path is not None else [], dtype=float).reshape(-1, 2)))


_selection_view = ContextVar('traffic_solo_gap_selection_view', default=None)


@contextmanager
def selection_view(selector, transfer):
    # override/runtime wrapper 서명은 유지한다. 중첩/예외에서도 이전 호출로 복원한다.
    token = _selection_view.set((selector, transfer))
    try:
        yield
    finally:
        _selection_view.reset(token)


def view_for_selection(selector, scene, path):
    from arena_autonomy.pass_release_trial import threat_view
    selected = _selection_view.get()
    if selected is not None and selected[0] is selector:
        transfer = selected[1]
        if transfer is not None and transfer.matches(scene, path, selector.lat):
            return transfer.view
    return threat_view(scene, path, selector.lat)


def eligible(selector, scene, baseline, scan, yaw, explained, *, view_transfer=None):
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
    if view_transfer is not None:
        path = baseline[2].get('path_points')
        snapshot = np.array(path if path is not None else [], dtype=float, copy=True).reshape(-1, 2)
        snapshot.setflags(write=False)
        view_transfer.append(ThreatViewTransfer(scene, snapshot, selector.lat, view))
    return view is not None and all(view[name]['present'] is False for name, _ in SIDES)
