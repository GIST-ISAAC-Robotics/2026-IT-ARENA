"""원문 스캔의 공간적 순환 인접성. 취득 시각의 정확성을 인증하지 않는다.

명시적인 생성자에서 만든 동일 장면에만 사용한다. 재투영한 방향이나 source_index만
보고 한 바퀴라고 추정하지 않으며, 부분 배열·중복 끝점·동시 취득은 순환하지 않는다.
운영 노드에서 자동 활성화하지 않는다.
"""
from dataclasses import dataclass
import math

import numpy as np

from arena_autonomy.local_scene import LocalScene, scene_from_scan


@dataclass(frozen=True, slots=True)
class ScanCycle:
    scene: LocalScene
    source_angle_min: float
    source_angle_increment: float
    ray_count: int
    full_turn: bool
    rejection: str | None

    def valid_for(self, scene):
        # 객체를 강하게 보유하므로 id 재사용으로 다른 장면의 증거를 꺼낼 수 없다.
        return (self.scene is scene and self.full_turn and self.ray_count == len(scene.rays)
                and all(ray.source_index == i for i, ray in enumerate(scene.rays)))


def scene_with_cycle(ranges, **kwargs):
    """scene_from_scan과 같은 입력으로 원문 metadata를 함께 보존한다.

    공간적 첫/끝 인접성과 취득 시차는 별개다. 시각은 원래 값 그대로 두며 실제
    C1 드라이버의 정렬·반전·복제·host batching 문제를 해결했다는 뜻이 아니다.
    """
    values = tuple(ranges)
    scene = scene_from_scan(values, **kwargs)
    minimum = float(kwargs['angle_min'])
    increment = float(kwargs['angle_increment'])
    count = len(values)
    # ROS float32 각도 metadata의 누적 반올림만 허용한다. 8192광선에서도 한
    # 광선 간격보다 1000배 이상 작으며 빠진 광선을 메우는 허용치는 아니다.
    tolerance = max(1e-9, math.tau * np.finfo(np.float32).eps)
    rejection = None
    if count < 3 or abs(count * increment - math.tau) > tolerance:
        rejection = 'not_endpoint_excluded_full_turn'
    elif any(b.acquired_at_s <= a.acquired_at_s for a, b in zip(scene.rays, scene.rays[1:])):
        rejection = 'acquisition_not_strictly_ordered'
    return ScanCycle(scene, minimum, increment, count, rejection is None, rejection)


def cyclic_count(scene, cycle):
    """유효한 명시 증거만 반환한다. 좌표/시각 변경·다른 장면이면 기존 규칙이다."""
    return cycle.ray_count if isinstance(cycle, ScanCycle) and cycle.valid_for(scene) else None


def adjacent_indices(first, second, *, cycle_count=None):
    delta = np.abs(first - second)
    adjacent = delta <= 1
    if cycle_count is not None:
        adjacent |= delta == cycle_count - 1
    return adjacent
