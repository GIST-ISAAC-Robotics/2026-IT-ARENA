"""원 선택 경로를 현재 센서/운동 입력으로 재검사한다. 기본 노드에는 미연결이다.

`scope`/`supported`는 처리 범위만 나타내며 진행 허가가 아니다. `rebase`는
미지원 시 None, 현재 검사에서 거절하면 0속도 결과를 반환한다. ROS 발행,
상태 전이, 시각만 바꾸는 재발행을 하지 않는다. 독립 보호층은 별도로 필요하다.
"""
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan

from . import _parallel, _guard, _present, _released, _released_parallel


def _absent_guard_scope(output):
    if not _guard.supported(output):
        return False
    normal = output[2]['normal']
    sides = normal.get('selector_sides') or {}
    partial = normal.get('partial_observation') or {}
    return (set(sides) == {'left', 'right'}
            and all(side.get('present') is False for side in sides.values())
            and partial.get('allowed') is True
            and partial.get('reason') == 'fully_observed')


def scope(output):
    """r174 원 실행의 우선순위와 범위를 유지한다. 부적합 경로를 새로 만들지 않는다."""
    if _parallel.supported(output):
        return ('solo_baseline' if output[2]['normal']['selector_choice'] == 'baseline'
                else 'solo_or_guard_parallel')
    if _absent_guard_scope(output):
        return 'absent_guard_baseline'
    if _released.supported(output):
        return 'released_baseline'
    if _present.supported(output):
        return 'present_guard_baseline'
    if _released_parallel.supported(output):
        return 'released_parallel'
    return None


def supported(output):
    return scope(output) is not None


def rebase(output, submitted, current, scene, history, cfg):
    """같은 선택 경로/원 경로 시각을 유지하며 현재 기하와 각 상태 조건을 재계산한다."""
    selected = scope(output)
    if selected in ('solo_baseline', 'solo_or_guard_parallel'):
        return _parallel.rebase(output, submitted, current, scene, history, cfg)
    if selected == 'absent_guard_baseline':
        scan = MeasurementScan(**thaw(current['scan']['measurements']))
        return _guard.rebase(output, submitted, current, scan, scene, history, cfg)
    if selected == 'released_baseline':
        return _released.rebase(output, submitted, current, scene, history, cfg)
    if selected == 'present_guard_baseline':
        return _present.rebase(output, submitted, current, scene, history, cfg)
    if selected == 'released_parallel':
        return _released_parallel.rebase(output, submitted, current, scene, history, cfg)
    return None
