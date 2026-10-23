"""선택형 거리 증거의 오프라인 상세 진단 진입점. 자체 주행 허가는 없다."""
from arena_autonomy.corner_distance import corner_distance_evidence as _evidence


def corner_distance_evidence(points, reference, side, config):
    return _evidence(points, reference, side, config, detailed=True)
