"""독립 guard의 현재 입력 근거. 경로/거리 보간 또는 자유 공간 증명이 아니다."""
import math
import numpy as np
from arena_autonomy.scan_input import audit_scan_front, scan_angles


def front_evidence(scan, yaw):
    # lidar_safety.on_scan의 mask_scan을 지난 입력이다. CAD 거리 bands 안의 자기
    # 반환은 이미 unknown이다. 각도 sector 전체를 다시 제외하면 같은 각도의
    # 더 먼 외부 반환까지 지워지므로 여기서는 현재 남아 있는 반환만 센다.
    metadata = audit_scan_front(scan, yaw, metadata_only=True)
    result = dict(accepted=False, reason='angular_metadata_hole', unknown_filled=False,
        residual_hidden_object_risk=True, policy='independent_current_command_v1',
        scope='current returns and command occupancy; no global visibility proof')
    if metadata.reason != 'ok':
        return result
    angles = (scan_angles(scan)+yaw+math.pi) % math.tau-math.pi
    values = np.asarray(scan.ranges, dtype=float)
    front = np.abs(angles) <= math.radians(100)+1e-6
    if np.any(values[front] < scan.range_min):
        return {**result, 'reason':'invalid_near_return'}
    valid = np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max)
    count = int(np.count_nonzero(front & valid))
    return {**result, 'accepted':count > 0,
        'reason':'current_front_returns' if count else 'front_unobserved', 'front_return_samples':count}
