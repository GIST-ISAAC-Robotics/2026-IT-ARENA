"""사용자 지정 임시 차체 중심 사각형의 근거리 자체 반환만 제외한다.

실장 보정이나 자유 공간 인증이 아니다. 원시 입력을 고치거나 가림 뒤 광선을
만들지 않으며, 제외한 원천 인덱스를 masked로 보존한다.
"""
import copy
from dataclasses import replace
import json
import math

import numpy as np

from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.scan_input import scan_angles


class MaskedLaserScan:
    """ROS 메시지 자체는 보존하고 내부 소비자용 거리/분류만 가진다."""
    def __init__(self, source, ranges, indices):
        self.source = source
        self.ranges = list(ranges)
        self.self_masked_indices = tuple(indices)

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, 'source'), name)

    def __deepcopy__(self, memo):
        return type(self)(copy.deepcopy(self.source, memo), self.ranges, self.self_masked_indices)


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name+' must be finite numeric')
    return float(value)


class QuickSelfMask:
    """취득 순간 센서→base_link 변환 뒤 사각형에 든 유효 반환만 고른다."""
    def __init__(self, text, motion_config):
        raw = json.loads(text)
        keys = {'schema', 'enabled', 'frame_id', 'status', 'center_xy_m',
                'body_length_m', 'body_width_m', 'margin_each_side_m'}
        if not isinstance(raw, dict) or set(raw) != keys:
            raise ValueError('invalid quick self mask keys')
        if (raw['schema'] != 'arena_quick_self_mask_v1' or type(raw['enabled']) is not bool
                or raw['frame_id'] != 'base_link'
                or raw['status'] != 'temporary_user_approximation_unmeasured'):
            raise ValueError('invalid quick self mask identity/frame/status')
        center = raw['center_xy_m']
        if not isinstance(center, (list, tuple)) or len(center) != 2:
            raise ValueError('self mask center needs two components')
        cx, cy = (_number(v, 'center_xy_m') for v in center)
        length = _number(raw['body_length_m'], 'body_length_m')
        width = _number(raw['body_width_m'], 'body_width_m')
        margin = _number(raw['margin_each_side_m'], 'margin_each_side_m')
        if length <= 0 or width <= 0 or margin < 0:
            raise ValueError('positive self mask size and nonnegative margin required')
        self.bounds = (cx-length/2-margin, cx+length/2+margin,
                       cy-width/2-margin, cy+width/2+margin)
        if not all(math.isfinite(v) for v in self.bounds):
            raise ValueError('self mask bounds overflow')
        self.sensor = tuple(_number(getattr(motion_config, k), k)
                            for k in ('lidar_x_m', 'lidar_y_m', 'lidar_yaw_rad'))
        self.frame = motion_config.lidar_frame_id
        self.enabled = raw['enabled']
        self.profile = raw
        self.last_raw_scan = None
        self.last_counts = dict(scan_received=False, masked_returns=0, input_rays=0)

    def apply(self, scan):
        self.last_raw_scan = scan
        self.last_counts = dict(scan_received=True, masked_returns=0, input_rays=len(scan.ranges))
        if not self.enabled:
            return scan
        if scan.header.frame_id != self.frame:
            raise ValueError('self mask scan frame differs from configured lidar frame')
        values = np.asarray(scan.ranges, dtype=float)
        valid = np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max)
        # 무반환/범위 밖 입력에는 투영이나 mask 승격을 하지 않는다.
        ids = np.flatnonzero(valid)
        sx, sy, yaw = self.sensor
        bearings = scan_angles(scan)[ids]+yaw
        x = sx+values[ids]*np.cos(bearings)
        y = sy+values[ids]*np.sin(bearings)
        left, right, bottom, top = self.bounds
        # 삼각함수의 경계 roundoff만 허용한다. 영역 외 추가 여유가 아니다.
        eps = 1e-12
        selected = ids[(x >= left-eps) & (x <= right+eps)
                       & (y >= bottom-eps) & (y <= top+eps)]
        self.last_counts['masked_returns'] = int(len(selected))
        if not len(selected):
            return scan
        ranges = values.copy()
        ranges[selected] = np.nan
        indices = tuple(sorted(set(getattr(scan, 'self_masked_indices', ())) | set(selected.tolist())))
        if isinstance(scan, MeasurementScan):
            return replace(scan, ranges_m=tuple(ranges.tolist()), self_masked_indices=indices)
        return MaskedLaserScan(scan, ranges.tolist(), indices)

    def snapshot(self):
        return dict(**self.profile, bounds_xy_m=list(self.bounds), sensor_xy_yaw=list(self.sensor),
                    **self.last_counts, raw_input_preserved=True, occluded_space='unknown')


def configure_node(node, callback):
    """계획기/독립 보호의 같은 파라미터와 입력 경계. 빈 문자열은 기존 경로 그대로."""
    text = node.declare_parameter('lidar_self_mask_json', '').value
    if not isinstance(text, str):
        raise ValueError('lidar_self_mask_json must be a JSON string')
    if not text:
        return callback
    if getattr(node, 'motion', None) is None:
        raise ValueError('self mask requires explicit motion lidar extrinsics')
    mask = QuickSelfMask(text, node.motion.history.config)
    node.quick_self_mask = mask

    def receive(scan):
        callback(mask.apply(scan))
    return receive
