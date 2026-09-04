"""ROS와 무관한 픽셀 인식·라이다 벽 추종 계산. 정답 위치 입력 없음."""
from dataclasses import dataclass
import math

import cv2
import numpy as np


def image_rgb(message):
    if message.encoding not in ("rgb8", "bgr8"):
        raise ValueError("RGB8/BGR8 영상이 필요합니다.")
    image = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
    image = image[:, :message.width * 3].reshape(message.height, message.width, 3)
    return image[:, :, ::-1].copy() if message.encoding == "bgr8" else image.copy()


def image_depth_m(message):
    """ROS ``32FC1`` 깊이 영상을 행 패딩과 엔디언 표시를 보존해 읽습니다."""
    if message.encoding != "32FC1" or message.step < message.width * 4:
        raise ValueError("32FC1 깊이 영상이 필요합니다.")
    dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
    row_values = message.step // dtype.itemsize
    depth = np.frombuffer(message.data, dtype=dtype).reshape(message.height, row_values)
    return depth[:, :message.width].astype(np.float32, copy=False)


def road_center_angle(rgb, fx, cx, row_fractions=(.62, .70, .78),
                      minimum_run_fraction=.07):
    """단독 주행 시험에서 어두운 무채색 노면 띠의 중심 방위를 찾습니다.

    깊이 통로만으로는 같은 높이의 도로와 잔디를 구분할 수 없습니다. 공식
    시험 월드의 회색 노면을 RGB로 보조 분할하되, 여러 하단 행에서 충분히
    넓은 연속 띠가 반복될 때만 사용합니다. 다른 차량·조명·실물 재질에 대한
    일반화는 의도하지 않은 설명 가능한 시뮬레이션 기준선입니다.
    """
    rgb = np.asarray(rgb, dtype=np.uint8)
    numeric = (fx, cx, minimum_run_fraction, *row_fractions)
    if (rgb.ndim != 3 or rgb.shape[2] != 3 or
            not all(math.isfinite(float(value)) for value in numeric)):
        raise ValueError("RGB 노면 중심 매개변수가 올바르지 않습니다.")
    if (fx <= 0 or not 0 < minimum_run_fraction < 1 or
            not row_fractions or any(not 0 < value < 1 for value in row_fractions)):
        raise ValueError("RGB 노면 중심 범위가 올바르지 않습니다.")
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    saturation, value = hsv[:, :, 1], hsv[:, :, 2]
    road = ((saturation <= 38) & (value >= 50) & (value <= 132)).astype(np.uint8)
    road = cv2.morphologyEx(road, cv2.MORPH_CLOSE, np.ones((3, 11), np.uint8))
    minimum_width = max(8, int(round(rgb.shape[1] * minimum_run_fraction)))
    observations = []
    widths = []
    centers = []
    for fraction in row_fractions:
        row = int(round((rgb.shape[0] - 1) * float(fraction)))
        low, high = max(0, row - 2), min(rgb.shape[0], row + 3)
        line = np.mean(road[low:high], axis=0) >= .6
        starts = np.flatnonzero(line & ~np.r_[False, line[:-1]])
        ends = np.flatnonzero(line & ~np.r_[line[1:], False]) + 1
        candidates = []
        for start, end in zip(starts, ends):
            width = int(end - start)
            if width < minimum_width:
                continue
            center = (float(start) + float(end - 1)) / 2.0
            # 넓은 띠를 우선하되, 벽의 어두운 그림자를 노면으로 고르는 일을
            # 줄이기 위해 광축에서 멀수록 약하게 감점합니다.
            score = width - .30 * abs(center - float(cx))
            candidates.append((score, width, center))
        if not candidates:
            continue
        _, width, center = max(candidates)
        observations.append(math.atan2(float(cx) - center, float(fx)))
        widths.append(width)
        centers.append(center)
    if len(observations) < 2:
        return None, {"road_center_reason": "insufficient_gray_road_rows",
                      "road_center_rows": len(observations)}
    angle = float(np.median(observations))
    return angle, {
        "road_center_reason": "gray_road_center",
        "road_center_angle_rad": angle,
        "road_center_rows": len(observations),
        "road_center_median_width_px": float(np.median(widths)),
        "road_center_median_x_px": float(np.median(centers)),
    }


def depth_wall_points(depth, fx, fy, cx, cy, camera_xyz=(0.055, 0.0, 0.075),
                      stride=4, minimum_height=.09, maximum_height=.28,
                      minimum_forward=.18, maximum_forward=1.45):
    """전방 깊이에서 벽 높이의 표본만 차량 평면 ``(x, y)``로 투영합니다.

    ROS optical frame의 ``z``가 전방, ``x``가 우측, ``y``가 아래쪽이라는
    규약을 차량 ``x`` 전방, ``y`` 좌측, ``z`` 위쪽으로 변환합니다. 평지와
    높은 신호등은 제외하지만, 실제 스테레오 결측·왜곡을 복구하지는 않습니다.
    """
    depth = np.asarray(depth, dtype=np.float32)
    values = [fx, fy, cx, cy, *camera_xyz, stride, minimum_height, maximum_height,
              minimum_forward, maximum_forward]
    if depth.ndim != 2 or not all(math.isfinite(float(value)) for value in values):
        raise ValueError("깊이 투영 매개변수가 올바르지 않습니다.")
    if (fx <= 0 or fy <= 0 or int(stride) != stride or stride < 1 or
            minimum_height >= maximum_height or minimum_forward >= maximum_forward):
        raise ValueError("깊이 투영 범위가 올바르지 않습니다.")
    rows = np.arange(0, depth.shape[0], int(stride))
    cols = np.arange(0, depth.shape[1], int(stride))
    sampled = depth[np.ix_(rows, cols)].astype(float, copy=False)
    u, v = np.meshgrid(cols, rows)
    cam_x, cam_y, cam_z = (float(value) for value in camera_xyz)
    with np.errstate(invalid="ignore", over="ignore"):
        forward = sampled + cam_x
        left = -(u - float(cx)) * sampled / float(fx) + cam_y
        height = cam_z - (v - float(cy)) * sampled / float(fy)
    valid = (np.isfinite(sampled) & (sampled > 0) &
             (forward >= minimum_forward) & (forward <= maximum_forward) &
             (height >= minimum_height) & (height <= maximum_height))
    return np.column_stack((forward[valid], left[valid])), int(np.count_nonzero(valid))


def depth_lateral_clearance(points, side, minimum_forward=.18, maximum_forward=.70):
    """가까운 전방 벽 표본의 강건한 횡방향 거리를 반환합니다."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (2,) or side not in ("left", "right"):
        raise ValueError("벽 표본 또는 방향이 올바르지 않습니다.")
    sign = 1.0 if side == "left" else -1.0
    lateral = points[:, 1] * sign
    keep = ((points[:, 0] >= minimum_forward) & (points[:, 0] <= maximum_forward) &
            (lateral >= .05) & (lateral <= 1.0) & np.all(np.isfinite(points), axis=1))
    candidates = lateral[keep]
    if len(candidates) < 20:
        return None
    return float(np.median(candidates))


def depth_gap_command(depth, fx, cx, cy, side="left", max_steering=.43,
                      max_speed=.35, min_speed=.14, maximum_range=4.0,
                      free_clearance=.55, row_half_height=14, column_stride=4):
    """전방 깊이 영상의 열린 통로를 찾아 지도 없이 조향합니다.

    카메라 광축 높이 부근의 수평 띠에서 각 방위의 가장 가까운 물체를
    구합니다. 충분히 먼 연속 구간 중 폭·깊이·정면 근접도를 우선하고,
    거의 같은 후보가 여럿일 때만 ArUco가 정한 벽 쪽을 보조 기준으로
    사용합니다. 무한대 깊이는 카메라 최대 거리까지 열린 것으로 봅니다.
    """
    depth = np.asarray(depth, dtype=np.float32)
    numeric = (fx, cx, cy, max_steering, max_speed, min_speed, maximum_range,
               free_clearance, row_half_height, column_stride)
    if (depth.ndim != 2 or side not in ("left", "right") or
            not all(math.isfinite(float(value)) for value in numeric)):
        raise ValueError("깊이 통로 조향 매개변수가 올바르지 않습니다.")
    if (fx <= 0 or max_steering <= 0 or max_speed <= 0 or min_speed <= 0 or
            min_speed > max_speed or maximum_range <= free_clearance or
            int(row_half_height) != row_half_height or row_half_height < 2 or
            int(column_stride) != column_stride or column_stride < 1):
        raise ValueError("깊이 통로 조향 범위가 올바르지 않습니다.")
    center_row = int(round(cy))
    r0 = max(0, center_row - int(row_half_height))
    r1 = min(depth.shape[0], center_row + int(row_half_height) + 1)
    columns = np.arange(0, depth.shape[1], int(column_stride))
    if r1 <= r0 or not len(columns):
        return 0.0, 0.0, {"reason": "depth_invalid"}
    band = depth[r0:r1:2, :][:, columns].astype(float, copy=False)
    finite = np.isfinite(band)
    valid = finite & (band >= .12) & (band <= maximum_range)
    finite_count = int(np.count_nonzero(valid))
    too_close = np.isneginf(band) | (finite & (band >= 0) & (band < .12))
    too_close_count = int(np.count_nonzero(too_close))
    # ROS 깊이 관례상 +Inf는 측정 상한 밖(열림), -Inf는 최소거리 안쪽
    # (너무 가까움)입니다. 둘을 같은 무효값으로 처리하면 벽 코앞을 빈 공간으로
    # 오인하므로, 후자는 0 m 장애물로 보수적으로 다룹니다.
    samples = np.where(valid, band, maximum_range)
    samples = np.where(too_close | np.isnan(band), 0.0, samples)
    profile = np.min(samples, axis=0)
    if len(profile) >= 5:
        padded = np.pad(profile, 2, mode="edge")
        profile = np.median(np.lib.stride_tricks.sliding_window_view(padded, 5), axis=1)
    angles = np.arctan2(float(cx) - columns, float(fx))
    near_weights = np.count_nonzero(too_close, axis=0).astype(float)
    near_angle = (float(np.average(angles, weights=near_weights))
                  if float(np.sum(near_weights)) > 0 else None)
    near_avoidance = (max(-max_steering, min(max_steering, -1.35 * near_angle))
                      if near_angle is not None else 0.0)
    usable = np.abs(angles) <= min(.70, max_steering * 1.6)
    free = (profile >= free_clearance) & usable
    runs = []
    start = None
    for index, is_free in enumerate(np.r_[free, False]):
        if is_free and start is None:
            start = index
        elif not is_free and start is not None:
            if index - start >= 3:
                run_angles = angles[start:index]
                run_ranges = profile[start:index]
                center_angle = float(np.mean(run_angles))
                width = float(abs(run_angles[0] - run_angles[-1]))
                # ``side``는 갈림길에서 비슷한 두 통로가 생길 때만 약하게
                # 작용하고, 하나뿐인 통로의 중앙을 옆으로 밀지는 않습니다.
                side_sign = 1.0 if side == "left" else -1.0
                score = (width + .12 * float(np.mean(run_ranges)) -
                         .20 * abs(center_angle) + .06 * side_sign * center_angle)
                runs.append((score, start, index, width))
            start = None
    if not runs:
        return 0.0, 0.0, {"reason": "no_free_gap", "finite_depth_samples": finite_count,
                          "too_close_depth_samples": too_close_count,
                          "near_obstacle_angle_rad": near_angle,
                          "near_avoidance_rad": near_avoidance}
    _, start, end, width = max(runs, key=lambda item: item[0])
    run_ranges = profile[start:end]
    run_angles = angles[start:end]
    # 단순히 '지나갈 수 있는' 전체 폭의 중앙을 잡으면 급커브에서 바깥 벽을
    # 너무 늦게 따라갑니다. 선택한 통로 안에서 가장 멀리 열린 연속 부분의
    # 중앙을 조준해, 굽은 코스의 소실 방향을 일찍 따라갑니다.
    far = run_ranges >= max(free_clearance, .85 * float(np.max(run_ranges)))
    far_runs = []
    far_start = None
    for index, is_far in enumerate(np.r_[far, False]):
        if is_far and far_start is None:
            far_start = index
        elif not is_far and far_start is not None:
            far_runs.append((index - far_start, far_start, index))
            far_start = None
    if far_runs:
        _, far_start, far_end = max(far_runs, key=lambda item: item[0])
        target_angle = float(np.mean(run_angles[far_start:far_end]))
    else:
        weights = np.maximum(.05, run_ranges - free_clearance)
        target_angle = float(np.average(run_angles, weights=weights))
    steering = max(-max_steering, min(max_steering, 1.40 * target_angle))
    target_index = start + int(np.argmin(np.abs(angles[start:end] - target_angle)))
    lo, hi = max(0, target_index - 2), min(len(profile), target_index + 3)
    path_clearance = float(np.min(profile[lo:hi]))
    if path_clearance < .22:
        return 0.0, steering, {"reason": "obstacle_stop", "target_angle_rad": target_angle,
                               "path_clearance_m": path_clearance,
                               "finite_depth_samples": finite_count,
                               "too_close_depth_samples": too_close_count,
                               "near_obstacle_angle_rad": near_angle,
                               "near_avoidance_rad": near_avoidance,
                               "gap_width_rad": width}
    speed = max(min_speed, max_speed / (1 + 3.0 * abs(target_angle)))
    speed = min(speed, max(min_speed, (path_clearance - .16) * .9))
    return speed, steering, {"reason": "depth_gap_following", "target_angle_rad": target_angle,
                             "path_clearance_m": path_clearance,
                             "finite_depth_samples": finite_count,
                             "too_close_depth_samples": too_close_count,
                             "near_obstacle_angle_rad": near_angle,
                             "near_avoidance_rad": near_avoidance,
                             "gap_width_rad": width}


class StartSignal:
    """먼저 빨간 렌즈를 찾고 같은 하우징의 녹색 렌즈를 연속 확인합니다.

    실험 신호등의 렌즈 지름 대비 간격을 사용합니다. 노면의 녹색을 출발
    신호로 쓰지 않으며, 세계 좌표·신호 제어 토픽·시간표를 읽지 않습니다.
    """
    def __init__(self):
        self.red_roi = None
        self.red_frames = 0
        self.green_frames = 0
        self.started = False
        self.observed = "unknown"

    def update(self, rgb):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        red = (((hsv[:, :, 0] < 12) | (hsv[:, :, 0] > 170)) &
               (hsv[:, :, 1] > 140) & (hsv[:, :, 2] > 160)).astype(np.uint8) * 255
        # 출발 상태에서 렌즈는 수평선 위에 있습니다. 하단 노면은 제외합니다.
        red[int(rgb.shape[0] * .58):] = 0
        contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if 5 <= width <= 100 and 4 <= height <= 100 and .55 < width / height < 1.8 and area > 10:
                candidates.append((area, x + width / 2, y + height / 2, float(width)))
        if candidates and not self.started:
            _, cx, cy, diameter = max(candidates)
            if self.red_roi is None or math.hypot(cx - self.red_roi[0], cy - self.red_roi[1]) < 20:
                self.red_frames += 1
            else:
                self.red_frames = 1
            self.red_roi = (cx, cy, diameter)
            self.observed = "red"
            self.green_frames = 0
        elif self.red_roi is not None and self.red_frames >= 2 and not self.started:
            cx, cy, diameter = self.red_roi
            # 빨강부터 초록까지 두 칸: 2 * 0.105 m / 0.052 m.
            gx = cx + diameter * (0.210 / 0.052)
            half = max(6.0, diameter * .85)
            x0, x1 = max(0, int(gx - half)), min(rgb.shape[1], int(gx + half + 1))
            y0, y1 = max(0, int(cy - half)), min(rgb.shape[0], int(cy + half + 1))
            roi = hsv[y0:y1, x0:x1]
            mask = ((roi[:, :, 0] > 40) & (roi[:, :, 0] < 90) &
                    (roi[:, :, 1] > 130) & (roi[:, :, 2] > 175))
            green = np.count_nonzero(mask) >= max(8, diameter * diameter * .12)
            self.green_frames = self.green_frames + 1 if green else 0
            self.observed = "green" if green else "waiting_green"
            self.started = self.green_frames >= 3
        return self.started


def marker_ids(rgb):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    if hasattr(cv2.aruco, "ArucoDetector"):
        _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    return [] if ids is None else [int(value) for value in ids.flatten()]


@dataclass
class WallEstimate:
    distance: float
    heading: float
    curvature: float
    samples: int
    residual: float


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, lidar_x=0.0):
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    return np.column_stack((ranges[valid] * np.cos(angles[valid]) + lidar_x,
                            ranges[valid] * np.sin(angles[valid]))), int(np.count_nonzero(valid))


def estimate_wall(points, side, x_min=-.22, x_max=.65, minimum_span=.20):
    """진행방향 근처의 한쪽 벽을 2차 곡선으로 맞추고 좁은 기둥은 배제합니다."""
    sign = 1 if side == "left" else -1
    x, y = points[:, 0], points[:, 1] * sign
    keep = (x > x_min) & (x < x_max) & (y > .08) & (y < 1.05)
    x, y = x[keep], y[keep]
    if len(x) < 12 or np.ptp(x) < minimum_span:
        return None
    for _ in range(3):
        coefficients = np.polynomial.polynomial.polyfit(x, y, 2)
        residuals = np.abs(y - np.polynomial.polynomial.polyval(x, coefficients))
        threshold = max(.022, 3 * float(np.median(residuals)))
        mask = residuals <= threshold
        if np.all(mask) or np.count_nonzero(mask) < 12 or np.ptp(x[mask]) < minimum_span:
            break
        x, y = x[mask], y[mask]
    c, b, a = coefficients
    error = float(np.sqrt(np.mean((y - np.polynomial.polynomial.polyval(x, coefficients)) ** 2)))
    distance = float(c / math.sqrt(1 + b * b))
    if not .08 < distance < .95 or error > .10:
        return None
    return WallEstimate(distance, sign * math.atan(float(b)),
                        sign * float(2 * a / (1 + b * b) ** 1.5), len(x), error)


def follow_command(points, side, target_distance=.425, wheelbase=.145,
                   max_steering=.43, max_speed=.35, min_speed=.14,
                   wall_x_min=-.22, wall_x_max=.65, wall_minimum_span=.20):
    estimate = estimate_wall(points, side, wall_x_min, wall_x_max, wall_minimum_span)
    if estimate is None:
        return 0.0, 0.0, {"reason": "wall_lost"}
    sign = 1 if side == "left" else -1
    denominator = max(.30, 1 + sign * target_distance * estimate.curvature)
    curvature = estimate.curvature / denominator
    curvature += 3.5 * estimate.heading + sign * 5.0 * (estimate.distance - target_distance)
    steering = max(-max_steering, min(max_steering, math.atan(wheelbase * curvature)))
    front = points[(points[:, 0] > 0) & (np.abs(points[:, 1]) < .095)]
    front_distance = float(np.min(front[:, 0])) if len(front) else math.inf
    danger = points[(points[:, 0] > -.105) & (points[:, 0] < .125) & (np.abs(points[:, 1]) < .09)]
    speed = max(min_speed, max_speed / (1 + .55 * abs(curvature)))
    speed = min(speed, max(min_speed, (front_distance - .15) * .6))
    reason = "following"
    if len(danger) or front_distance < .15:
        speed, reason = 0.0, "obstacle_stop"
    return speed, steering, {"reason": reason, "wall_distance_m": estimate.distance,
                            "wall_heading_rad": estimate.heading, "wall_curvature": estimate.curvature,
                            "fit_samples": estimate.samples, "fit_residual_m": estimate.residual,
                            "front_distance_m": front_distance if math.isfinite(front_distance) else None}
