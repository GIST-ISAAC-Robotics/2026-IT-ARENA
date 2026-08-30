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


def estimate_wall(points, side):
    """진행방향 근처의 한쪽 벽을 2차 곡선으로 맞추고 좁은 기둥은 배제합니다."""
    sign = 1 if side == "left" else -1
    x, y = points[:, 0], points[:, 1] * sign
    keep = (x > -.22) & (x < .65) & (y > .08) & (y < 1.05)
    x, y = x[keep], y[keep]
    if len(x) < 12 or np.ptp(x) < .20:
        return None
    for _ in range(3):
        coefficients = np.polynomial.polynomial.polyfit(x, y, 2)
        residuals = np.abs(y - np.polynomial.polynomial.polyval(x, coefficients))
        threshold = max(.022, 3 * float(np.median(residuals)))
        mask = residuals <= threshold
        if np.all(mask) or np.count_nonzero(mask) < 12 or np.ptp(x[mask]) < .20:
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
                   max_steering=.43, max_speed=.35, min_speed=.14):
    estimate = estimate_wall(points, side)
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
