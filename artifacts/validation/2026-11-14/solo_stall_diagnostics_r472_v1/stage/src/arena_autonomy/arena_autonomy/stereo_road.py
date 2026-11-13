"""단독 주행 비교용 RGB 노면 경계 + 깊이 평지 확인 + 짧은 목표점 추종.

회색 노면, 녹색 잔디, 평지, 알려진 일반 도로 폭을 가정합니다. 지면 깊이만으로
도로를 식별한다고 주장하지 않으며 차량 가림/곡면/실물 조명에는 검증되지 않았습니다.
"""
import math

import cv2
import numpy as np


def road_target(rgb, depth, color_k, depth_k, camera_xyz=(.055, 0., .075),
                road_width=.45, target_x=.34):
    """공통 카메라 원점의 명목 내부 파라미터로 가까운 노면 중심을 구합니다.

    현재 Gazebo RGB/깊이 카메라는 원점/자세가 같습니다. 실물의 RGB-깊이
    외부 파라미터와 정렬이 확보되기 전에는 이 투영을 그대로 사용하면 안 됩니다.
    """
    rgb = np.asarray(rgb)
    depth = np.asarray(depth)
    values = (*color_k, *depth_k, *camera_xyz, road_width, target_x)
    if (rgb.ndim != 3 or rgb.shape[2] != 3 or depth.ndim != 2 or
            not all(math.isfinite(float(v)) for v in values)):
        raise ValueError("RGB-D 도로 투영 입력이 올바르지 않습니다.")
    fx, fy, cx, cy = map(float, color_k)
    dfx, dfy, dcx, dcy = map(float, depth_k)
    cam_x, cam_y, cam_z = map(float, camera_xyz)
    if min(fx, fy, dfx, dfy, cam_z, road_width) <= 0 or target_x <= cam_x:
        raise ValueError("RGB-D 도로 투영 범위가 올바르지 않습니다.")
    hsv = cv2.cvtColor(rgb.astype(np.uint8), cv2.COLOR_RGB2HSV)
    gray = (hsv[:, :, 1] <= 38) & (hsv[:, :, 2] >= 50) & (hsv[:, :, 2] <= 132)
    columns = np.arange(rgb.shape[1])
    dcols = np.rint(dcx + (columns - cx) * dfx / fx).astype(int)
    in_width = (dcols >= 0) & (dcols < depth.shape[1])
    samples = []
    for forward in (.28, .32, .36, .40, .45, .50, .58, .68, .80, 1.0):
        z = forward - cam_x
        row = int(round(cy + fy * cam_z / z))
        drow = int(round(dcy + (row - cy) * dfy / fy))
        if not (2 <= row < rgb.shape[0] - 2 and 0 <= drow < depth.shape[0]):
            continue
        measured = depth[drow, np.clip(dcols, 0, depth.shape[1] - 1)]
        with np.errstate(invalid="ignore"):
            height = cam_z - (drow - dcy) * measured / dfy
        ground = in_width & np.isfinite(measured) & (measured > 0) & (np.abs(height) < .018)
        line = (np.mean(gray[row-2:row+3], axis=0) >= .6) & ground
        line = cv2.morphologyEx(line.astype(np.uint8)[None, :], cv2.MORPH_CLOSE,
                               np.ones((1, 9), np.uint8))[0].astype(bool)
        starts = np.flatnonzero(line & ~np.r_[False, line[:-1]])
        ends = np.flatnonzero(line & ~np.r_[line[1:], False]) + 1
        candidates = []
        for start, end in zip(starts, ends):
            width_px = int(end - start)
            width_m = width_px * z / fx
            if width_px < 24 or width_m < .08:
                continue
            left_seen, right_seen = start > 3, end < rgb.shape[1] - 3
            if not left_seen and not right_seen:
                continue  # 양 경계가 영상 밖: 중심이라고 만들어내지 않습니다.
            left_y, right_y = cam_y + (cx - start) * z / fx, cam_y + (cx - end + 1) * z / fx
            if left_seen and right_seen:
                if not .24 <= width_m <= .85:
                    continue
                center = (left_y + right_y) / 2
                boundary = "both"
            elif left_seen:
                center, boundary = left_y - road_width / 2, "left_plus_nominal_width"
            else:
                center, boundary = right_y + road_width / 2, "right_plus_nominal_width"
            candidates.append((width_px - .2 * abs((start + end)/2-cx), center, boundary))
        if candidates:
            _, center, boundary = max(candidates)
            samples.append((forward, float(center), boundary))
    near = [s for s in samples if .27 <= s[0] <= .60]
    detail = {"road_model": "gray_rgb_depth_confirmed_flat_ground_nominal_width",
              "road_center_samples": len(samples),
              "road_near_samples": len(near),
              "road_points_xy": [[x, y] for x, y, _ in samples]}
    if len(near) < 2:
        detail["road_reason"] = "insufficient_near_ground_road"
        return None, detail
    # 먼 곳까지 곡률을 외삽하지 않고 목표에 가장 가까운 두 점을 선형 보간합니다.
    x_values, y_values = np.asarray([[x, y] for x, y, _ in near]).T
    actual_x = float(np.clip(target_x, x_values[0], x_values[-1]))
    target_y = float(np.interp(actual_x, x_values, y_values))
    detail.update(road_reason="rgbd_road_target", road_target_x_m=actual_x,
                  road_target_y_m=target_y,
                  inferred_boundary_samples=sum(b != "both" for _, _, b in near))
    return (actual_x, target_y), detail


def road_steering(target, wheelbase=.145, max_steering=.45):
    """차량 기준 목표점에 대한 기본 Pure Pursuit 조향각."""
    x, y = map(float, target)
    if not all(math.isfinite(v) for v in (x, y, wheelbase, max_steering)) or min(x, wheelbase, max_steering) <= 0:
        raise ValueError("목표점 추종 매개변수가 올바르지 않습니다.")
    return float(np.clip(math.atan2(2 * wheelbase * y, x*x+y*y), -max_steering, max_steering))
