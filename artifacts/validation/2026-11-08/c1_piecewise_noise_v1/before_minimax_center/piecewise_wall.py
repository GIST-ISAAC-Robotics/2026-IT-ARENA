"""현재 연결 벽의 두 직선과 접선 원호. 경로 표현이며 자유 공간 증거가 아니다."""
import math

import numpy as np


def _vertices(points, tolerance=.025):
    """관측 순서를 유지한 RDP 꼭짓점. 벽 사이 결측을 연결하는 데 쓰지 않는다."""
    if len(points) < 3:
        return [0, len(points)-1]
    chord = points[-1]-points[0]
    length = np.linalg.norm(chord)
    if length < 1e-9:
        return list(range(len(points)))
    distance = np.abs(np.cross(chord, points-points[0]))/length
    index = int(np.argmax(distance))
    if distance[index] <= tolerance:
        return [0, len(points)-1]
    return (_vertices(points[:index+1], tolerance)[:-1] +
            [index+i for i in _vertices(points[index:], tolerance)])


def corner_path(samples, side, offset, minimum_radius):
    """한 모서리만 표현한다. 곡선/짧은 면/여러 모서리는 기존 적합으로 돌린다.

    양쪽 벽의 평행선을 같은 반경으로 연결하므로 같은 직각 회랑이면 동일한
    중심 경로를 얻는다. 반경은 알려진 벽 오프셋이며 조향 가능 최소 반경보다
    작게 만들지 않는다. 점유·관측·경로 확인은 호출자가 그대로 수행한다.
    """
    samples = np.asarray(samples, dtype=float)
    if len(samples) < 12 or offset < minimum_radius or not np.all(np.isfinite(samples)):
        return None
    vertices = _vertices(samples)
    if len(vertices) != 3:
        vertices = _noisy_corner_vertices(samples)
        if vertices is None:
            return None
    split = vertices[1]
    lines = []
    for pts in (samples[:split+1], samples[split:]):
        if len(pts) < 5 or np.linalg.norm(pts[-1]-pts[0]) < .15:
            return None
        center = pts.mean(axis=0)
        tangent = np.linalg.svd(pts-center, full_matrices=False)[2][0]
        if tangent @ (pts[-1]-pts[0]) < 0:
            tangent = -tangent
        normal = np.array([tangent[1], -tangent[0]])
        if np.max(np.abs((pts-center) @ normal)) > .03:
            return None
        lines.append((center, tangent, normal))
    (p, t0, n0), (q, t1, n1) = lines
    angle = math.atan2(float(np.cross(t0, t1)), float(t0 @ t1))
    if not math.radians(40) <= abs(angle) <= math.radians(135):
        return None
    sign = 1 if side == 'left' else -1
    p, q = p+sign*offset*n0, q+sign*offset*n1
    a, _ = np.linalg.solve(np.column_stack([t0, -t1]), q-p)
    corner = p+a*t0
    radius = offset
    tangent_length = radius*math.tan(abs(angle)/2)
    first, last = corner-tangent_length*t0, corner+tangent_length*t1
    turn = 1 if angle > 0 else -1
    center = first+turn*radius*np.array([-t0[1], t0[0]])
    start_angle = math.atan2(*(first-center)[::-1])
    theta = start_angle+np.linspace(0., angle, max(12, math.ceil(radius*abs(angle)/.02)+1))
    arc = center+radius*np.column_stack([np.cos(theta), np.sin(theta)])
    # 창 가장자리가 원호 중간을 자르면 그 지점까지만 남긴다. 뒤로 향하는
    # 직선이나 관측 끝에서 시작한 두 번째 가지를 붙이지 않는다.
    start = p+t0*float((samples[0]+sign*offset*n0-p) @ t0)
    end = q+t1*float((samples[-1]+sign*offset*n1-q) @ t1)
    before, after = float((first-start) @ t0), float((end-last) @ t1)
    lo = 0 if before >= 0 else int(np.argmin(np.linalg.norm(arc-start, axis=1)))
    hi = len(arc)-1 if after >= 0 else int(np.argmin(np.linalg.norm(arc-end, axis=1)))
    if hi-lo < 3:
        return None
    pieces = []
    if before > 1e-6:
        pieces.append(np.linspace(start, first, max(2, math.ceil(before/.02)+1))[:-1])
    pieces.append(arc[lo:hi+1])
    if after > 1e-6:
        pieces.append(np.linspace(last, end, max(2, math.ceil(after/.02)+1))[1:])
    points = np.vstack(pieces)
    length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    return np.column_stack([np.interp(np.linspace(0., length[-1], 36), length, points[:, i])
                            for i in (0, 1)])


def _noisy_corner_vertices(points):
    """잡음이 있는 연결 구간에서 원 점의 두 직선 잔차로 모서리 위치를 고른다.

    RDP가 실패한 경우의 제안만 보완한다. 원시 점을 평활화/삭제하거나 잔차·각도·
    최소 면 길이 기준을 넓히지 않는다. 새 자유 공간을 만들지 않는다.
    """
    chord = points[2:]-points[:-2]
    middle = points[1:-1]-points[:-2]
    roughness = np.median(np.abs(chord[:, 0]*middle[:, 1]-chord[:, 1]*middle[:, 0]) /
                          np.maximum(1e-9, np.linalg.norm(chord, axis=1)))
    if roughness <= .003:
        return None
    best = None
    for split in range(4, len(points)-4):
        lines = []
        loss = 0.
        for pts in (points[:split+1], points[split:]):
            if np.linalg.norm(pts[-1]-pts[0]) < .15:
                break
            center = pts.mean(axis=0)
            tangent = np.linalg.svd(pts-center, full_matrices=False)[2][0]
            if tangent @ (pts[-1]-pts[0]) < 0:
                tangent = -tangent
            residual = (pts-center) @ np.array([tangent[1], -tangent[0]])
            if np.max(np.abs(residual)) > .03:
                break
            lines.append(tangent)
            loss += float(residual @ residual)
        if len(lines) != 2:
            continue
        angle = math.atan2(float(np.cross(lines[0], lines[1])), float(lines[0] @ lines[1]))
        if math.radians(40) <= abs(angle) <= math.radians(135):
            candidate = (loss, split)
            if best is None or candidate < best:
                best = candidate
    return None if best is None else [0, best[1], len(points)-1]
