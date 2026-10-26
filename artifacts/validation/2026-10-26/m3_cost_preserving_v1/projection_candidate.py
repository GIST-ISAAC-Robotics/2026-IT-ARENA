"""경로 투영 산술 후보. 원래 모든 선분·sqrt·argmin 순서와 끝 접선 연장을 유지한다."""
import numpy as np


def project_xy(frame, points):
    q = np.asarray(points, dtype=float).reshape(-1, 2)
    a = frame.p[:-1]
    n = len(frame.length)
    lo = np.where(np.arange(n) == 0, -np.inf, 0.)
    hi = np.where(np.arange(n) == n-1, np.inf, frame.length)
    s, lateral = np.empty(len(q)), np.empty(len(q))
    tx, ty = frame.tangent[:, 0], frame.tangent[:, 1]
    # NxMx2 배열의 반복 생성/축 축약만 줄인다. 선분 후보를 근사 제거하지 않는다.
    for k in range(0, len(q), 128):
        p = q[k:k+128]
        rx = p[:, 0, None]-a[None, :, 0]
        ry = p[:, 1, None]-a[None, :, 1]
        t = rx*tx[None]+ry*ty[None]
        tc = np.clip(t, lo[None], hi[None])
        fx = a[None, :, 0]+tc*tx[None]
        fy = a[None, :, 1]+tc*ty[None]
        dx, dy = p[:, 0, None]-fx, p[:, 1, None]-fy
        # sqrt를 제거하면 반올림으로 같아진 거리의 argmin 동률 처리까지 바뀔 수 있다.
        distance = np.sqrt(dx*dx+dy*dy)
        nearest = np.argmin(distance, axis=1)
        row = np.arange(len(p))
        s[k:k+len(p)] = frame.s[nearest]+tc[row, nearest]
        lateral[k:k+len(p)] = tx[nearest]*dy[row, nearest]-ty[nearest]*dx[row, nearest]
    return s, lateral
