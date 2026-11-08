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
            # 표본 평균의 우연한 치우침 때문에 한쪽 잔차만 커지지 않게,
            # 같은 방향의 원시 반환 띠 가운데로 선을 옮긴다. 최대 잔차 3 cm는 유지한다.
            residual -= (residual.max()+residual.min())/2
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
