"""corner_path_budget_v1 구현 적용 스크립트(1회 실행 기록). 저장소 루트에서 실행."""
import pathlib
p=pathlib.Path('src/arena_autonomy/arena_autonomy/local_path.py')
b=p.read_bytes().decode('utf-8')
assert chr(13) not in b
t=b.replace('\r\n','\n')
old='''        path = corner_path(samples, side, offset, minimum_radius)
        if path is not None:
            return path, {'reason': 'local_path', 'wall_model': 'piecewise_corner',
                          'boundary_samples': len(wall),
                          'boundary_observed_ahead_m': float(last-anchor_s),
                          'wall_distance_m': float(abs(wall[anchor, 1])), 'path_points': path.tolist()}
'''
new='''        path = corner_path(samples, side, offset, minimum_radius)
        if path is not None:
            budget = None
            # 벽 탐색 길이는 벽 호길이다. 바깥 모서리의 오프셋 경로는 평행 이동·원호 때문에
            # 그보다 짧다. 같은 연결 구간에 실제 관측이 남을 때만 한 번 더 읽는다.
            # 관측 끝 밖 외삽·다른 구간 연결·경계 길이 기준 변경은 하지 않는다.
            ahead = _path_ahead(path)
            if ahead < horizon and arc[-1]-last > 1e-9:
                extended = min(arc[-1], last+max(0., horizon-ahead)+.05)  # .05는 재표본화 여유
                wider_s = np.linspace(first, extended, max(12, int((extended-first)/.025)+1))
                wider = corner_path(np.column_stack([np.interp(wider_s, arc, wall[:, axis]) for axis in (0, 1)]),
                                    side, offset, minimum_radius)
                # 두 번째 모서리·다른 면이 들어오면 corner_path가 거부한다. 기존 경로를 벽 적합
                # 허용량 안에서 그대로 포함하고 더 길어질 때만 바꾼다.
                if wider is not None and _path_ahead(wider) >= ahead and _covers(wider, path, .03):
                    budget = {'wall_ahead_before_m': float(last-anchor_s), 'path_ahead_before_m': ahead,
                              'path_ahead_m': _path_ahead(wider), 'extended': True}
                    path, last = wider, extended
            info = {'reason': 'local_path', 'wall_model': 'piecewise_corner',
                    'boundary_samples': len(wall),
                    'boundary_observed_ahead_m': float(last-anchor_s),
                    'wall_distance_m': float(abs(wall[anchor, 1])), 'path_points': path.tolist()}
            if budget is not None:
                info['corner_path_budget'] = budget
            return path, info
'''
assert t.count(old)==1
t=t.replace(old,new)
anchor='def local_path(points, side,'
helpers='''def _path_ahead(path):
    """원점 최근접 경로점부터 끝까지의 경로 호길이."""
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    return float(arc[-1]-arc[int(np.argmin(np.linalg.norm(path, axis=1)))])


def _covers(path, old, tolerance):
    """old의 모든 점이 path 선분에서 tolerance 안에 있는가."""
    a, d = path[:-1], np.diff(path, axis=0)
    norm2 = np.maximum(1e-12, np.sum(d*d, axis=1))
    t = np.clip(np.sum((old[:, None]-a[None])*d[None], axis=2)/norm2, 0., 1.)
    distance = np.linalg.norm(a[None]+t[:, :, None]*d[None]-old[:, None], axis=2)
    return bool(np.max(np.min(distance, axis=1)) <= tolerance)


'''
assert t.count(anchor)==1
t=t.replace(anchor,helpers+anchor)
p.write_bytes(t.encode('utf-8'))
