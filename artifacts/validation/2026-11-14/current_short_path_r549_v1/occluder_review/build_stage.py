"""공통 원본을 보존하고 실제 광선 방위 정렬 후보만 별도 생성한다."""
from pathlib import Path
import difflib, hashlib, json
A = Path(__file__).resolve().parent
ROOT = A.parents[4]
REL = Path('src/arena_autonomy/arena_autonomy')

def replace_once(text, old, new):
    assert text.count(old) == 1, (old, text.count(old))
    return text.replace(old, new, 1)

manifest = {}
for name in ('local_path.py', 'current_wall_reacquire.py', 'local_follow.py'):
    src = ROOT/REL/name
    old = src.read_text(encoding='utf8')
    new = old
    if name == 'local_path.py':
        new = replace_once(new, 'narrow_occluder_bridge=False, minimum_ahead_m=.28):',
            'narrow_occluder_bridge=False, minimum_ahead_m=.28, bearings_rad=None):')
        new = replace_once(new, '    rays는 points와 같은 순서의 원시 광선 번호다. 주지 않으면 거리 간격만으로 연결한다.',
            '    rays는 points와 같은 순서의 원시 광선 번호다. 주지 않으면 거리 간격만으로 연결한다.\n'
            '    bearings_rad는 같은 실제 반환 광선의 보정 후 방향이다. 주어지면 이 각도로 정렬한다.\n'
            '    센서 원점과 차체 원점의 차이로 가까운 벽과 먼 배경이 교차 정렬되는 것을 막는다.')
        new = replace_once(new, '    finite = np.all(np.isfinite(points), axis=1)',
            '    bearings = None if bearings_rad is None else np.asarray(bearings_rad, dtype=float)\n'
            '    if bearings is not None and (bearings.shape != (len(points),) or not np.all(np.isfinite(bearings))):\n'
            '        return None, {"reason": "wall_bearings_invalid"}\n'
            '    finite = np.all(np.isfinite(points), axis=1)')
        new = replace_once(new, '    # 보정이 끝난 현재 좌표의 방위 순서. 왼쪽은 감소, 오른쪽은 증가 방향이 전진이다.\n    angles = np.arctan2(points[:, 1], points[:, 0])',
            '    # 왼쪽은 감소, 오른쪽은 증가 방향이 전진이다. 광선이 없으면 기존 원점 방위를 유지한다.\n'
            '    angles = np.arctan2(points[:, 1], points[:, 0]) if bearings is None else bearings[finite]')
    elif name == 'current_wall_reacquire.py':
        new = replace_once(new, '    try:\n        # 저속 재획득은 차체 길이만큼 관측한 선호 벽을 사용할 수 있다.\n        # 경로 자체의 전방0.25m·형상·제동/원시점유 검사는 뒤에서 그대로 적용한다.\n        # 짧다는 이유로 반대 벽으로 전환해 갈림길 방향을 바꾸지 않는다.\n        minimum_ahead = min(.28, cfg.length_m) if cap_mps <= .10 else .28',
            '    # occupied_endpoints와 같은 return 순서이며 각 광선의 보정된 실제 방향을 쓴다.\n'
            '    bearings = [math.atan2(ray.direction_xy[1], ray.direction_xy[0])\n'
            '                for ray in scene.rays if ray.status == "return"]\n'
            '    try:\n        minimum_ahead = .28')
        new = replace_once(new, '            minimum_ahead_m=minimum_ahead)',
            '            minimum_ahead_m=minimum_ahead, bearings_rad=bearings)')
    else:
        new = replace_once(new, '        bridge = {\'narrow_occluder_bridge\': True} if cfg.narrow_occluder_bridge else {}\n        def observed_path(cloud, wall_side):',
            '        bearing_by_point = {tuple(ray.hit_xy_m): math.atan2(ray.direction_xy[1], ray.direction_xy[0])\n'
            '                            for ray in scene.rays if ray.status == "return"}\n'
            '        bridge = {\'narrow_occluder_bridge\': True} if cfg.narrow_occluder_bridge else {}\n'
            '        def observed_path(cloud, wall_side):\n'
            '            # 경계 후보 부분집합도 원시 반환점이다. 경로용 정렬만 바꾸며 점유점은 그대로 둔다.\n'
            '            observed = dict(bridge, bearings_rad=[bearing_by_point[tuple(p)] for p in cloud.tolist()])')
        start = new.index('        def observed_path(cloud, wall_side):')
        stop = new.index('        carried = None', start)
        new = new[:start]+new[start:stop].replace('**bridge', '**observed')+new[stop:]
    orig = A/'original'/REL/name
    out = A/'stage'/REL/name
    orig.parent.mkdir(parents=True, exist_ok=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    orig.write_bytes(src.read_bytes())
    out.write_text(new, encoding='utf8', newline='\n')
    patch = ''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
        fromfile='a/'+(REL/name).as_posix(),tofile='b/'+(REL/name).as_posix()))
    (A/(name+'.diff')).write_text(patch,encoding='utf8',newline='\n')
    manifest[name] = dict(original_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
        stage_sha256=hashlib.sha256(out.read_bytes()).hexdigest())
(A/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
print(json.dumps(manifest,indent=2))
