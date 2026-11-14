"""원본은 보존하고 observed_straight_wall의 대응 방향만 stage에 전달한다."""
from pathlib import Path
import difflib, hashlib, json

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
REL = Path('src/arena_autonomy/arena_autonomy/bypass_entry_trial.py')
source = ROOT/REL
old = source.read_text(encoding='utf8')
before = '''    sign = 1 if side == 'left' else -1
    path, detail = local_path(points[sign*points[:, 1] > .10], side,
                              offset=config.wall_offset_m, horizon=1.8)'''
after = '''    sign = 1 if side == 'left' else -1
    # occupied_endpoints와 같은 return 순서의 방향에 동일 부분집합 마스크를 적용한다.
    # 같은 좌표의 여러 반환도 각 광선과의 짝을 유지하며 원시 점유는 바꾸지 않는다.
    bearings = np.asarray([math.atan2(ray.direction_xy[1], ray.direction_xy[0])
                           for ray in scene.rays if ray.status == 'return'])
    selected = sign*points[:, 1] > .10
    path, detail = local_path(points[selected], side,
                              offset=config.wall_offset_m, horizon=1.8,
                              bearings_rad=bearings[selected])'''
assert old.count(before) == 1
new = old.replace(before, after, 1)
original = A/'original'/REL
stage = A/'stage'/REL
original.parent.mkdir(parents=True, exist_ok=True)
stage.parent.mkdir(parents=True, exist_ok=True)
original.write_bytes(source.read_bytes())
stage.write_text(new, encoding='utf8', newline='\n')
(A/'observed_straight_wall.patch').write_text(''.join(difflib.unified_diff(
    old.splitlines(True), new.splitlines(True), fromfile='a/'+REL.as_posix(), tofile='b/'+REL.as_posix())),
    encoding='utf8', newline='\n')
manifest = dict(source=REL.as_posix(),
    original_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    stage_sha256=hashlib.sha256(stage.read_bytes()).hexdigest(),
    scope='observed_straight_wall return subset bearing only; no shared edit')
(A/'stage_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n',encoding='utf8')
print(json.dumps(manifest, indent=2))
