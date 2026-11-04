"""정지 옆차 출발 한 번의 설정/소스 고정. 상대 정답은 평가에만 사용한다."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
import validate_g1_traffic as ev


def main():
    config = json.loads((HERE.parent/'right_reapproach_v1/scenario.json').read_bytes())
    config['scope'] = '좁은 정지 옆차 출발의 별도 1회. 오른쪽 사각 재접근 해결 주장이 아님.'
    config['registered_at'] = '2026-11-03 출발 후보 폐루프 전'
    case = config['cases'][0]
    case.update(name='side_departure_static', schedule=[dict(from_s=0., to_s=4.3, speed_mps=0.)],
                expected='정지옆차 약85mm 간격에서 실제RGB신호 후 제한 전진. 판정은 PLAN.md.')
    config['acceptance']['side_departure_static'] = config['acceptance'].pop('right_reapproach')
    config['matrix']['g1g2']['cases'] = ['side_departure_static']
    path = HERE/'scenario.json'
    with path.open('x', encoding='utf-8') as f:
        json.dump(config, f, ensure_ascii=False, indent=2)
    config = ev.load_config(path)
    line, scene = ev.load_centerline()
    start = ev.actor_start_s(config, scene, 0, case)
    world = ev.DEMO_WORLD/'world.sdf'
    route = ev.check_route(config, line, case, start, ev.road_polygon(world), ev.obstacle_polygons(world))
    files = set(ev.source_paths(path))
    files.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    files.update(p.relative_to(ROOT).as_posix() for p in HERE.glob('*') if p.suffix in ('.py', '.sh', '.md'))
    files.update(('scripts/official_traffic_course.py', 'scripts/traffic_road_route.py',
                  'config/tests/side_departure_r1_sha256.json'))
    registered = dict(route=route, source_sha256={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(files)})
    with (HERE/'registered.json').open('x', encoding='utf-8') as f:
        json.dump(registered, f, ensure_ascii=False, indent=2)
    with zipfile.ZipFile(HERE/'scene_source.zip', 'x', compression=zipfile.ZIP_DEFLATED) as z:
        for p in sorted(files):
            z.write(ROOT/p, p)
    print(json.dumps(dict(route=route, source_count=len(files))))


if __name__ == '__main__':
    main()
