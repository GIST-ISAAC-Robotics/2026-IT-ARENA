"""오른쪽 재접근 입력을 실행 전 보존한다. 상대 각본은 평가 전용."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
import validate_g1_traffic as evaluator


def main():
    config = json.loads((ROOT/'config/tests/official_traffic_v1.json').read_bytes())
    config['registered_at'] = '2026-11-03 v2 출발 보완/100Hz 상대 준비 전'
    config['actor']['command_hz'] = 100.
    config['scope'] = '현재 교통 제어기의 오른쪽 정지차 뒤 재접근 1회. PLAN.md의 장면 성립과 경기 목표를 별도 판정.'
    config['launch']['grid_slot'] = 0
    case = dict(name='right_reapproach', behavior_kind='slow_lead', route_kind='side_road_straight',
                actor_start_center_gap_m=0., actor_lateral_offset_m=-.145, duration_s=5.9,
                schedule=[dict(from_s=0., to_s=3.2, speed_mps=0.),
                          dict(from_s=3.2, to_s=3.4, speed_mps=.8),
                          dict(from_s=3.4, to_s=3.6, speed_mps=1.6),
                          dict(from_s=3.6, to_s=5.9, speed_mps=2.4)],
                expected='오른쪽 차 관측→뒤 사각→다시 접근. 장면 성립부터 실제 반환과 평가 자세로 판정.')
    config['cases'] = [case]
    config['acceptance']['right_reapproach'] = config['acceptance']['slow_lead'].copy()
    config['matrix'] = {'g1g2': {'cases': ['right_reapproach'], 'seeds': [0]}}
    path = HERE/'scenario.json'
    final = True
    if final and path.exists():
        if json.loads(path.read_bytes()) != config:
            raise ValueError('최종 등록에서 시나리오를 바꾸지 않는다')
    else:
        with path.open('x', encoding='utf-8', newline='\n') as stream:
            json.dump(config, stream, ensure_ascii=False, indent=2)
    config = evaluator.load_config(path)
    line, scene = evaluator.load_centerline()
    start = evaluator.actor_start_s(config, scene, 0, case)
    world = evaluator.DEMO_WORLD/'world.sdf'
    route = evaluator.check_route(config, line, case, start,
                                  evaluator.road_polygon(world), evaluator.obstacle_polygons(world))
    pose = evaluator.primary_pose_at(config, line, start, case, 0.)
    (HERE/'registered_actor.sdf').write_text(evaluator.actor_sdf(config, pose), encoding='utf-8', newline='\n')
    files = set(evaluator.source_paths(path))
    files.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    files.update(p.relative_to(ROOT).as_posix() for p in HERE.iterdir() if p.suffix in ('.py', '.sh', '.md'))
    files.update(('scripts/traffic_road_route.py', 'scripts/official_traffic_course.py',
                  'config/tests/side_departure_selector_r3_sha256.json', 'artifacts/validation/2026-11-03/right_reapproach_v1/evaluate.py'))
    registered = dict(route=route, source_sha256={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(files)})
    with (HERE/('registered_final.json' if final else 'registered.json')).open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(registered, stream, ensure_ascii=False, indent=2)
    with zipfile.ZipFile(HERE/'scene_source.zip', 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        for name in sorted(files):
            archive.write(ROOT/name, name)
    print(json.dumps(dict(route=route, sources=len(files)), ensure_ascii=False))


if __name__ == '__main__':
    main()
