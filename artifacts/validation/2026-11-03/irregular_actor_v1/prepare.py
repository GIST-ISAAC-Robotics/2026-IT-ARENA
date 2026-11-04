"""등록 입력의 경로와 관측/충돌 형상을 보존. 제어 입력에는 전달하지 않는다."""
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
import validate_g1_traffic as evaluator

CFG = ROOT/'config/tests/irregular_actor_v1.json'


def main():
    config = evaluator.load_config(CFG)
    case = evaluator.case_named(config, 'persistent_block')
    line, scene = evaluator.load_centerline()
    start = evaluator.actor_start_s(config, scene, 0, case)
    world = evaluator.DEMO_WORLD/'world.sdf'
    route = evaluator.check_route(config, line, case, start, evaluator.road_polygon(world), evaluator.obstacle_polygons(world))
    pose = evaluator.primary_pose_at(config, line, start, case, 0.)
    (HERE/'registered_actor.sdf').write_text(evaluator.actor_sdf(config, pose), encoding='utf-8', newline='\n')
    files = set(evaluator.source_paths(CFG))
    files.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    files.update(p.relative_to(ROOT).as_posix() for p in HERE.glob('*.py'))
    files.update(p.relative_to(ROOT).as_posix() for p in HERE.glob('*.sh'))
    files.update(('config/tests/actor_outline_r1_sha256.json', 'scripts/official_traffic_course.py',
                  'scripts/official_2x5_facilities.py', 'scripts/prelap_signal_eval.py'))
    registered = dict(config=CFG.relative_to(ROOT).as_posix(), actor_pose=pose, route=route,
                      source_sha256={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(files)})
    with (HERE/'registered.json').open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(registered, stream, ensure_ascii=False, indent=2)
    print(json.dumps(dict(route=route, source_files=len(files)), ensure_ascii=False))


if __name__ == '__main__':
    main()
