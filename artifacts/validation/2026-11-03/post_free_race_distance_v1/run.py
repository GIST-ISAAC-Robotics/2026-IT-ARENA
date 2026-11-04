"""새 공식 월드의 선정 교통 구성 300초·5랩. 평가 전용 등록 진입점."""
import argparse
from hashlib import sha256
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
import validate_g1_traffic as evaluator
import official_traffic_course as official
import g1_course_eval
from traffic_actors import overlay_case, actor_specs, check_additional_routes

PARENT = HERE.parents[1]/'2026-11-01/race_distance_v2'
PREVIOUS = ROOT/'artifacts/validation/2026-11-02/official_race_distance_v2'
BASE = ROOT/'artifacts/validation/2026-11-02/official_traffic_v1'
CONFIG = HERE/'course.json'
OVERLAY = HERE/'actor_overlay.json'
OUTPUT = HERE/'run1'
FIVE = ROOT/'artifacts/validation/2026-10-28/multilap_preparation_v1/course_candidate.py'
spec = importlib.util.spec_from_file_location('five_lap_geometry', FIVE)
five = importlib.util.module_from_spec(spec)
spec.loader.exec_module(five)
g1_course_eval.check_full_course = five.check_full_course


def write(path, data):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def expected():
    prev = json.loads((PREVIOUS/'registered.json').read_bytes())['source_sha256']
    for name in ('course.json','actor_overlay.json','trial_plan.json'):
        path=PREVIOUS/name
        if prev[path.relative_to(ROOT).as_posix()] != digest(path):
            raise ValueError('보존된 11/2 등록 파일 불일치: '+name)
    return tuple(json.loads((PREVIOUS/name).read_bytes()) for name in ('course.json','actor_overlay.json'))


def geometry(cfg, overlay):
    case = overlay_case(cfg['cases'][0], OVERLAY)
    assert case['additional_actors'] == overlay['additional_actors']
    line, scene = evaluator.load_centerline()
    road = evaluator.road_polygon(official.WORLD/'world.sdf')
    obstacles = evaluator.obstacle_polygons(official.WORLD/'world.sdf')
    start = evaluator.actor_start_s(cfg, scene, 0, case)
    main = five.check_full_course(cfg, line, case, start, road, obstacles)
    _, others = check_additional_routes(cfg, case, actor_specs(cfg, case), scene, 0, line, road, obstacles,
        evaluator.actor_start_s, evaluator.check_route, evaluator.actor_pose_at)
    return dict(primary=main, additional=others,
                scope='평가 상대의 배치를 새 공식 모델과 대조. 자차 주행 결과는 아니다.')


def arguments():
    return ['scripts/validate_g1_traffic.py', '--mode', 'g1g2', '--handoff-reserve', 'paced',
            '--handoff-yield-progress', '--handoff-rear-sweep', '--handoff-observed-motion',
            '--side-departure-trial', '--current-lane-preference-trial', '--official-signal-course', '--record-follow-decisions', '--config', str(CONFIG),
            '--actor-overlay', str(OVERLAY), '--case', 'stop_restart_full_course', '--seed', '0',
            '--output', str(OUTPUT)]


def facility(args, cfg, case):
    wanted, overlay = expected()
    if (cfg != wanted or json.loads(OVERLAY.read_bytes()) != overlay
            or args.config.resolve() != CONFIG or args.actor_overlay.resolve() != OVERLAY
            or args.output.resolve() != OUTPUT or args.mode != 'g1g2' or args.seed != 0
            or args.handoff_reserve != 'paced' or not args.handoff_yield_progress
            or not args.handoff_rear_sweep or not args.handoff_observed_motion
            or not args.side_departure_trial or not args.current_lane_preference_trial
            or not args.record_follow_decisions or args.case != 'stop_restart_full_course'
            or case != overlay_case(cfg['cases'][0], OVERLAY)):
        raise ValueError('등록한 최신 공식5랩 구성과 다름')
    files = set(evaluator.source_paths(CONFIG, actor_overlay=OVERLAY))
    files.update(p.relative_to(ROOT).as_posix() for p in
                 (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    extra = [*official.ENTRY_FILES, Path(__file__).resolve(), FIVE, HERE/'trial_plan.json',
             HERE/'geometry.json', PARENT/'course.json', PARENT/'actor_overlay.json',
             ROOT/'config/tests/entry_obstruction_r1_sha256.json']
    extra.extend([PREVIOUS/'registered.json',PREVIOUS/'course.json',PREVIOUS/'actor_overlay.json',
                  PREVIOUS/'trial_plan.json', HERE/'review.py',HERE/'dense_progress.py',HERE/'scene.sh',HERE/'PLAN.md',
                  ROOT/'config/tests/current_lane_preference_r1_sha256.json'])
    for path in (HERE.parent/'right_reapproach_v3/run1_goal.json', HERE.parent/'merge_lane_preference_v1/review.json'):
        if json.loads(path.read_bytes())['passed'] is not True:
            raise ValueError('현재 재접근/본선 복귀 선행 시험 미통과: '+str(path))
        extra.append(path)
    for name in ('p1', 'r1', 'c1'):
        path = BASE/f'{name}_goal.json'
        if json.loads(path.read_bytes())['passed'] is not True:
            raise ValueError('새 월드 대표 교통 선행 실패')
        extra.append(path)
    files.update(p.relative_to(ROOT).as_posix() for p in extra)
    return {name: digest(ROOT/name) for name in sorted(files)}


def registration():
    cfg, overlay = expected()
    if json.loads(CONFIG.read_bytes()) != cfg:
        raise ValueError('등록 각본 변경')
    request = SimpleNamespace(config=CONFIG, actor_overlay=OVERLAY, output=OUTPUT, mode='g1g2', seed=0,
        handoff_reserve='paced', handoff_yield_progress=True, handoff_rear_sweep=True,
        handoff_observed_motion=True, side_departure_trial=True, current_lane_preference_trial=True, record_follow_decisions=True, case='stop_restart_full_course')
    return dict(args=arguments(), source_sha256=facility(request, cfg, overlay_case(cfg['cases'][0], OVERLAY)),
                scope='공식 신호 실제 RGB 출발, 새 월드 경기 거리. 기본 활성화·물성 변경 없음.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    if OUTPUT.exists():
        raise FileExistsError('기존 실행 덮어쓰기 금지')
    if args.prepare_only:
        cfg, overlay = expected()
        for dest,name in ((CONFIG,'course.json'),(OVERLAY,'actor_overlay.json')):
            with dest.open('xb') as stream: stream.write((PREVIOUS/name).read_bytes())
        write(HERE/'geometry.json', geometry(cfg, overlay))
        write(HERE/'registered.json', registration())
        print('geometry and sources registered; no simulation started')
    else:
        if registration() != json.loads((HERE/'registered.json').read_bytes()):
            raise ValueError('등록 후 소스 변경')
        official.inputs = facility
        sys.argv = arguments()
        raise SystemExit(evaluator.main())
