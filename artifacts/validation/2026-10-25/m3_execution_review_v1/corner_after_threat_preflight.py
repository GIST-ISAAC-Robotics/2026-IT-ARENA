"""run32 새 config/overlay의 정적 검사. 주행·판정선 변경 없음. 기존 run27 조건과 초기 배치를 대조한다."""
import hashlib
import json
from pathlib import Path
import sys

import validate_g1_traffic as runner
from traffic_actors import overlay_case, actor_specs, check_additional_routes

CONFIG = Path('config/tests/g1_corner_after_threat_v1.json')
OVERLAY = Path('config/tests/e3_stop_restart_corner_v1.json')
RUN27 = Path('artifacts/validation/2026-10-25/m3_execution_review_v1/run27_small_center_stop_restart/run/report.json')


def main():
    out = Path(sys.argv[1])
    if out.exists():
        raise FileExistsError(out)
    cfg = runner.load_config(CONFIG)
    stop = runner.load_config()                       # g1_traffic_v1 (run27)
    ext = runner.load_config(Path(cfg['parent_config']['path']))
    assert hashlib.sha256(Path(cfg['parent_config']['path']).read_bytes()).hexdigest() == cfg['parent_config']['sha256']
    differences = {k: [stop.get(k), cfg.get(k)] for k in ('launch', 'actor', 'seeds', 'vehicle_and_sensor_unchanged')
                   if stop.get(k) != cfg.get(k)}
    case = overlay_case(runner.case_named(cfg, 'stop_restart_first_curve'), OVERLAY)
    specs = actor_specs(cfg, case)
    line, scene = runner.load_centerline()
    road = runner.road_polygon(runner.DEMO_WORLD/'world.sdf')
    obstacles = runner.obstacle_polygons(runner.DEMO_WORLD/'world.sdf')
    s0 = runner.actor_start_s(cfg, scene, 0, case)
    lead = runner.check_route(cfg, line, case, s0, road, obstacles)
    starts, routes = check_additional_routes(cfg, case, specs, scene, 0, line, road, obstacles,
        runner.actor_start_s, runner.check_route, runner.actor_pose_at)
    run27 = json.loads(RUN27.read_text(encoding='utf-8'))
    old_side = run27['actor_overlay']['routes']['actors']['eval_e3_side']
    new_side = routes['actors']['eval_e3_side']
    same_start = (abs(run27['route_check']['actor_start_s_m']-lead['actor_start_s_m']) < 1e-9
                  and abs(old_side['actor_start_s_m']-new_side['actor_start_s_m']) < 1e-9
                  and abs(old_side['actor_end_s_m']-new_side['actor_end_s_m']) < 1e-9
                  and old_side['lateral_offset_m'] == new_side['lateral_offset_m'])
    result = {'scope': 'static geometry/config only; not a drive or acceptance change',
              'config_sha256': hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
              'overlay_sha256': hashlib.sha256(OVERLAY.read_bytes()).hexdigest(),
              'launch_actor_seed_vehicle_differences_vs_run27_config': differences,
              'extension_parent_cases_untouched': [c['name'] for c in ext['cases']],
              'lead_route': lead, 'side_routes': routes, 'same_initial_placement_as_run27': same_start,
              'side_actor_schedule': case['additional_actors'][0]['schedule']}
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2, default=str)+'\n', encoding='utf8')
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
    assert same_start and not differences


if __name__ == '__main__':
    main()
