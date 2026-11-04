"""등록 교통 시험 장면을 새 월드의 정적 경로 검사에 넣어 공식 tl_post와의 겹침을 확인한다. 주행 없음."""
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/'scripts')]
import validate_g1_traffic as harness  # noqa: E402

WORLD = harness.DEMO_WORLD/'world.sdf'
SIDE = ROOT/'config/tests/e3_two_actors_harness_v1.json'


def main():
    line, scene = harness.load_centerline()
    road, walls = harness.road_polygon(WORLD), harness.obstacle_polygons(WORLD)
    rows = []
    for path in sorted((ROOT/'config/tests').glob('*.json')):
        if path.name.endswith('_sha256.json'):
            continue
        try:
            config = harness.load_config(path)
        except Exception as error:
            rows.append(dict(config=path.name, status='not_a_traffic_config', detail=type(error).__name__))
            continue
        for case in config['cases']:
            try:
                start = harness.actor_start_s(config, scene, 0, case)
                harness.check_route(config, line, case, start, road, walls)
                rows.append(dict(config=path.name, case=case['name'], status='ok'))
            except Exception as error:
                rows.append(dict(config=path.name, case=case.get('name'), status='rejected', detail=str(error)))
    result = dict(scope='static route check on the v2026.11.01 runtime world; not a driving result',
                  rejected=[row for row in rows if row['status'] == 'rejected'],
                  ok_cases=sum(row['status'] == 'ok' for row in rows),
                  skipped_configs=sorted(row['config'] for row in rows if row['status'] == 'not_a_traffic_config'))
    (HERE/'traffic_scene_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps({k: (len(v) if isinstance(v, list) else v) for k, v in result.items()}, ensure_ascii=False))


if __name__ == '__main__':
    main()
