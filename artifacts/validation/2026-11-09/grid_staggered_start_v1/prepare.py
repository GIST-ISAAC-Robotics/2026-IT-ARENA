"""11/8 공식 5대 그리드 설정에서 사례 이름·창·출발 각본만 바꾼 파생본을 만든다. 기존 파일은 읽기만 한다."""
import copy
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = ROOT/'artifacts/validation/2026-11-08/grid_pack_newcourse_v1'
CASE = 'five_vehicle_grid_staggered'
DURATION = 12.0
SPEED = .50
# 초록 적용(평가기 관측) 뒤 각본 출발 시각. 앞쪽 차량부터 출발한다.
STARTS = {'eval_grid_2': .20, 'eval_grid_1': .80, 'eval_grid_0': 1.40, 'eval_g1b_actor': 2.00}
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def schedule(start):
    return [dict(from_s=0., to_s=start, speed_mps=0.), dict(from_s=start, to_s=DURATION, speed_mps=SPEED)]


def main():
    config_path, overlay_path = BASE/'new_track_course.json', BASE/'additional_actors.json'
    config = json.loads(config_path.read_bytes())
    overlay = json.loads(overlay_path.read_bytes())
    (base_case,) = config['cases']
    assert base_case['name'] == 'five_vehicle_grid' and overlay['base_case'] == 'five_vehicle_grid'
    case = copy.deepcopy(base_case)
    case.update(name=CASE, duration_s=DURATION, schedule=schedule(STARTS['eval_g1b_actor']),
                expected='공식 grid3 자차, grid4 주 상대와 grid0/1/2 추가 상대. 초록 뒤 grid2 0.2·grid1 0.8·grid0 1.4·grid4 2.0초부터 각 0.5 m/s. 각본 시각은 평가기 전용.')
    derived = copy.deepcopy(config)
    derived.update(registered_at='2026-11-09 실행 전 준비', cases=[case],
                   scope='새 코스 공식 5칸 앞쪽부터 차례 출발 대표. 각본만 기준과 다름. 경기 거리/모든 그리드 조합/실차 검증 아님.',
                   derived_from=dict(path=config_path.relative_to(ROOT).as_posix(), sha256=sha(config_path),
                                     changed_keys=['registered_at', 'scope', 'cases', 'derived_from']))
    extras = copy.deepcopy(overlay)
    extras.update(base_case=CASE, registered_at='2026-11-09 before the single staggered grid run',
                  scope='new-course official-grid 5 cars, fixed official slots; evaluator-only staggered 0.5m/s schedules (front starts first)',
                  derived_from=dict(path=overlay_path.relative_to(ROOT).as_posix(), sha256=sha(overlay_path),
                                    changed_keys=['base_case', 'registered_at', 'scope', 'schedule', 'derived_from']))
    for actor in extras['additional_actors']:
        actor['schedule'] = schedule(STARTS[actor['model_name']])
    for name, data in (('new_track_course.json', derived), ('additional_actors.json', extras)):
        with open(HERE/name, 'x', encoding='utf-8', newline='\n') as f:
            f.write(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    print('prepared', CASE, STARTS)


if __name__ == '__main__':
    main()
