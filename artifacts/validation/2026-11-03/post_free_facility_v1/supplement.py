"""원본 실패를 유지한 저장 증거 보충. 차량 제어/판정선은 변경하지 않는다."""
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from g1_traffic_eval import kinematic_speed_segments, stop_pose_evidence
from surface_contact_eval import SurfaceMap, VehicleGeometry, rectangle


def main():
    old = json.loads((HERE/'run1_analysis.json').read_bytes())
    poses = [json.loads(s)[:4] for s in (HERE/'run1/ego_poses.jsonl').open() if s.strip()]
    assert all(b[0] > a[0] for a, b in zip(poses, poses[1:]))
    # 첫 속도<1 cm/s 표본 직전 0.2초에는 제동 꼬리가 남아 있을 수 있다.
    # 동일 1 cm/s·0.2초 기준을 처음 연속 만족한 시각과 그 이후 끝까지를 따로 기록한다.
    stop_request = min(r['sim_s'] for r in
        (json.loads(s) for s in (HERE/'run1/ops_log.jsonl').open(encoding='utf-8'))
        if r.get('row') == 'sample' and r.get('latch') == 'explicit_stop')
    segments = kinematic_speed_segments(poses)
    below_from, first = None, None
    for s in segments:
        if s['t0'] < stop_request:
            continue
        if s['average_speed_mps'] >= .01 or s['t1']-s['t0'] > .05:
            below_from = None
            continue
        if below_from is None:
            below_from = s['t0']
        if s['t1']-below_from >= .2:
            first = stop_pose_evidence(poses, s['t1'], .2)
            break
    tail = (stop_pose_evidence(poses, poses[-1][0], poses[-1][0]-first['required_first_s'])
            if first else None)

    # 모니터 로그는 약0.20~0.22초 간격이므로 0.2초 유효 기한을 넘는 표본이 생겼다.
    # 조향을 보간하거나 누락을 없애지 않고, 모든 조향각의 접지면을 포함하는 사각 외곽으로 평가한다.
    # 바퀴 중심은 조향으로 이동하지 않는 기존 평가 모델과 동일하다.
    surface = SurfaceMap.from_world(ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf')
    geometry = VehicleGeometry()
    radius = math.hypot(geometry.contact_patch_length_m/2, geometry.wheel_width_m/2)
    driving = [p for p in poses if old['start']['truth'][0] <= p[0] <= old['stop']['required_last_s']]
    wall_possible, over_two_possible, max_off = 0, 0, 0
    for t, x, y, yaw in driving:
        c, s = math.cos(yaw), math.sin(yaw)
        off, wall = 0, False
        for lx, ly in geometry.wheel_centers().values():
            box = rectangle(x+lx*c-ly*s, y+lx*s+ly*c, 0., 2*radius, 2*radius)
            off += not surface.road_contains(box)
            wall |= surface.walls.intersects(box)
        max_off = max(max_off, off)
        over_two_possible += off > 2
        wall_possible += wall
    result = dict(original_analysis_passed=old['passed'], original_failed=old['failed'],
        stop_latch_first_logged_s=stop_request, first_continuous_stop=first,
        delay_to_confirmed_stop_s=None if first is None else first['required_last_s']-stop_request,
        stationary_tail=tail,
        all_steering_contact_bound=dict(samples=len(driving), half_side_m=radius,
            max_possible_off_road_wheels=max_off, three_plus_possible_samples=over_two_possible,
            wall_possible_samples=wall_possible,
            scope='全조향각 접지면을 포함하는 외곽. 실제 조향각/접지력/접촉 센서 증명이 아님'),
        scope='동일 주행의 보충 분석. 원본 STOP·조향 표본 검사 실패는 변경하지 않음. STOP 직전 짧은 자동0명령과 겹쳐 순수한 버튼 제동 단독 시험은 아님')
    (HERE/'run1_supplement.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
