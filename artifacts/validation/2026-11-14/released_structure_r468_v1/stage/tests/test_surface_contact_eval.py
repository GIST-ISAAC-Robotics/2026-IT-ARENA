"""바퀴별 노면 평가기의 반례. 합성 지면과 실제 공식 월드만 쓰며 제어기 경로는 없다."""
import math
from pathlib import Path

import pytest
import yaml
from shapely.geometry import Point, box
from shapely.ops import unary_union

import surface_contact_eval as sc
from audit_local_pursuit_road import road_polygon

REPO = Path(__file__).resolve().parents[1]
WORLD = REPO/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
G = sc.VehicleGeometry()


def straight_map(other_gap=False):
    """직선 도로 ±0.225, 잔디 0.20, 벽 0.05. other_gap이면 x∈[2,2.5] 왼쪽 잔디를 비운다."""
    road = box(-5, -.225, 5, .225)
    grass = unary_union([box(-5, .225, 5, .425), box(-5, -.425, 5, -.225)])
    if other_gap:
        grass = grass.difference(box(2, .225, 2.5, .425))
    walls = unary_union([box(-5, .425, 5, .475), box(-5, -.475, 5, -.425)])
    return sc.SurfaceMap(road, grass, walls)


def counts(surface, x, y, yaw, steer=0.):
    return sc.evaluate_pose(surface, x, y, yaw, steer)['off_counts']


def test_defaults_match_vehicle_yaml_and_state_status():
    data = yaml.safe_load((REPO/'src/arena_description/config/vehicle.yaml').read_text(encoding='utf-8'))['vehicle']
    drive = data['drivetrain']
    assert (G.body_length_m, G.body_width_m) == (data['footprint']['length_m'], data['footprint']['width_m'])
    assert (G.wheelbase_m, G.track_width_m, G.wheel_radius_m, G.wheel_width_m, G.max_wheel_steering_rad) == (
        drive['wheelbase_m'], drive['track_width_m'], drive['wheel_radius_m'], drive['wheel_width_m'],
        drive['max_steering_angle_rad'])
    assert set(sc.VALUE_STATUS) == set(sc.asdict(G))
    with pytest.raises(ValueError):
        sc.VehicleGeometry(track_width_m=.15)   # 직진 바퀴가 15 cm 외형을 넘으면 거절


@pytest.mark.parametrize('y, yaw, expected', [
    (0., 0., 0),        # 중앙
    (.16, .3, 1),       # 기울어진 자세에서 앞왼쪽만
    (.22, 0., 2),       # 같은 쪽 앞뒤 두 바퀴
    (.28, .5, 3),       # 큰 yaw로 오른쪽 앞바퀴까지
    (.32, 0., 4),       # 네 바퀴 모두 잔디
])
def test_zero_to_four_wheels_on_straight(y, yaw, expected):
    surface = straight_map()
    result = sc.evaluate_pose(surface, 0., y, yaw)
    assert result['off_counts'] == {'any': expected, 'center': expected, 'full': expected}
    assert result['grass_wheels']['any'] == expected and result['other_wheels']['any'] == 0
    assert result['wall_wheels'] == 0


def test_one_and_three_wheel_identity():
    surface = straight_map()
    one = sc.evaluate_pose(surface, 0., .16, .3)['wheels']
    assert [n for n, w in one.items() if w['off']['any']] == ['front_left']
    three = sc.evaluate_pose(surface, 0., .28, .5)['wheels']
    assert sorted(n for n, w in three.items() if w['off']['any']) == ['front_left', 'front_right', 'rear_left']


def test_two_wheels_same_side_band_is_wide_but_finite():
    """같은 쪽 두 바퀴 상태가 유지되는 중심 횡위치 범위와 그 밖의 전이(any 해석, yaw 0)."""
    surface = straight_map()
    patch_half = G.wheel_width_m/2
    lo = .225-G.track_width_m/2+patch_half          # 바깥 바퀴 접지면 전체가 나가는 시작
    hi = .225+G.track_width_m/2-patch_half          # 안쪽 바퀴가 걸치기 시작
    assert lo == pytest.approx(.1635) and hi == pytest.approx(.2865)
    assert counts(surface, 0, lo+.001, 0)['any'] == 2 and counts(surface, 0, hi-.001, 0)['any'] == 2
    assert counts(surface, 0, hi+.001, 0)['any'] == 4          # 안쪽 바퀴 일부 → any 4
    assert counts(surface, 0, hi+.001, 0)['full'] == 2          # 전체 이탈 해석은 아직 2
    # 3 cm 위치 오차: 띠 가운데(0.225)에서는 판정이 바뀌지 않지만 끝 근처에서는 바뀐다.
    for error in (-.03, .03):
        assert counts(surface, 0, .225+error, 0)['any'] == 2
    assert counts(surface, 0, hi-.01+.03, 0)['any'] == 4


def test_boundary_straddle_interpretations_differ():
    surface = straight_map()
    on_edge = .225-G.track_width_m/2      # 왼쪽 바퀴 접지 중심이 정확히 도로 끝
    assert counts(surface, 0, on_edge, 0) == {'any': 2, 'center': 0, 'full': 0}
    assert counts(surface, 0, on_edge+.002, 0) == {'any': 2, 'center': 2, 'full': 0}
    assert counts(surface, 0, on_edge+G.wheel_width_m/2+.0005, 0) == {'any': 2, 'center': 2, 'full': 2}


def test_other_ground_is_not_grass_and_wall_is_separate():
    surface = straight_map(other_gap=True)
    result = sc.evaluate_pose(surface, 2.25, .22, 0.)
    left = result['wheels']['front_left']
    assert left['off']['any'] and left['off_surface'] == 'other' and left['fractions']['grass'] == 0
    assert result['grass_wheels']['any'] == 0 and result['other_wheels']['any'] == 2
    assert surface.point_class(2.25, .3) == 'other' and surface.point_class(2.25, .45) == 'wall'
    # 잔디 경계에 걸친 바퀴는 mixed로 남긴다(잔디로 일괄 분류하지 않음).
    edge = sc.evaluate_pose(surface, 2.5-.0675+.001, .22, 0.)['wheels']['front_left']
    assert edge['off_surface'] == 'mixed' and edge['fractions']['other'] > 0 and edge['fractions']['grass'] > 0
    hit = sc.evaluate_pose(surface, 0., .42-.0675, 0.)['wheels']['front_left']
    assert hit['fractions']['wall'] > 0 and hit['off']['any']


def test_corner_uses_actual_curved_surface():
    """반경 0.70–1.15 m 도로(폭 0.45)의 원호. 바깥 띠는 잔디, 안쪽 원판은 잔디."""
    center = Point(0, 0)
    road = center.buffer(1.15, 256).difference(center.buffer(.70, 256))
    grass = center.buffer(1.35, 256).difference(center.buffer(1.15, 256)).union(center.buffer(.70, 256))
    walls = center.buffer(1.40, 256).difference(center.buffer(1.35, 256))
    surface = sc.SurfaceMap(road, grass, walls)
    def at(radius, angle=.3):
        return counts(surface, radius*math.cos(angle), radius*math.sin(angle), angle+math.pi/2)
    assert at(.925)['any'] == 0
    # 바깥쪽: 두 바깥 바퀴가 같은 반경이 아니다(축이 원호의 현). 직선 띠와 다른 경계를 확인한다.
    outer = [r/1000 for r in range(1050, 1250)]
    states = [at(r)['any'] for r in outer]
    assert 2 in states and 4 in states and states == sorted(states)
    first_two = outer[states.index(2)]
    # 축이 원호의 현이므로 앞뒤 바깥 바퀴의 반경은 sqrt((r+T/2)^2+(L/2)^2)로 직선보다 먼저 나간다.
    chord = math.sqrt((1.15-G.wheel_width_m/2)**2-(G.wheelbase_m/2)**2)-G.track_width_m/2
    assert first_two == pytest.approx(chord, abs=.0015)
    assert first_two < 1.15-G.track_width_m/2+G.wheel_width_m/2
    assert at(.70+.0675-.002)['any'] == 2                         # 안쪽 잔디 두 바퀴


def test_crossing_event_and_durations():
    """20° 사선 이탈 후 복귀. 원시 에지 1, 바퀴 진입 순서, 지속 시간."""
    surface = straight_map()
    times, evaluations = [], []
    yaw = math.radians(20)
    for i in range(0, 1501):
        t = i*.002
        y = .10+.22*math.sin(math.pi*t/3.)          # 0→3 s 왕복, 최대 0.32 m
        times.append(t)
        evaluations.append(sc.evaluate_pose(surface, t*.3, y, yaw if t < 1.5 else -yaw))
    summary = sc.analyze_series(times, evaluations, merge_gap_s=.5)
    anyk = summary['interpretations']['any']
    assert anyk['max_off_wheels'] >= 3
    assert anyk['penalty_state_raw_rising_edges'] == anyk['penalty_events_merged'] == 1
    event = anyk['penalty_events'][0]
    assert 0 < event['duration_s'] < 3 and event['off_surfaces'] == ['grass']
    assert anyk['grass_contact_time_s'] >= event['duration_s']
    assert summary['body_outside_samples_over_1e-6_m2'] > 0


def test_flicker_raw_edges_versus_merged_events():
    surface = straight_map()
    times, evaluations = [], []
    for i in range(300):
        t = i*.01
        # 0.0–0.4 s 동안 2↔4바퀴를 10 ms마다 오가고, 이후 2바퀴로 안정.
        y = (.29 if i % 2 else .22) if t < .4 else .22
        times.append(t)
        evaluations.append(sc.evaluate_pose(surface, 0, y, 0.))
    merged = sc.analyze_series(times, evaluations, merge_gap_s=.5)['interpretations']['any']
    raw = sc.analyze_series(times, evaluations, merge_gap_s=0.)['interpretations']['any']
    assert merged['penalty_state_raw_rising_edges'] == raw['penalty_events_merged'] == 20
    assert merged['penalty_events_merged'] == 1 and merged['penalty_events'][0]['raw_edges'] == 20
    assert merged['wheel_off_toggles']['front_right'] == 40
    assert merged['samples_by_off_wheel_count']['2'] == 280


def test_sample_gap_weighting_is_capped_and_reported():
    surface = straight_map()
    times = [0., .01, .5, .51]
    evaluations = [sc.evaluate_pose(surface, 0, .22, 0.)]*4
    summary = sc.analyze_series(times, evaluations, max_sample_gap_s=.05)
    assert summary['sample_gaps_over_limit'] == 1 and summary['max_sample_gap_s'] == pytest.approx(.49)
    assert summary['interpretations']['any']['time_by_off_wheel_count_s']['2'] == pytest.approx(.01+.05+.01+.01)


def test_steering_footprint_protrudes_beyond_body_box():
    l, t, r, w = G.wheelbase_m, G.track_width_m, G.wheel_radius_m, G.wheel_width_m
    left, right = sc.ackermann_wheel_angles(1.0)
    assert left == pytest.approx(G.max_wheel_steering_rad) and 0 < right < left
    footprint = sc.steering_footprint(0, 0, 0, 1.0)
    expected = t/2+r*math.sin(left)+w/2*math.cos(left)
    assert footprint.bounds[3] == pytest.approx(expected)
    assert expected-G.body_width_m/2 == pytest.approx(.0120, abs=2e-4)
    assert sc.steering_footprint(0, 0, 0, 0.).bounds == pytest.approx((-.1, -.075, .1, .075))
    # 차체는 도로 안이지만 조향 바퀴 외곽만 도로 밖인 자세: 두 지표가 갈라진다.
    surface = straight_map()
    result = sc.evaluate_pose(surface, 0, .225-.075-.001, 0, 1.0)
    assert result['body_outside_road_area_m2'] == 0 and result['steering_footprint_outside_road_area_m2'] > 0
    assert result['off_counts']['any'] == 0


def test_body_metric_matches_existing_audit_on_official_world():
    surface = sc.SurfaceMap.from_world(WORLD)
    assert surface.road.symmetric_difference(road_polygon(WORLD)).area < 1e-12
    for x, y, yaw in ((9.966, 6.4665, 1.57), (9.966+.2, 6.4665, 1.57), (9.966-.25, 6.8, 1.6)):
        expected = sc.rectangle(x, y, yaw, .20, .15).difference(road_polygon(WORLD)).area
        assert sc.evaluate_pose(surface, x, y, yaw)['body_outside_road_area_m2'] == pytest.approx(expected, abs=1e-12)


def test_official_world_junction_gap_is_other_ground():
    """공식 실행 월드의 갈림길 접합부(7.601, 5.885) 공백은 잔디가 아니다."""
    surface = sc.SurfaceMap.from_world(WORLD)
    assert surface.point_class(7.601, 5.885) == 'other'
