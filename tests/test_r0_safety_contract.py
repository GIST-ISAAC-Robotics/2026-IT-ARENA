"""2026-10-05 pre-G0 전용 회귀. 정상 후속 변경 시 새 snapshot/판정을 등록한다.

정확 byte hash는 해당 dirty 기준선의 보존 검증이며 모든 미래 버전/개행의
동일성을 요구하는 일반 기능 검사가 아니다. 변경 전 manifest를 소급 덮어쓰지 않는다.
"""
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from arena_autonomy.local_path import braking_speed, mask_scan, pursuit_command, swept_limit
from arena_autonomy.lidar_observation import audit_front_observation

ROOT = Path(__file__).resolve().parents[1]


def test_pre_g0_working_tree_hashes_preserved():
    baseline = json.loads((ROOT/'config/tests/multi_vehicle_g0_r0_sha256.json').read_text())['sha256']
    assert {path: hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in baseline} == baseline
    manifest = ROOT/'artifacts/validation/2026-10-05/g0_baseline/sha256_manifest.json'
    if manifest.exists():
        original = {row['path']: row['sha256'] for row in json.loads(manifest.read_text())['files']}
        assert {path: original[path] for path in baseline} == baseline


@pytest.mark.parametrize('distance,age,deceleration', [(.5, 0., 2.), (.5, .2, 2.), (3., .3, 2.), (.06, .2, 2.), (.5, -1., 2.)])
def test_braking_solves_independent_stop_distance_equation(distance, age, deceleration):
    speed = braking_speed(distance, age, deceleration)
    available = max(0., distance-.06)
    assert speed >= 0
    assert speed*(max(0., age)+.1)+speed**2/(2*deceleration) == pytest.approx(available)


@pytest.mark.parametrize('point,expected', [((.50, 0.), .36), ((.50, .101), 3.), ((.124, 0.), 0.), ((.50, .10), .36)])
def test_straight_swept_quantization_contract(point, expected):
    # 독립 기하: 반길이 .10+margin .025, 첫 4cm grid=.40, 한 step 차감=.36.
    assert swept_limit(np.array([point]), 0.) == pytest.approx(expected)


def test_turned_swept_pose_matches_independent_circle_center():
    wheelbase, curvature, distance = .135, 1., .4
    yaw = curvature*distance
    x = math.sin(yaw)/curvature-wheelbase/2+wheelbase/2*math.cos(yaw)
    y = (1-math.cos(yaw))/curvature+wheelbase/2*math.sin(yaw)
    # 중심점은 팽창 외곽에 먼저 들어온다. 원 궤적의 독립 scalar 판정으로 첫 간섭을 계산한다.
    first = None
    for index in range(76):
        d, a = index*.04, index*.04
        cx = math.sin(a)-wheelbase/2+wheelbase/2*math.cos(a)
        cy = 1-math.cos(a)+wheelbase/2*math.sin(a)
        dx, dy = x-cx, y-cy
        if abs(math.cos(a)*dx+math.sin(a)*dy) <= .125 and abs(-math.sin(a)*dx+math.cos(a)*dy) <= .1:
            first = max(0., d-.04)
            break
    assert swept_limit(np.array([[x, y]]), math.atan(curvature*wheelbase), wheelbase) == pytest.approx(first)


def test_mask_boundaries_and_front_100_degree_contract():
    scan = SimpleNamespace(angle_min=-math.pi, angle_increment=math.tau/360, ranges=[1.]*360)
    masked = mask_scan(scan)
    assert math.isnan(masked.ranges[0]) and math.isnan(masked.ranges[29])
    assert masked.ranges[30] == masked.ranges[330] == 1.
    assert scan.ranges == [1.]*360
    assert audit_front_observation(masked.ranges, -math.pi, math.tau/360, .05, 12.).reason == 'ok'
    values = [1.]*360
    values[280] = math.inf  # +100° inclusive
    assert audit_front_observation(values, -math.pi, math.tau/360, .05, 12.).reason == 'front_unobserved'
    values[280], values[281] = 1., math.inf
    assert audit_front_observation(values, -math.pi, math.tau/360, .05, 12.).reason == 'ok'


def test_straight_pursuit_stays_on_observed_centerline():
    points = np.column_stack((np.linspace(-.1, 2., 200), np.full(200, .425)))
    speed, steer, meta = pursuit_command(points, 'left', 1., wheelbase=.135, max_speed=2.5)
    assert steer == pytest.approx(0., abs=1e-8) and 0 < speed <= 2.5
    assert meta['path_wall'] == 'left' and not meta['wall_fallback']
    assert meta['observed_path_speed_cap_mps'] == pytest.approx(braking_speed(meta['observed_path_ahead_m'], .2))
