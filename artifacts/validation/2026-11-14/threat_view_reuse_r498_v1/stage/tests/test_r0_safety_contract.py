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
    manifest = ROOT/'artifacts/validation/2026-10-05/g0_baseline/sha256_manifest.json'
    assert manifest.exists()
    captured = json.loads(manifest.read_text())
    original = {row['path']: row['sha256'] for row in captured['files']}
    assert {path: original[path] for path in baseline} == baseline
    assert hashlib.sha256((manifest.parent/captured['source_archive']).read_bytes()).hexdigest() == captured['archive_sha256']
    approved = json.loads((ROOT/'config/tests/g1b_r0_sha256.json').read_text())
    assert hashlib.sha256((ROOT/approved['original_baseline']).read_bytes()).hexdigest() == approved['original_baseline_sha256']
    assert hashlib.sha256(manifest.read_bytes()).hexdigest() == approved['original_source_manifest_sha256']


def test_current_g1b_r0_contract_allows_only_the_registered_launch_change():
    approved = json.loads((ROOT/'config/tests/g1b_r0_sha256.json').read_text())
    old = json.loads((ROOT/approved['original_baseline']).read_text())['sha256']
    current = approved['sha256']
    assert set(current) == set(old)
    changed = {path for path in old if current[path] != old[path]}
    assert changed == set(approved['approved_changes']) == {'src/arena_bringup/launch/simulation.launch.py'}
    for path in changed:
        assert approved['approved_changes'][path]['before'] == old[path]
        assert approved['approved_changes'][path]['after'] == current[path]
    # 이후 계측 확장은 별도 계약에 연결한다. 당시 G1-b manifest는 덮어쓰지 않는다.
    successor = json.loads((ROOT/'config/tests/braking_contact_r0_sha256.json').read_text())
    assert successor['parent_contract'] == 'config/tests/g1b_r0_sha256.json'
    assert hashlib.sha256((ROOT/successor['parent_contract']).read_bytes()).hexdigest() == successor['parent_contract_sha256']
    expected = dict(current)
    assert set(successor['approved_changes']) == {'src/arena_bringup/launch/simulation.launch.py'}
    for path, change in successor['approved_changes'].items():
        assert change['before'] == current[path]
        expected[path] = change['after']
    assert successor['sha256'] == expected
    zero = json.loads((ROOT/'config/tests/zero_speed_r0_sha256.json').read_text())
    assert zero['parent_contract'] == 'config/tests/braking_contact_r0_sha256.json'
    assert hashlib.sha256((ROOT/zero['parent_contract']).read_bytes()).hexdigest() == zero['parent_contract_sha256']
    assert set(zero['approved_changes']) == {'src/arena_bringup/launch/simulation.launch.py'}
    for path, change in zero['approved_changes'].items():
        assert change['before'] == expected[path]
        expected[path] = change['after']
    # 0속도 계약의 구현 해시는 이후 계약 연결의 시작값이다(현재 파일 대조는 마지막 후속 계약에서 한다).
    assert zero['sha256'] == expected
    # 10/8 물리 step 민감도 계약. 0속도 계약은 소급 수정하지 않고 다음 계약에 연결한다.
    step = json.loads((ROOT/'config/tests/physics_step_r0_sha256.json').read_text())
    assert step['parent_contract'] == 'config/tests/zero_speed_r0_sha256.json'
    assert hashlib.sha256((ROOT/step['parent_contract']).read_bytes()).hexdigest() == step['parent_contract_sha256']
    assert set(step['approved_changes']) == {'src/arena_bringup/launch/simulation.launch.py'}
    for path, change in step['approved_changes'].items():
        assert change['before'] == expected[path]
        expected[path] = change['after']
    assert step['implementation_sha256'] == zero['implementation_sha256']
    assert step['sha256'] == expected
    # 10/8 격자 발행 r0/r1(런타임 불변)과 10/9 r2(launch 추적 인자·플러그인 계측)를 차례로 연결한다.
    implementation, parent = dict(step['implementation_sha256']), 'config/tests/physics_step_r0_sha256.json'
    for name in ('scheduled_command_r0_sha256.json', 'scheduled_command_r1_sha256.json',
                 'scheduled_command_r2_sha256.json'):
        successor = json.loads((ROOT/'config/tests'/name).read_text())
        assert successor['parent_contract'] == parent
        assert hashlib.sha256((ROOT/parent).read_bytes()).hexdigest() == successor['parent_contract_sha256']
        for path, change in successor['approved_changes'].items():
            assert change['before'] == expected[path]
            expected[path] = change['after']
        for path, change in successor.get('approved_implementation_changes', {}).items():
            assert change['before'] == implementation.get(path)
            implementation[path] = change['after']
        assert successor['sha256'] == expected and successor['implementation_sha256'] == implementation
        parent = 'config/tests/'+name
    assert set(successor['approved_changes']) == {'src/arena_bringup/launch/simulation.launch.py'}
    # 평가기 GC(r3/r4)·선택형 초기 배치(r5/r6)에서도 최초 R0의 런타임 파일 집합은 유지한다.
    for name in ('scheduled_command_r3_sha256.json', 'scheduled_command_r4_sha256.json',
                 'scheduled_command_r5_sha256.json', 'scheduled_command_r6_sha256.json',
                 'scheduled_command_r7_sha256.json'):
        successor = json.loads((ROOT/'config/tests'/name).read_text())
        assert successor['parent_contract'] == parent
        assert hashlib.sha256((ROOT/parent).read_bytes()).hexdigest() == successor['parent_contract_sha256']
        assert set(successor['approved_changes']) <= {'src/arena_bringup/launch/simulation.launch.py'}
        for path, change in successor['approved_changes'].items():
            assert change['before'] == expected[path]
            expected[path] = change['after']
        assert {path: successor['sha256'][path] for path in expected} == expected
        for path, change in successor.get('approved_implementation_changes', {}).items():
            if path in implementation:
                assert change['before'] == implementation[path]
                implementation[path] = change['after']
        assert {path: successor['implementation_sha256'][path] for path in implementation} == implementation
        parent = 'config/tests/'+name
    for path, value in implementation.items():
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == value
    from source_contract_helpers import hash_before_observation_extension
    assert {path: hash_before_observation_extension(ROOT,path) for path in expected} == expected


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
