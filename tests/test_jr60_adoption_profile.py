"""JR60P20 임시 채택 프로필: 좌표·배선 예약 외곽·구조 각도·마커·실행 인자 정적 불변. 주행 없음."""
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from arena_autonomy.known_structure import validate as validate_structure
from arena_autonomy.local_path import mask_scan
from arena_autonomy.local_scene import blind_parameter
from test_low_speed_arc_reacquire import launch_module

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT/'src/arena_description/config/b0183_c1_jr60p20_sim.yaml'
PREVIOUS = ROOT/'src/arena_description/config/b0183_c1_jr50_sim.yaml'
RUN_PROFILE = ROOT/'artifacts/validation/2026-11-04/lidar_jr_forward_v1/profiles/jr60p20.yaml'
RUN_RECORD = ROOT/'artifacts/validation/2026-11-04/lidar_jr_forward_v1/jr60p20_1_registered.json'
# 관성·충돌을 포함한 최소 차체. 장착 구조가 표시 형상만 추가하는지 확인한다.
MODEL = """<sdf version='1.10'><model name='arena_car'><link name='chassis'>
<inertial><mass>2.0</mass></inertial>
<collision name='collision'><geometry><box><size>0.2 0.15 0.06</size></box></geometry></collision>
<visual name='visual'><geometry><box><size>0.188 0.13 0.06</size></box></geometry></visual>
</link></model></sdf>"""
WHEEL_RADIUS, BODY_HEIGHT = .0325, .06
MARKER_SIZE = .05
RAISED_MARKER = ([-0.1005, 0., 0.140], MARKER_SIZE, True)
POST_D = .007
CHANNEL_WIDTH, CHANNEL_DEPTH = .016, .012  # 접선폭 × 반경깊이, 벽 2 mm
TOL = 1e-6  # µm 반올림한 중심


def lidar():
    return yaml.safe_load(PROFILE.read_text(encoding='utf-8'))['lidar']


def rear_geometry():
    sx, sy, _ = lidar()['xyz_m']
    rows = []
    for x, y, _ in lidar()['scan_plane_posts']:
        if x > 0:
            continue
        distance = math.hypot(x - sx, y - sy)
        u = ((x - sx)/distance, (y - sy)/distance)
        t = (-u[1], u[0])
        center = (x + (POST_D/2 + CHANNEL_DEPTH/2)*u[0], y + (POST_D/2 + CHANNEL_DEPTH/2)*u[1])
        corners = [(center[0] + a*CHANNEL_DEPTH/2*u[0] + b*CHANNEL_WIDTH/2*t[0],
                    center[1] + a*CHANNEL_DEPTH/2*u[1] + b*CHANNEL_WIDTH/2*t[1]) for a in (-1, 1) for b in (-1, 1)]
        rows.append(dict(post=(x, y), u=u, center=center, corners=corners))
    return rows


def test_selected_coordinates_and_only_wire_reserve_differs_from_run_profile():
    profile = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    run = yaml.safe_load(RUN_PROFILE.read_text(encoding='utf-8'))
    assert profile['lidar']['xyz_m'] == [.060, 0., .075]
    assert profile['lidar']['scan_plane_posts'] == [[.020, .062, .007], [.020, -.062, .007],
                                                    [-.080, .052, .007], [-.080, -.052, .007]]
    assert profile['lidar']['masked_rear_half_angle_deg'] == 0.
    assert profile['lidar']['known_structure_mask'] == 'structure_aware'
    # 1랩 실행 JR60P20과 다른 키는 배선 예약 외곽뿐이다(카메라·IMU·마스크·차체 표시 동일).
    run['lidar'].pop('scan_plane_wires')
    wires = profile['lidar'].pop('scan_plane_wires')
    assert profile == run
    assert len(wires) == 2 and all(w[2] == .020 for w in wires)
    # 이전 JR50 프리셋은 재현용으로 그대로 남는다.
    previous = yaml.safe_load(PREVIOUS.read_text(encoding='utf-8'))['lidar']
    assert previous['xyz_m'] == [.05, 0., .075]
    assert previous['scan_plane_wires'] == [[-.08, .052, .002], [-.08, -.052, .002]]


def test_reserve_envelope_covers_channel_outside_post_inside_footprint():
    wires = lidar()['scan_plane_wires']
    for row, (wx, wy, diameter) in zip(rear_geometry(), wires):
        cx, cy = row['center']
        assert math.hypot(wx - cx, wy - cy) <= TOL
        # 외곽 16×12 사각형 전체가 Ø20 외접원 안(반대각선 10 mm)
        assert max(math.hypot(x - wx, y - wy) for x, y in row['corners']) <= diameter/2 + TOL
        # 사각형 안쪽 면은 지주 외면에 접하고 센서 반대편에 있다(지주 속에 숨기지 않음).
        px, py = row['post']
        inner = min(((x - px)*row['u'][0] + (y - py)*row['u'][1]) for x, y in row['corners'])
        assert math.isclose(inner, POST_D/2, abs_tol=TOL)
        assert math.hypot(wx - px, wy - py) > POST_D/2 and diameter/2 > POST_D/2
        # 센서 뒤쪽·200×150 외곽 안
        assert wx + diameter/2 < 0 < lidar()['xyz_m'][0]
        for x, y in [*row['corners'], (wx - diameter/2, wy), (wx, wy + math.copysign(diameter/2, wy))]:
            assert abs(x) <= .100 and abs(y) <= .075


def mount():
    launch = launch_module()
    profile = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    model, sectors = launch._mount_trial(MODEL, profile, WHEEL_RADIUS, BODY_HEIGHT, *RAISED_MARKER)
    return ET.fromstring(model), json.loads(sectors)


def test_model_visuals_and_structure_sectors_come_from_same_geometry():
    model, sectors = mount()
    chassis = model.find('model/link')
    original = ET.fromstring(MODEL).find('model/link')
    # 질량·충돌체 불변, 표시 형상만 추가
    assert ET.tostring(chassis.find('inertial')) == ET.tostring(original.find('inertial'))
    assert [ET.tostring(c) for c in chassis.findall('collision')] == [ET.tostring(c) for c in original.findall('collision')]
    sx, sy, sz = lidar()['xyz_m']
    chassis_z = WHEEL_RADIUS + BODY_HEIGHT/2
    shapes = []
    for prefix, rows in (('c1_mount_post_', lidar()['scan_plane_posts']), ('c1_mount_wire_', lidar()['scan_plane_wires'])):
        for index, (x, y, diameter) in enumerate(rows):
            visual = chassis.find(f"visual[@name='{prefix}{index}']")
            px, py, pz = (float(v) for v in visual.find('pose').text.split()[:3])
            assert (px, py) == (x, y) and math.isclose(pz + chassis_z, sz)
            assert float(visual.find('geometry/cylinder/radius').text) == diameter/2
            shapes.append((px, py, diameter))
    assert len(chassis.findall('visual/geometry/cylinder')) == 6
    # 마커를 올렸으므로 구조 각도는 지주 4 + 예약 외곽 2, 마커 각도 없음.
    assert len(sectors) == 6
    for (x, y, diameter), (center, half) in zip(shapes, sectors):
        distance = math.hypot(x - sx, y - sy)
        assert math.isclose(center, math.degrees(math.atan2(y - sy, x - sx)))
        assert math.isclose(half, math.degrees(math.asin(diameter/2/distance)) + .5)
    # 예약 외곽 그림자가 같은 방위의 뒤 지주 그림자를 덮는다.
    for post, wire in zip(sectors[2:4], sectors[4:6]):
        assert math.isclose(post[0], wire[0], abs_tol=.01) and wire[1] > post[1] + 2


def test_rear_center_stays_open_for_planner_and_independent_safety():
    _, sectors = mount()
    text = json.dumps(sectors)
    # 계획기와 독립 보호층은 같은 문자열을 같은 검증으로 받는다.
    parsed = validate_structure('structure_aware_v1', text)
    spec = blind_parameter(0., text)
    assert [list(s) for s in parsed] == [list(s) for s in spec['sectors_deg']]
    n = 3600
    scan = SimpleNamespace(angle_min=-math.pi, angle_increment=math.tau/n, ranges=[2.]*n)
    nan = np.isnan(mask_scan(scan, spec).ranges)
    angles = np.degrees(-math.pi + np.arange(n)*math.tau/n)
    # 정후방 ±15°는 가리지 않는다. 가린 광선은 거리를 만들지 않고 NaN(unknown)으로 남는다.
    assert not nan[np.abs(np.abs(angles) - 180.) <= 15.].any()
    rear_gap = 2*(180. - max(c + h for c, h in sectors if c > 90.))
    assert 32.0 < rear_gap < 33.0  # 물리 33.53° − 양끝 여유 0.5°×2; 얇은 배선 37.07°
    wire = sectors[4]
    assert nan[np.abs(angles - wire[0]) <= wire[1] - .1].all()


def test_raised_marker_clears_scan_plane_and_reserve_display():
    _, _, sz = lidar()['xyz_m']
    marker_bottom = RAISED_MARKER[0][2] - MARKER_SIZE/2
    reserve_top = sz + .06/2  # 표시 원기둥 길이 0.06 m
    assert marker_bottom > reserve_top > sz
    # 평면에서도 판(±25 mm)과 예약 외곽(|y| 45 mm 이상)이 겹치지 않는다.
    assert min(abs(y) - d/2 for _, y, d in lidar()['scan_plane_wires']) > MARKER_SIZE/2
    # 기본 높이(0.090 m)라면 마커가 스캔면에 걸려 정후방 각도가 추가된다.
    launch = launch_module()
    profile = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    default = launch._mount_trial(MODEL, profile, WHEEL_RADIUS, BODY_HEIGHT, [-0.1005, 0., 0.090], MARKER_SIZE, True)[1]
    assert len(json.loads(default)) == 7 and math.isclose(abs(json.loads(default)[-1][0]), 180.)


def runner(*extra):
    output = subprocess.run([sys.executable, '-B', str(ROOT/'scripts/run_current_solo.py'), '/tmp/jr60_print_only',
                             '--print-args', *extra], check=True, capture_output=True, text=True).stdout
    return json.loads(output)


def test_solo_runner_selects_jr60p20_and_raised_marker():
    args = runner()
    assert args[args.index('--local-sensor-config') + 1] == str(PROFILE.resolve())
    assert args[args.index('--pilot-marker-z') + 1] == '0.140'
    # 33.225 s 기록의 실행 인자와 출력·프로필 경로 외에 같다(성능 근거 승계 아님).
    recorded = json.loads(RUN_RECORD.read_text(encoding='utf-8'))['args']
    strip = lambda values: [v for i, v in enumerate(values)
                            if i == 0 or values[i-1] not in ('--local-sensor-config', '--output')]
    assert strip(args) == strip(recorded)
    previous = runner('--previous-jr50')
    assert previous[previous.index('--local-sensor-config') + 1] == str(PREVIOUS.resolve())
    assert strip(previous) == strip(args)


def test_traffic_validator_marker_flag_is_opt_in_and_forwarded():
    source = (ROOT/'scripts/validate_g1_traffic.py').read_text(encoding='utf-8')
    assert "parser.add_argument('--pilot-marker-z', type=float, default=None," in source
    assert "extra.append(f'pilot_marker_z:={args.pilot_marker_z:.3f}')" in source
    # 범위 밖·G1 계열이 아닌 모드는 ROS를 시작하기 전에 거절한다.
    for extra in (['--mode', 'g1', '--pilot-marker-z', '0.10'], ['--mode', 'r0', '--pilot-marker-z', '0.14']):
        result = subprocess.run([sys.executable, '-B', str(ROOT/'scripts/validate_g1_traffic.py'), '--case', 'stop_restart',
                                 '--seed', '0', '--output', '/tmp/jr60_reject_only', *extra],
                                cwd=ROOT, capture_output=True, text=True)
        assert result.returncode == 2 and '--pilot-marker-z' in result.stderr
    # launch 사슬: g1_follow → (local_pursuit/demo) → simulation의 pilot_marker_z
    follow = (ROOT/'src/arena_bringup/launch/g1_follow.launch.py').read_text(encoding='utf-8')
    assert "launch_arguments={'pilot_marker_z': LaunchConfiguration('pilot_marker_z')}" in follow
    pursuit = (ROOT/'src/arena_bringup/launch/local_pursuit.launch.py').read_text(encoding='utf-8')
    assert '"pilot_marker_z": ""}' in pursuit and '**{key: LaunchConfiguration(key) for key in defaults}' in pursuit
    demo = (ROOT/'src/arena_bringup/launch/demo.launch.py').read_text(encoding='utf-8')
    assert '"pilot_marker_z": LaunchConfiguration("pilot_marker_z")' in demo
