#!/usr/bin/env python3
"""v2026.11.06 배포 world.sdf의 Gazebo 로드·정지 렌더 감사 (WSL, ROS Jazzy/Gazebo Harmonic).

    python3 -I tools/gz_audit.py load   --run-id NN
    python3 -I tools/gz_audit.py render --run-id NN

- 배포 output_final을 ~/arena_scratch 아래 새 경로로 복사하고 SHA-256을 대조한 뒤 그 사본에서 실행한다.
  world.sdf는 바꾸지 않는다. 렌더는 별도 server.config(Sensors 추가)와 실행 중 생성한 정지 카메라만 사용한다.
- 결과는 checks/gazebo/<phase>_<run-id>/ 에 남기고 기존 결과 폴더를 덮어쓰지 않는다.
- 차량·제어기·지도·정답 위치를 쓰지 않는다. 주행/실물 검증이 아니다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid

ART = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[5]
SRC = ART / 'extracted/output_final'
WORLD_NAME = 'it_arena_track'
for extra in ('/opt/ros/jazzy/lib/python3.12/site-packages', '/opt/ros/jazzy/local/lib/python3.12/dist-packages'):
    if extra not in sys.path:
        sys.path.append(extra)
sys.path.insert(0, str(ROOT / 'src/arena_autonomy'))  # 팀 검출기(읽기 전용 import)

SERVER_RENDER_CONFIG = """<server_config><plugins>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
</plugins></server_config>
"""

# 현재 B0183 시뮬 구성(b0183_c1_jr60p20_sim.yaml): 1280x720, 수평화각 75°, 차체 원점(축간 중앙·지면) 기준
# x=+0.055, z=0.145, 수평. 도로 윗면 z=0.003 위에 바퀴가 놓인다고 보고 카메라 z=0.148.
VEH_CAM = dict(w=1280, h=720, hfov=math.radians(75.0), dx=0.055, z=0.003 + 0.145)
LIDAR_Z = 0.003 + 0.075


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, data) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=1, default=str) + '\n', encoding='utf-8')


def clean_log(text: str) -> str:
    text = re.sub(r'\x1b\[[0-9;]*m', '', text)
    return text.replace(str(ROOT), '<WORKSPACE>').replace(str(Path.home()), '<WSL_HOME>')


def err_wrn(text: str):
    return [l for l in clean_log(text).splitlines()
            if any(k in l for k in ('[Err]', '[Wrn]', 'Error', 'ERROR', 'Warn', 'WARN', 'Exception', 'Unable', 'Failed', 'failed'))]


def prepare_copy(run_dir_name: str) -> Path:
    expected = json.loads((ART / 'checks/zip_audit.json').read_text(encoding='utf-8'))['files']
    base = Path.home() / 'arena_scratch/official_v20261106_audit_v1' / run_dir_name
    if base.exists():
        raise FileExistsError(f'scratch exists: {base}')
    target = base / 'output_final'
    shutil.copytree(SRC, target)
    for rel, rec in expected.items():
        if not rel.startswith('output_final/'):
            continue
        p = base / rel
        if sha256(p) != rec['sha256']:
            raise RuntimeError(f'copy hash mismatch {rel}')
    return target


def group_alive(pgid: int):
    alive = []
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():
            continue
        try:
            if os.getpgid(int(d.name)) == pgid:
                state = (d / 'stat').read_text().rsplit(')', 1)[1].split()[0]
                if state != 'Z':
                    alive.append(int(d.name))
        except (ProcessLookupError, PermissionError, FileNotFoundError):
            pass
    return alive


def stop_group(proc: subprocess.Popen):
    sigs = []
    for sig, grace in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        if not group_alive(proc.pid):
            break
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        sigs.append(sig.name)
        end = time.monotonic() + grace
        while group_alive(proc.pid) and time.monotonic() < end:
            time.sleep(0.1)
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        pass
    return {'return_code': proc.returncode, 'signals': sigs, 'remaining_pids': group_alive(proc.pid)}


def run_cmd(cmd, cwd, env, log_path: Path, timeout: float):
    t0 = time.monotonic()
    with log_path.open('wb') as log:
        proc = subprocess.Popen(cmd, cwd=cwd, env=env, stdin=subprocess.DEVNULL, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        expired = False
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            expired = True
        cleanup = stop_group(proc)
    text = log_path.read_text(encoding='utf-8', errors='replace')
    log_path.write_text(clean_log(text), encoding='utf-8')
    return {'command': cmd, 'timeout_s': timeout, 'timed_out': expired, 'wall_s': round(time.monotonic() - t0, 3),
            **cleanup, 'err_wrn_lines': err_wrn(text), 'log': log_path.name}


def phase_load(out: Path, run_id: str):
    world_dir = prepare_copy(f'load_{run_id}')
    env = os.environ.copy()
    env.pop('GZ_SIM_SERVER_CONFIG_PATH', None)
    env['GZ_PARTITION'] = 'arena_v1106_load_' + uuid.uuid4().hex[:10]
    cfg = Path.home() / '.gz/sim/8/server.config'
    res = {'phase': 'load', 'world_copy': str(world_dir).replace(str(Path.home()), '<WSL_HOME>'),
           'world_sha256': sha256(world_dir / 'world.sdf'),
           'default_server_config': cfg.read_text(encoding='utf-8') if cfg.exists() else None,
           'versions': {}}
    for name, cmd in (('gz_sim', ['gz', 'sim', '--versions']), ('gz_sdf', ['gz', 'sdf', '--versions'])):
        res['versions'][name] = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=30).stdout.strip()
    res['sdf_check'] = run_cmd(['gz', 'sdf', '-k', 'world.sdf'], world_dir, env, out / 'sdf_check.log', 60)
    res['server_iterations'] = run_cmd(['stdbuf', '-oL', '-eL', 'gz', 'sim', '-s', '-r', '-v', '4', '--iterations', '2000', 'world.sdf'],
                                       world_dir, env, out / 'server_iterations.log', 180)
    log = (out / 'server_iterations.log').read_text(encoding='utf-8')
    res['server_iterations'].update({
        'loaded_physics': 'Loaded system [gz::sim::systems::Physics]' in log,
        'serving_world_controls': 'Serving world controls' in log,
        'world_name_in_log': WORLD_NAME in log,
        'mentions_dartsim': 'dartsim' in log})
    res['world_unchanged_after'] = sha256(world_dir / 'world.sdf') == res['world_sha256']
    return res


# ------------------------------------------------------------------ render
def rpy_look_at(pos, target):
    dx, dy, dz = (target[i] - pos[i] for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    return (0.0, pitch, yaw)


def camera_sdf(name, topic, pos, rpy, w, h, hfov, near=0.02, far=40.0, rate=2):
    return f'''<sdf version="1.8"><model name="{name}"><static>true</static>
<pose>{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} {rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}</pose>
<link name="link"><sensor name="cam" type="camera"><always_on>true</always_on><update_rate>{rate}</update_rate>
<topic>{topic}</topic><camera><horizontal_fov>{hfov:.6f}</horizontal_fov>
<image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
<clip><near>{near}</near><far>{far}</far></clip></camera></sensor></link></model></sdf>'''


def lat_tag(lat):
    """ROS 토픽 이름에 쓸 수 있는 횡 위치 표기(r=진행 방향 오른쪽, l=왼쪽, c=중앙)."""
    if abs(lat) < 1e-9:
        return 'c00'
    return ('r' if lat > 0 else 'l') + f"{abs(int(round(lat * 100))):02d}"


def build_views():
    scene = json.loads((SRC / 'scene.json').read_text(encoding='utf-8'))
    views = []

    def top(name, cx, cy, span_x, w, h, height, label):
        hfov = 2 * math.atan(span_x / 2 / height)
        views.append(dict(name=name, kind='top', label=label, pos=(cx, cy, height), rpy=(0.0, math.pi / 2, math.pi / 2),
                          w=w, h=h, hfov=hfov, far=height + 5))

    def look(name, pos, target, w, h, hfov, label, kind='oblique'):
        views.append(dict(name=name, kind=kind, label=label, pos=pos, rpy=rpy_look_at(pos, target), w=w, h=h, hfov=hfov, far=40))

    def vehicle(name, x, y, yaw, label, kind='vehicle', z=None, hfov=None, w=None, h=None, dx=None):
        dx = VEH_CAM['dx'] if dx is None else dx
        views.append(dict(name=name, kind=kind, label=label,
                          pos=(x + dx * math.cos(yaw), y + dx * math.sin(yaw), VEH_CAM['z'] if z is None else z),
                          rpy=(0.0, 0.0, yaw), w=w or VEH_CAM['w'], h=h or VEH_CAM['h'], hfov=hfov or VEH_CAM['hfov'],
                          near=0.02, far=30, body_xy_yaw=(x, y, yaw)))

    top('overview_top', 6.14, 7.2, 9.2, 1400, 2150, 25.0, '전체 코스 수직 상부(25 m)')
    look('overview_oblique_se', (13.0, -3.0, 8.0), (6.14, 7.0, 0.0), 1920, 1200, 1.0, '남동 사선 전체')
    top('detail_start_grid', 9.8, 5.0, 1.6, 900, 1800, 3.5, '그리드·신호·출발선 상부')
    top('detail_P22', 5.22, 4.5, 2.6, 1300, 1300, 4.0, 'P22 지름길 상부')
    top('detail_P21', 3.2, 2.1, 2.4, 1200, 1200, 4.0, 'P21 지름길·삼각 섬 상부')
    top('detail_corner_C1', 9.8, 13.2, 1.6, 800, 800, 3.0, '직각 코너 C1 상부')
    look('oblique_P21_triangle', (2.2, 3.7, 0.6), (3.1, 1.9, 0.0), 1600, 1000, 1.2, 'P21 삼각 섬 사선(0.6 m)')
    look('oblique_start_signal', (10.6, 3.0, 1.2), (9.8, 6.1, 0.3), 1600, 1000, 1.0, '출발 구역 사선(1.2 m)')
    for slot in scene['starting_grid']['slots']:
        vehicle(f"grid{slot['index']}_cam", slot['x'], slot['y'], slot['yaw_rad'], f"그리드 {slot['index']} 차량 카메라")
    # ID20: (6.155, 6.0) +x 면, 접근 y=6.0을 -x 진행. d = 카메라와 마커판 사이 진행 방향 거리
    for d, lat in ((0.8, 0.0), (1.2, 0.0), (1.6, 0.0), (1.2, 0.2), (1.2, -0.2)):
        cx = 6.155 + d
        vehicle(f"id20_d{int(round(d*100)):03d}_{lat_tag(lat)}", cx - VEH_CAM['dx'] * math.cos(math.pi), 6.0 + lat, math.pi,
                f'ID20 접근 d={d} m, 횡 {lat:+.1f} m')
    # ID30: (2.48, 3.505) +y 면, 접근 x=2.48을 -y 진행
    for d, lat in ((0.8, 0.0), (1.2, 0.0), (1.8, 0.0), (2.4, 0.0), (1.2, 0.2), (1.2, -0.2)):
        cy = 3.505 + d
        vehicle(f"id30_d{int(round(d*100)):03d}_{lat_tag(lat)}", 2.48 - lat, cy - VEH_CAM['dx'] * math.sin(-math.pi / 2), -math.pi / 2,
                f'ID30 접근 d={d} m, 횡 {lat:+.1f} m')
    # LiDAR 광축 높이(0.078 m) 광각 사진: 낮은 물체 확인용
    for name, x, y, yaw, label in (('lidar_front_grid', 9.71, 4.72, math.pi / 2, '선두 그리드 LiDAR 높이'),
                                   ('lidar_P22_approach', 6.14, 5.4, -math.pi / 2, 'P22 접근 LiDAR 높이'),
                                   ('lidar_P21_approach', 2.48, 3.4, -math.pi / 2, 'P21 접근 LiDAR 높이')):
        vehicle(name, x, y, yaw, label, kind='lidar_height', z=LIDAR_Z, hfov=2.0, w=1600, h=600, dx=0.06)
    return views, scene


def project(view, p):
    """핀홀 투영(px, py, depth). 카메라 +x 전방, +y 왼쪽, +z 위."""
    import numpy as np
    roll, pitch, yaw = view['rpy']
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    v = R.T @ (np.array(p) - np.array(view['pos']))
    if v[0] <= 1e-6:
        return None
    f = view['w'] / 2 / math.tan(view['hfov'] / 2)
    return (view['w'] / 2 - f * v[1] / v[0], view['h'] / 2 - f * v[2] / v[0], float(v[0]))


def marker_geometry(view, marker):
    """마커 코드(7 cm) 네 모서리 투영과 화면 포함 여부."""
    x, y, z, yaw = marker['x'], marker['y'], marker['z'], marker['yaw']
    lx, ly = -math.sin(yaw), math.cos(yaw)
    corners = []
    for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append(project(view, (x + a * 0.035 * lx, y + a * 0.035 * ly, z + b * 0.035)))
    if any(c is None for c in corners):
        return {'in_front': False}
    us = [c[0] for c in corners]
    vs = [c[1] for c in corners]
    inside = all(0 <= u < view['w'] for u in us) and all(0 <= v < view['h'] for v in vs)
    partial = any(0 <= u < view['w'] and 0 <= v < view['h'] for u, v in zip(us, vs))
    # 판 법선과 카메라 방향
    cam = view['pos']
    to_cam = (cam[0] - x, cam[1] - y)
    dot = (math.cos(yaw) * to_cam[0] + math.sin(yaw) * to_cam[1]) / max(1e-9, math.hypot(*to_cam))
    return {'in_front': True, 'code_fully_in_frame': inside, 'code_partly_in_frame': partial,
            'code_px_bbox': [round(min(us), 1), round(min(vs), 1), round(max(us), 1), round(max(vs), 1)],
            'code_px_width': round(max(us) - min(us), 1), 'code_px_height': round(max(vs) - min(vs), 1),
            'range_m': round(math.dist(cam, (x, y, z)), 3), 'face_cos_to_camera': round(dot, 4)}


def phase_render(out: Path, run_id: str, timeout: float):
    import numpy as np
    import cv2
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from arena_autonomy.core import marker_ids
    from arena_autonomy.race_start_local import LocalOfficialStartSignal

    world_dir = prepare_copy(f'render_{run_id}')
    views, scene = build_views()
    markers = []
    for m in scene['aruco_markers']['markers']:
        p = m['pose']
        markers.append({'id': m['id'], 'x': p['x'], 'y': p['y'], 'z': p['z'] + 0.05, 'yaw': p['yaw_rad']})
    lamps = {}
    import xml.etree.ElementTree as ET
    root = ET.parse(world_dir / 'world.sdf').getroot()
    for link in root.iter('link'):
        if link.attrib['name'].startswith('lamp_'):
            lamps[link.attrib['name']] = [float(v) for v in link.findtext('pose').split()[:3]]
    cfg = out / 'server_render.config'
    cfg.write_text(SERVER_RENDER_CONFIG, encoding='utf-8')
    env = os.environ.copy()
    env['GZ_SIM_SERVER_CONFIG_PATH'] = str(cfg)
    env['GZ_PARTITION'] = 'arena_v1106_render_' + uuid.uuid4().hex[:10]
    env['ROS_DOMAIN_ID'] = str(140 + os.getpid() % 40)
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    env['ROS_STATIC_PEERS'] = ''
    os.environ.update({k: env[k] for k in ('GZ_PARTITION', 'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS')})
    res = {'phase': 'render', 'world_sha256': sha256(world_dir / 'world.sdf'),
           'world_copy': str(world_dir).replace(str(Path.home()), '<WSL_HOME>'),
           'server_config': SERVER_RENDER_CONFIG, 'vehicle_camera': {**VEH_CAM, 'hfov_deg': 75.0,
           'source': 'src/arena_description/config/b0183_c1_jr60p20_sim.yaml (실물 보정 미측정, 핀홀)'},
           'detector': {'function': 'arena_autonomy.core.marker_ids(rgb, .02) 현행 local_pursuit/vision_process 값',
                        'comparison': 'marker_ids(rgb) OpenCV 기본 minMarkerDistanceRate=0.05'},
           'views': [], 'note': '정지 카메라 렌더. 차량 모델·충돌·주행·노출/왜곡 없음.'}
    procs = []
    logs = []
    try:
        slog = (out / 'server.log').open('wb')
        logs.append(slog)
        server = subprocess.Popen(['stdbuf', '-oL', '-eL', 'gz', 'sim', '-s', '-r', '-v', '4', 'world.sdf'], cwd=world_dir, env=env,
                                  stdin=subprocess.DEVNULL, stdout=slog, stderr=subprocess.STDOUT, start_new_session=True)
        procs.append(server)
        deadline = time.monotonic() + 90
        ready = False
        while time.monotonic() < deadline:
            if server.poll() is not None:
                raise RuntimeError('server exited early')
            out_l = subprocess.run(['gz', 'service', '-l'], capture_output=True, text=True, env=env, timeout=20).stdout
            if f'/world/{WORLD_NAME}/create' in out_l:
                ready = True
                break
            time.sleep(1.0)
        res['server_ready'] = ready
        if not ready:
            raise RuntimeError('create service not ready')
        topics = {v['name']: f"/audit_v1106/{v['name']}/image" for v in views}
        blog = (out / 'bridge.log').open('wb')
        logs.append(blog)
        bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                   *[f'{t}@sensor_msgs/msg/Image[gz.msgs.Image' for t in topics.values()]],
                                  env=env, stdin=subprocess.DEVNULL, stdout=blog, stderr=subprocess.STDOUT, start_new_session=True)
        procs.append(bridge)
        rclpy.init()
        node = rclpy.create_node('official_v1106_render_audit')
        frames = {n: [] for n in topics}

        def make_cb(n):
            def cb(msg):
                if len(frames[n]) < 4:
                    frames[n].append(msg)
            return cb
        subs = [node.create_subscription(Image, t, make_cb(n), qos_profile_sensor_data) for n, t in topics.items()]
        spawn = []
        for v in views:
            sdf = camera_sdf('audit_' + v['name'], topics[v['name']], v['pos'], v['rpy'], v['w'], v['h'], v['hfov'],
                             v.get('near', 0.02), v.get('far', 40.0))
            r_ = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD_NAME}/create', '--reqtype', 'gz.msgs.EntityFactory',
                                 '--reptype', 'gz.msgs.Boolean', '--timeout', '10000', '--req', 'sdf: ' + json.dumps(sdf)],
                                capture_output=True, text=True, env=env, timeout=30)
            spawn.append({'name': v['name'], 'ok': 'data: true' in r_.stdout, 'stderr': r_.stderr.strip()[:300]})
        res['spawn'] = spawn
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and min(len(f) for f in frames.values()) < 3:
            rclpy.spin_once(node, timeout_sec=0.1)
        res['frames_received'] = {n: len(f) for n, f in frames.items()}
        imgdir = out / 'images'
        anndir = out / 'annotated'
        imgdir.mkdir()
        anndir.mkdir()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for v in views:
            fl = frames[v['name']]
            entry = {k: v[k] for k in ('name', 'kind', 'label', 'w', 'h')}
            entry.update({'pos': [round(c, 4) for c in v['pos']], 'rpy': [round(c, 5) for c in v['rpy']],
                          'hfov_rad': round(v['hfov'], 5), 'frames': len(fl)})
            if v.get('body_xy_yaw'):
                entry['vehicle_body_xy_yaw'] = [round(c, 4) for c in v['body_xy_yaw']]
            if not fl:
                res['views'].append(entry)
                continue
            msg = fl[-1]
            arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
            if msg.encoding != 'rgb8':
                entry['encoding'] = msg.encoding
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            path = imgdir / f"{v['name']}.png"
            cv2.imwrite(str(path), bgr)
            gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
            entry.update({'png': f"images/{v['name']}.png", 'sha256': sha256(path), 'stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                          'gray_mean_std': [round(float(gray.mean()), 2), round(float(gray.std()), 2)]})
            ann = bgr.copy()
            if v['kind'] == 'vehicle':
                ids_team = sorted(set(marker_ids(arr, .02)))
                ids_default = sorted(set(marker_ids(arr)))
                params = cv2.aruco.DetectorParameters_create()
                params.minMarkerDistanceRate = 0.02
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
                det = []
                if ids is not None:
                    for c, i in zip(corners, ids.flatten()):
                        c = c.reshape(4, 2)
                        det.append({'id': int(i), 'px_bbox': [round(float(c[:, 0].min()), 1), round(float(c[:, 1].min()), 1),
                                                               round(float(c[:, 0].max()), 1), round(float(c[:, 1].max()), 1)]})
                        cv2.polylines(ann, [c.astype(np.int32)], True, (0, 0, 255), 2)
                        cv2.putText(ann, f'ID{int(i)}', (int(c[0, 0]), int(c[0, 1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                entry['ids_team_minrate_002'] = ids_team
                entry['ids_opencv_default_005'] = ids_default
                entry['detections'] = det
                entry['unexpected_ids'] = [i for i in ids_team + ids_default if i not in (0, 20, 30, 45)]
                entry['rejected_candidates'] = 0 if rejected is None else len(rejected)
                entry['marker_projection'] = {}
                for m in markers:
                    g = marker_geometry(v, m)
                    if g.get('in_front') and g.get('code_partly_in_frame'):
                        entry['marker_projection'][str(m['id'])] = g
                        b = g['code_px_bbox']
                        cv2.rectangle(ann, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (255, 0, 255), 1)
                # 신호등: 램프 투영과 팀 검출기(LocalOfficialStartSignal) 3프레임 관측
                lp = {}
                for n, p in lamps.items():
                    pr = project(v, p)
                    if pr and 0 <= pr[0] < v['w'] and 0 <= pr[1] < v['h']:
                        u, vv = int(pr[0]), int(pr[1])
                        hsv = cv2.cvtColor(arr[max(0, vv - 1):vv + 2, max(0, u - 1):u + 2], cv2.COLOR_RGB2HSV).reshape(-1, 3)
                        lp[n] = {'px': [round(pr[0], 1), round(pr[1], 1)], 'range_m': round(pr[2], 3),
                                 'hsv_center_median': [int(x) for x in np.median(hsv, axis=0)]}
                        cv2.circle(ann, (u, vv), 6, (0, 255, 255), 1)
                entry['lamps_in_frame'] = len(lp)
                entry['lamp_projection'] = lp
                if lp:
                    us = [q['px'][0] for q in lp.values()]
                    vs = [q['px'][1] for q in lp.values()]
                    entry['lamp_bank_px_bbox'] = [min(us), min(vs), max(us), max(vs)]
                    # 마커 코드 투영과 램프 묶음 투영의 겹침
                    over = {}
                    for mid, g in entry['marker_projection'].items():
                        b = g['code_px_bbox']
                        over[mid] = not (b[2] < min(us) - 8 or b[0] > max(us) + 8 or b[3] < min(vs) - 8 or b[1] > max(vs) + 8)
                    entry['marker_overlaps_lamp_bank'] = over
                # 정적 장면이므로 같은 프레임을 0.1 s 간격 시각으로 3회 넣어 빨강 배열 인식만 본다(점등 전이 아님).
                sig = LocalOfficialStartSignal()
                states = []
                base_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                for k in range(3):
                    sig.update(arr.copy(), base_t + 0.1 * k)
                    states.append(sig.observed)
                entry['team_start_signal_states_same_frame_x3'] = states
            cv2.putText(ann, v['label'].encode('ascii', 'ignore').decode() or v['name'], (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
            cv2.putText(ann, v['name'], (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imwrite(str(anndir / f"{v['name']}.png"), ann)
            res['views'].append(entry)
        for s in subs:
            node.destroy_subscription(s)
        node.destroy_node()
        rclpy.shutdown()
        res['capture_complete'] = all(len(f) >= 3 for f in frames.values())
    except Exception as exc:  # 실패도 기록한다.
        res['error'] = repr(exc)
        res['capture_complete'] = False
    finally:
        res['cleanup'] = [stop_group(p) for p in reversed(procs)]
        for lf in logs:
            lf.close()
        for name in ('server.log', 'bridge.log'):
            p = out / name
            if p.exists():
                text = p.read_text(encoding='utf-8', errors='replace')
                p.write_text(clean_log(text), encoding='utf-8')
                res[f'{name}_err_wrn'] = err_wrn(text)
        res['world_unchanged_after'] = sha256(world_dir / 'world.sdf') == res['world_sha256']
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('phase', choices=('load', 'render'))
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--timeout', type=float, default=240)
    a = ap.parse_args()
    out = ART / 'checks/gazebo' / f'{a.phase}_{a.run_id}'
    out.mkdir(parents=True, exist_ok=False)
    started = time.time()
    res = phase_load(out, a.run_id) if a.phase == 'load' else phase_render(out, a.run_id, a.timeout)
    res['started_unix'] = started
    res['finished_unix'] = time.time()
    write_json(out / 'result.json', res)
    ok = (res.get('capture_complete', True) and all(not c.get('remaining_pids') for c in res.get('cleanup', []))
          and res.get('world_unchanged_after', False))
    print(json.dumps({'out': str(out).replace(str(ROOT), '<WORKSPACE>'), 'ok': ok,
                      'error': res.get('error')}, ensure_ascii=False))
    (out / 'EXIT').write_text('0\n' if ok else '1\n')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
