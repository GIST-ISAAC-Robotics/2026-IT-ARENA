"""JR60P20 채택 프리셋의 평면 기하·스캔면 가림 계산. 인자 없음, geometry.json 기록. 주행·강도 계산 아님.

좌표 m(출력 mm), 차체 중심 원점, +x 전방. 그림자 각도는 센서 회전축 기준 원기둥 접선각이다.
시뮬 마스크는 simulation.launch.py와 같이 각 그림자 양끝에 0.5°를 더한다.
"""
import json
import math
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PROFILE = ROOT/'src/arena_description/config/b0183_c1_jr60p20_sim.yaml'
PREVIOUS = ROOT/'src/arena_description/config/b0183_c1_jr50_sim.yaml'
RUN_JR60P20 = HERE.parent/'lidar_jr_forward_v1/profiles/jr60p20.yaml'
VEHICLE = ROOT/'src/arena_description/config/vehicle.yaml'
POST_D, WIDTH, DEPTH, WALL = .007, .016, .012, .002
MARGIN = .5
MARKER_Z, MARKER_SIZE, DISPLAY_LENGTH = .140, .050, .06


def mm(v):
    return round(v*1000, 3)


def shadows(lidar, margin):
    sx, sy, _ = lidar['xyz_m']
    rows = []
    for kind in ('scan_plane_posts', 'scan_plane_wires'):
        for x, y, d in lidar.get(kind, []):
            r = math.hypot(x - sx, y - sy)
            rows.append(dict(kind=kind, center_deg=math.degrees(math.atan2(y - sy, x - sx)),
                             half_deg=math.degrees(math.asin(d/2/r)) + margin, range_m=r))
    return rows


def openings(rows):
    """한쪽(+y) 그림자 합집합으로 정면·정후방 연속 개방각과 전체 가림 합을 계산(좌우 대칭 배치)."""
    intervals = sorted([(r['center_deg'] - r['half_deg'], r['center_deg'] + r['half_deg'])
                        for r in rows if r['center_deg'] > 0])
    assert all(0 < lo < hi < 180 for lo, hi in intervals)
    union = []
    for lo, hi in intervals:
        if union and union[-1][1] >= lo:
            union[-1][1] = max(union[-1][1], hi)
        else:
            union.append([lo, hi])
    return dict(front_continuous_deg=round(2*union[0][0], 2), rear_continuous_deg=round(2*(180 - union[-1][1]), 2),
                blind_total_deg=round(2*sum(hi - lo for lo, hi in union), 2),
                one_side_union_deg=[[round(lo, 2), round(hi, 2)] for lo, hi in union])


def channels(lidar):
    sx, sy, _ = lidar['xyz_m']
    out = []
    rear_posts = [p for p in lidar['scan_plane_posts'] if p[0] < 0]
    for (px, py, _), (wx, wy, wd) in zip(rear_posts, lidar['scan_plane_wires']):
        r = math.hypot(px - sx, py - sy)
        u = ((px - sx)/r, (py - sy)/r)
        t = (-u[1], u[0])
        c = (px + (POST_D/2 + DEPTH/2)*u[0], py + (POST_D/2 + DEPTH/2)*u[1])
        corners = [(c[0] + a*DEPTH/2*u[0] + b*WIDTH/2*t[0], c[1] + a*DEPTH/2*u[1] + b*WIDTH/2*t[1])
                   for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        inner = [(c[0] + a*(DEPTH/2 - WALL)*u[0] + b*(WIDTH/2 - WALL)*t[0],
                  c[1] + a*(DEPTH/2 - WALL)*u[1] + b*(WIDTH/2 - WALL)*t[1])
                 for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        out.append(dict(
            post_mm=[mm(px), mm(py)], unit_sensor_to_post=[round(u[0], 6), round(u[1], 6)],
            exact_center_mm=[mm(c[0]), mm(c[1])], yaml_envelope_center_mm=[mm(wx), mm(wy)],
            yaml_rounding_offset_um=round(math.hypot(wx - c[0], wy - c[1])*1e6, 3),
            envelope_diameter_mm=mm(wd), outer_corners_mm=[[mm(x), mm(y)] for x, y in corners],
            nominal_inner_clear_mm=[mm(WIDTH - 2*WALL), mm(DEPTH - WALL)],
            inner_wall_corners_mm=[[mm(x), mm(y)] for x, y in inner],
            max_corner_distance_from_envelope_center_mm=mm(max(math.hypot(x - wx, y - wy) for x, y in corners)),
            inner_face_from_post_center_mm=mm(min((x - px)*u[0] + (y - py)*u[1] for x, y in corners)),
            rectangle_extent_mm=dict(x=[mm(min(x for x, _ in corners)), mm(max(x for x, _ in corners))],
                                     y=[mm(min(y for _, y in corners)), mm(max(y for _, y in corners))]),
            envelope_extent_mm=dict(x=[mm(wx - wd/2), mm(wx + wd/2)], y=[mm(wy - wd/2), mm(wy + wd/2)])))
    return out


def main():
    profile = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    lidar = profile['lidar']
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    drive = vehicle['drivetrain']
    radius, track, width, base = (float(drive[k]) for k in ('wheel_radius_m', 'track_width_m', 'wheel_width_m', 'wheelbase_m'))
    sz = lidar['xyz_m'][2]
    ch = channels(lidar)
    variants = {}
    for name, path in (('jr60p20_reserve_d20', PROFILE), ('jr60p20_thin_wire_run', RUN_JR60P20), ('jr50_previous', PREVIOUS)):
        lid = yaml.safe_load(path.read_text(encoding='utf-8'))['lidar']
        variants[name] = dict(physical=openings(shadows(lid, 0.)), mask_with_margin=openings(shadows(lid, MARGIN)))
    rows = shadows(lidar, MARGIN)
    marker = vehicle['identification_marker']['xyz_m']
    inner_wheel_y = track/2 - width/2
    checks = dict(
        envelope_covers_rectangle=all(c['max_corner_distance_from_envelope_center_mm'] <= c['envelope_diameter_mm']/2 + 1e-3 for c in ch),
        inner_face_tangent_to_post=all(abs(c['inner_face_from_post_center_mm'] - POST_D/2*1000) < 1e-3 for c in ch),
        inside_200x150=all(abs(v) <= 100 for c in ch for v in c['envelope_extent_mm']['x'])
                       and all(abs(v) <= 75 for c in ch for v in c['envelope_extent_mm']['y']),
        behind_sensor=all(c['envelope_extent_mm']['x'][1] < lidar['xyz_m'][0]*1000 for c in ch),
        rear_center_open=variants['jr60p20_reserve_d20']['mask_with_margin']['rear_continuous_deg'] > 0,
        reserve_shadow_covers_post_shadow=all(
            w['half_deg'] > p['half_deg'] and abs(w['center_deg'] - p['center_deg']) < .01
            for p, w in zip(rows[2:4], rows[4:6])),
        marker_bottom_above_reserve_display=MARKER_Z - MARKER_SIZE/2 > sz + DISPLAY_LENGTH/2,
        marker_plan_clear_of_reserve=min(min(abs(v) for v in c['envelope_extent_mm']['y'])
                                         for c in ch) > MARKER_SIZE/2*1000)
    result = dict(
        scope='평면 기하·광선 가림 계산. 실제 U채널 CAD·강도·하네스 실측·주행 성능 아님',
        profile=PROFILE.relative_to(ROOT).as_posix(),
        sensor_mm=[mm(v) for v in lidar['xyz_m']],
        posts_mm=[[mm(x), mm(y), mm(d)] for x, y, d in lidar['scan_plane_posts']],
        reserve_envelopes_mm=[[mm(x), mm(y), mm(d)] for x, y, d in lidar['scan_plane_wires']],
        channels=ch,
        mask_sectors=[dict(kind=r['kind'], center_deg=round(r['center_deg'], 3), half_deg=round(r['half_deg'], 3),
                           range_mm=mm(r['range_m'])) for r in rows],
        openings=variants,
        marker=dict(center_z_mm=mm(MARKER_Z), bottom_z_mm=mm(MARKER_Z - MARKER_SIZE/2), plan_xy_mm=[mm(marker[0]), mm(marker[1])],
                    half_width_mm=mm(MARKER_SIZE/2), scan_plane_z_mm=mm(sz), reserve_display_top_z_mm=mm(sz + DISPLAY_LENGTH/2),
                    default_center_z_mm=mm(marker[2])),
        hardware_flags=dict(
            rear_wheel_plan_y_mm=[mm(inner_wheel_y), mm(track/2 + width/2)], rear_wheel_top_z_mm=mm(2*radius),
            rear_axle_x_mm=mm(-base/2),
            channel_outer_max_abs_y_mm=max(abs(v) for c in ch for v in c['rectangle_extent_mm']['y']),
            note='채널 외곽 |y|가 뒤바퀴 안쪽면보다 크면 평면상 바퀴 위에 걸친다. 스캔면 75 mm는 바퀴 위끝 65 mm보다 10 mm 높다. '
                 '채널이 바퀴 위끝 아래로 내려가면 간섭 여부를 CAD에서 확인해야 한다.'),
        checks=checks)
    (HERE/'geometry.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')
    print(json.dumps(dict(checks=checks, openings=variants), ensure_ascii=False))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
