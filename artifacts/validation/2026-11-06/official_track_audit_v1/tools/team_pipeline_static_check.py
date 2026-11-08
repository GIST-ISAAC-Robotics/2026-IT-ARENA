#!/usr/bin/env python3
"""현재 팀 빌드·평가 함수가 v2026.11.06 원본을 그대로 받을 수 있는지 읽기 전용으로 대조한다.

WSL: python3 -I tools/team_pipeline_static_check.py
- 공용 scripts/ 모듈을 import만 하고 수정하지 않는다. 배포 track_gen.py(생성기)는 실행하지 않는다.
- 실행 월드(src/arena_gazebo/worlds/it_arena_official)와 공식 원본 ZIP은 읽기만 한다.
"""
from __future__ import annotations

import hashlib
import json
import math
import sys
import traceback
from pathlib import Path

ART = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[5]
SRC = ART / 'extracted/output_final'
ZIP = ROOT / 'assets/track/official/v2026.11.06/it_arena_track_v2026.11.06.zip'
sys.path.insert(0, str(ROOT / 'scripts'))
sys.dont_write_bytecode = True


def attempt(fn):
    try:
        return {'ok': True, 'value': fn()}
    except Exception as exc:  # 기대 실패도 그대로 기록한다.
        return {'ok': False, 'error': f'{type(exc).__name__}: {exc}',
                'where': traceback.extract_tb(exc.__traceback__)[-1].name}


def main():
    import numpy as np
    from shapely.ops import unary_union
    import build_official_track as bot
    from build_experimental_track import obstacle_polygons, rectangle
    from audit_local_pursuit_road import road_polygon
    from official_2x5_facilities import lamp_channels, SIGNAL_STRUCTURE
    import xml.etree.ElementTree as ET

    scene = json.loads((SRC / 'scene.json').read_text(encoding='utf-8'))
    res = {'scope': '정적 연결 감사. 팀 공용 코드 무수정, 배포 생성기 미실행.', 'checks': {}}
    c = res['checks']
    c['builder_release_registry'] = {'registered': sorted(bot.RELEASES), 'has_v2026.11.06': 'v2026.11.06' in bot.RELEASES,
                                     'expected_member_count': bot.EXPECTED_MEMBER_COUNT}
    c['builder_verify_archive_new_zip'] = attempt(lambda: bot.verify_archive(ZIP))
    hypothetical = {'upstream_release': 'v2026.11.06', 'profile': 'official_v2026.11.06',
                    'release_asset': 'it_arena_track_v2026.11.06.zip'}
    c['builder_profile_archive_hypothetical_profile'] = attempt(lambda: str(bot.profile_archive(hypothetical)))
    src_text = (ROOT / 'scripts/build_official_track.py').read_text(encoding='utf-8')
    c['builder_hardcoded_main_width_045'] = {
        'prepare_raises_unless_track_w_eq_045': "float(result[\"meta\"][\"track_w\"]) != .45" in src_text,
        'provenance_main_width_045': '"main_width_m": .45' in src_text,
        'default_archive_constant': str(bot.ARCHIVE.relative_to(ROOT))}
    c['builder_verify_official_markers_on_new_world'] = attempt(
        lambda: [{k: m[k] for k in ('id', 'official_sdf_board_center_pose')} for m in
                 bot.verify_official_markers(SRC / 'world.sdf', scene, 'v2026.11.06')])
    # 신호 링크 이름(팀 2×5 점등 연결)
    model = ET.parse(SRC / 'world.sdf').find("./world/model[@name='it_arena_track_static']")
    names = {l.attrib['name'] for l in model.findall('link')}
    lamp_info = []
    for ch in lamp_channels():
        link = model.find(f"link[@name='{ch['name']}']")
        emissive = link.findtext(f"visual[@name='{ch['name']}_vis']/material/emissive") if link is not None else None
        lamp_info.append({'name': ch['name'], 'present': link is not None, 'emissive': emissive,
                          'matches_team_lit_rgba': emissive == ch['lit_rgba']})
    c['team_2x5_lamp_channels'] = {'all_present_and_colors_match': all(x['present'] and x['matches_team_lit_rgba'] for x in lamp_info),
                                   'details': lamp_info,
                                   'signal_structure_present': {n: n in names for n in SIGNAL_STRUCTURE}}
    # 평가기용 도로·장애물 다각형(팀 함수)
    road = road_polygon(SRC / 'world.sdf')
    obstacles = obstacle_polygons(SRC / 'world.sdf')
    raw_parts = len(getattr(road, 'geoms', [road]))
    c['evaluator_road_polygon'] = {'area_m2': round(road.area, 5), 'parts_without_merge': raw_parts,
                                   'note': 'P21 사선 5 mm 계단 조각이 반올림으로 미세 분리되어 다중 다각형이 된다(면적은 동일).'}
    c['evaluator_obstacle_polygons'] = {'count': len(obstacles), 'area_m2': round(unary_union(obstacles).area, 5)}
    # 팀 inspect_geometry의 핵심(직진 자세 차체 외형 vs 벽)을 CSV 경로 표본으로 재현
    import csv
    from shapely.strtree import STRtree
    tree = STRtree(obstacles)

    def route(name):
        rows = list(csv.DictReader(open(SRC / name, encoding='utf-8')))
        pts = np.array([[float(r['x_m']), float(r['y_m'])] for r in rows])
        return pts

    def blocked(poly):
        return any(poly.intersection(obstacles[int(i)]).area > 1e-9 for i in tree.query(poly))

    from shapely.geometry import Point
    route_checks = {}
    for name in ('centerline.csv', 'branch_0.csv', 'branch_1.csv'):
        pts = route(name)
        closed = name == 'centerline.csv'
        bad_body, bad_steer, off_road = [], [], []
        for i in range(len(pts)):
            j = (i + 1) % len(pts) if closed else min(i + 1, len(pts) - 1)
            k = i if j != i else i - 1
            yaw = math.atan2(pts[j][1] - pts[k][1], pts[j][0] - pts[k][0]) if j != i else math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0])
            if blocked(rectangle(pts[i][0], pts[i][1], yaw, .20, .15)):
                bad_body.append(i)
            if blocked(rectangle(pts[i][0], pts[i][1], yaw, .2011, .1741)):
                bad_steer.append(i)
        route_checks[name] = {'samples': len(pts), 'body_20x15_blocked': len(bad_body), 'steered_envelope_blocked': len(bad_steer)}
    c['route_footprint_vs_walls'] = route_checks
    # 현재 실행 월드·제어 상수의 구 코스 가정
    runtime = ROOT / 'src/arena_gazebo/worlds/it_arena_official'
    prov = json.loads((runtime / 'provenance.json').read_text(encoding='utf-8'))
    rscene = json.loads((runtime / 'scene.json').read_text(encoding='utf-8'))
    c['current_runtime_world'] = {'upstream_release': prov['upstream']['release'], 'lap_length_m': rscene['track']['lap_length_m'],
                                  'width_m': rscene['track']['width_m'],
                                  'world_sha256': hashlib.sha256((runtime / 'world.sdf').read_bytes()).hexdigest()}
    li = (ROOT / 'src/arena_autonomy/arena_autonomy/lateral_intent.py').read_text(encoding='utf-8')
    lp = (ROOT / 'src/arena_autonomy/arena_autonomy/local_path.py').read_text(encoding='utf-8')
    wf = (ROOT / 'src/arena_autonomy/arena_autonomy/wall_follow.py').read_text(encoding='utf-8')
    pursuit = (ROOT / 'src/arena_autonomy/arena_autonomy/local_pursuit.py').read_text(encoding='utf-8')
    new_center_offset = scene['track']['width_m'] / 2 + scene['grass']['width_each_side_m']
    c['controller_spec_constants'] = {
        'lateral_intent_road_half_m_0225': 'road_half_m: float = .225' in li,
        'local_path_default_offset_0425': 'offset=.425' in lp,
        'wall_follow_target_wall_distance_0425': '"target_wall_distance_m": .425' in wf,
        'new_wall_to_centerline_m': round(new_center_offset, 3),
        'old_wall_to_centerline_m': 0.425,
        'marker_side_rule_present': "if 30 in self.ids:" in pursuit and "for marker in (0, 20, 45)" in pursuit}
    res['all_attempts'] = {k: v.get('ok') for k, v in c.items() if isinstance(v, dict) and 'ok' in v}
    (ART / 'checks/team_pipeline_static_check.json').write_text(json.dumps(res, ensure_ascii=False, indent=1, default=str) + '\n',
                                                               encoding='utf-8', newline='\n')
    print(json.dumps(res, ensure_ascii=False, indent=1, default=str)[:6000])


if __name__ == '__main__':
    main()
