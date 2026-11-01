"""보존 ZIP·SDF의 정적 대조. Gazebo 실행·시야·검출 성능 검증이 아니다."""
from pathlib import Path
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent


def archive(tag):
    path = ROOT / f'assets/track/official/{tag}/it_arena_track_{tag}.zip'
    with zipfile.ZipFile(path) as z:
        return {n: z.read(n) for n in z.namelist() if not n.endswith('/')}


def canon(e):
    return (e.tag, tuple(sorted(e.attrib.items())), (e.text or '').strip(), tuple(canon(c) for c in e))


def pose(e):
    return [float(x) for x in e.findtext('pose', '0 0 0 0 0 0').split()]


def main():
    old, new = archive('v2026.10.24'), archive('v2026.11.01')
    common = set(old) & set(new)
    same = sorted(n for n in common if old[n] == new[n])
    changed = sorted(n for n in common if old[n] != new[n])
    models = []
    for data in (old, new):
        tree = ET.fromstring(data['output_final/world.sdf'])
        models.append({e.get('name'): e for e in tree.findall('.//model/link')})
    a, b = models
    changed_links = sorted(n for n in a.keys() & b.keys() if canon(a[n]) != canon(b[n]))
    surface_links = ('ground_plane_link', 'track_surface', 'grass', 'walls')
    assert all(n in a and n in b for n in surface_links), '도로/잔디/벽 비교 대상 누락'
    scene = json.loads(new['output_final/scene.json'])
    tl = scene['traffic_light']; heading = tl['pose']['yaw_rad']
    forward = (math.cos(heading), math.sin(heading)); left = (-forward[1], forward[0])
    lamps = {n: e for n, e in b.items() if n.startswith('lamp_')}
    lamp_points = [pose(e)[:3] for e in lamps.values()]
    panel = b['tl_panel']; pp = pose(panel)
    dims = [float(x) for x in panel.findtext('visual/geometry/box/size').split()]
    normal = [math.cos(pp[4])*math.cos(pp[5]), math.cos(pp[4])*math.sin(pp[5]), -math.sin(pp[4])]
    markers = {}
    for n in (0, 45):
        p = pose(b[f'aruco_{n}'])
        d = [p[0]-tl['pose']['x'], p[1]-tl['pose']['y']]
        markers[str(n)] = dict(pose=p, forward_offset_m=sum(x*y for x,y in zip(d,forward)),
                               left_offset_m=sum(x*y for x,y in zip(d,left)))
    team = ET.parse(ROOT/'artifacts/validation/2026-11-01/signal_marker_offset_v1/fixture/world.sdf')
    team_lamps = [e for e in team.findall('.//link') if 'lamp_' in e.get('name','')]
    out = dict(scope='정적 파일/기하 비교만 수행. 실행 월드·제어기 미변경, Gazebo와 검출 시험 미실시.',
               same_zip_files=same, changed_zip_files=changed,
               added_zip_files=sorted(set(new)-set(old)), removed_zip_files=sorted(set(old)-set(new)),
               changed_sdf_links=changed_links, removed_sdf_links=sorted(a.keys()-b.keys()), added_sdf_links=sorted(b.keys()-a.keys()),
               compared_surface_links=list(surface_links),
               road_grass_wall_unchanged=all(canon(a[n])==canon(b[n]) for n in surface_links),
               marker20_30_unchanged=all(canon(a[f'aruco_{n}'])==canon(b[f'aruco_{n}']) for n in (20,30)),
               speed_bumps=scene['speed_bumps'], signal_metadata=tl,
               grid_to_signal_s_m=tl['s_m']-max(s['s_m'] for s in scene['starting_grid']['slots'])-scene['starting_grid']['slot_length_m']/2,
               panel=dict(pose=pp,size_m=dims,normal=normal,normal_dot_forward=sum(x*y for x,y in zip(normal,forward)),
                          modeled_bottom_m=pp[2]-dims[2]/2*abs(math.cos(pp[4]))-dims[0]/2*abs(math.sin(pp[4]))),
               lamp_count=len(lamps),lamp_shapes=sorted({list(e.find('visual/geometry'))[0].tag for e in lamps.values()}),
               lamp_diameters_m=sorted({2*float(e.findtext('visual/geometry/sphere/radius')) for e in lamps.values()}),
               lamp_mean_position_m=[sum(p[i] for p in lamp_points)/len(lamp_points) for i in range(3)],
               marker_poses=markers,old_team_lamp_shapes=sorted({list(v.find('geometry'))[0].tag for e in team_lamps for v in e.findall('visual')}),
               main_and_release_world_identical=(OUT/'upstream/track/output_final/world.sdf').read_bytes()==new['output_final/world.sdf'],
               main_and_release_scene_identical=(OUT/'upstream/track/output_final/scene.json').read_bytes()==new['output_final/scene.json'])
    (OUT/'model_audit.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(out,ensure_ascii=False,indent=1))


if __name__ == '__main__':
    main()
