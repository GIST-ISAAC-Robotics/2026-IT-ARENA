"""공식 v2026.11.06 world.sdf와 팀 실행 월드의 차이를 정규화 XML로 전수 대조한다.

공식 출력(ZIP output_final)에 팀 시설 변경 없이 같은 실행 변환(build_runtime_world)만 적용한 기준을 만들고,
실행 월드와 링크/시각/충돌 단위로 비교한다. 허용 차이: tl_post 제거, 광원 10개 재질(소등), finish_line_paint 추가.
원본 ZIP·실행 월드는 읽기만 한다.
"""
import hashlib
import json
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
from build_runtime_world import build_runtime_world  # noqa: E402

ZIP = ROOT / 'assets/track/official/v2026.11.06/it_arena_track_v2026.11.06.zip'
RUNTIME = ROOT / 'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
LAMPS = [f"lamp_{p}_{r}" for p in ('red_1', 'red_2', 'red_3', 'red_4', 'green') for r in ('top', 'bottom')]
QUERY = "./world/model[@name='it_arena_track_static']"


def canon(element):
    return ET.canonicalize(ET.tostring(element, encoding='unicode'), strip_text=True)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def children(model):
    return {(c.tag, c.get('name')): c for c in model}


def main():
    report = {'zip_sha256': sha(ZIP.read_bytes()), 'runtime': RUNTIME.relative_to(ROOT).as_posix()}
    scratch_root = ROOT / 'build/official_track_generation'
    scratch_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='audit_1106_', dir=scratch_root) as tmp:
        tmp = Path(tmp)
        with zipfile.ZipFile(ZIP) as archive:
            archive.extractall(tmp / 'release')
        source = tmp / 'release/output_final'
        official = ET.parse(source / 'world.sdf')
        build_runtime_world(source, tmp / 'reference')
        reference = ET.parse(tmp / 'reference/world.sdf')
        source_png = {p.name: sha(p.read_bytes()) for p in sorted((source / 'aruco').glob('*.png'))}
        source_files = {name: sha((source / name).read_bytes()) for name in
                        ('centerline.csv', 'left_boundary.csv', 'right_boundary.csv', 'branch_0.csv', 'branch_1.csv',
                         'map.png', 'map.yaml', 'map_with_grass.png', 'map_with_grass.yaml', 'preview.png')}
        source_scene = json.loads((source / 'scene.json').read_text(encoding='utf-8'))
    runtime = ET.parse(RUNTIME / 'world.sdf')

    # 월드 수준(물리·조명·플러그인 등)은 모델 외 전부 같아야 한다.
    ref_world, run_world = reference.getroot().find('world'), runtime.getroot().find('world')
    report['world_name'] = run_world.get('name')
    report['world_level_equal'] = [canon(c) for c in ref_world if c.tag != 'model'] == \
        [canon(c) for c in run_world if c.tag != 'model']
    report['world_models'] = [m.get('name') for m in run_world.findall('model')]

    ref_children, run_children = children(reference.find(QUERY)), children(runtime.find(QUERY))
    report['removed'] = sorted(f'{t}:{n}' for t, n in set(ref_children) - set(run_children))
    report['added'] = sorted(f'{t}:{n}' for t, n in set(run_children) - set(ref_children))
    changed = {}
    for key in sorted(set(ref_children) & set(run_children), key=str):
        a, b = ref_children[key], run_children[key]
        if canon(a) == canon(b):
            continue
        detail = []
        sub_a, sub_b = children(a), children(b)
        for sub in sorted(set(sub_a) | set(sub_b), key=str):
            if sub not in sub_a or sub not in sub_b:
                detail.append(f'{sub} presence differs')
            elif canon(sub_a[sub]) != canon(sub_b[sub]):
                inner_a, inner_b = children(sub_a[sub]), children(sub_b[sub])
                diff_tags = sorted(f'{t}' for (t, n) in set(inner_a) | set(inner_b)
                                   if (t, n) not in inner_a or (t, n) not in inner_b
                                   or canon(inner_a[(t, n)]) != canon(inner_b[(t, n)]))
                detail.append({'element': f'{sub[0]}:{sub[1]}', 'differing_children': diff_tags})
        changed[f'{key[0]}:{key[1]}'] = detail
    report['changed'] = changed

    # 광원 재질 상세(초기 소등)와 형상·pose 보존.
    lamps = {}
    for name in LAMPS:
        o = official.find(f"{QUERY}/link[@name='{name}']")
        r = runtime.find(f"{QUERY}/link[@name='{name}']")
        mo, mr = o.find(f"visual[@name='{name}_vis']/material"), r.find(f"visual[@name='{name}_vis']/material")
        lamps[name] = {
            'pose_equal': o.findtext('pose') == r.findtext('pose'),
            'geometry_equal': canon(o.find(f"visual[@name='{name}_vis']/geometry")) == canon(r.find(f"visual[@name='{name}_vis']/geometry")),
            'official_emissive': mo.findtext('emissive'), 'runtime_emissive': mr.findtext('emissive'),
            'runtime_diffuse': mr.findtext('diffuse'),
            'collision_count_official': len(o.findall('collision')), 'collision_count_runtime': len(r.findall('collision')),
        }
    report['lamps'] = lamps
    report['tl_post_official'] = official.find(f"{QUERY}/link[@name='tl_post']") is not None
    report['tl_post_runtime'] = runtime.find(f"{QUERY}/link[@name='tl_post']") is not None
    report['signal_structure_equal'] = {n: canon(official.find(f"{QUERY}/link[@name='{n}']")) ==
                                        canon(runtime.find(f"{QUERY}/link[@name='{n}']")) for n in ('tl_beam', 'tl_panel')}
    report['collisions'] = {'official_raw': len(official.findall('.//collision')),
                            'reference_runtime_transform': len(reference.findall('.//collision')),
                            'team_runtime': len(runtime.findall('.//collision'))}
    report['markers'] = {}
    for marker in (0, 20, 30, 45):
        o = official.find(f"{QUERY}/link[@name='aruco_{marker}']")
        r = runtime.find(f"{QUERY}/link[@name='aruco_{marker}']")
        report['markers'][marker] = {
            'official_pose': o.findtext('pose'), 'runtime_pose': r.findtext('pose'),
            'link_equal_after_runtime_transform': canon(ref_children[('link', f'aruco_{marker}')]) == canon(r),
            'png_equal': source_png[f'aruco_id{marker}.png'] == sha((RUNTIME / f'aruco/aruco_id{marker}.png').read_bytes()),
        }
    report['data_files_equal_to_official'] = {name: digest == sha((RUNTIME / name).read_bytes())
                                              for name, digest in source_files.items()}
    scene = json.loads((RUNTIME / 'scene.json').read_text(encoding='utf-8'))
    report['scene'] = {
        'aruco_markers_equal': scene['aruco_markers'] == source_scene['aruco_markers'],
        'grid_slots_equal': scene['starting_grid']['slots'] == source_scene['starting_grid']['slots'],
        'track_equal': scene['track'] == source_scene['track'],
        'branches_equal': scene['branches'] == source_scene['branches'],
        'traffic_light': {k: scene['traffic_light'].get(k) for k in
                          ('initial_state', 'state_control', 'track_post_removed', 'track_post_basis',
                           'udp_visual_controller_connected')},
    }
    allowed_changed = {f'link:{n}' for n in LAMPS}
    report['passed'] = bool(
        report['world_level_equal'] and report['removed'] == ['link:tl_post'] and report['added'] == ['link:finish_line_paint']
        and set(changed) <= allowed_changed
        and all(v['pose_equal'] and v['geometry_equal'] and v['runtime_emissive'] == '0 0 0 1' for v in lamps.values())
        and report['tl_post_official'] and not report['tl_post_runtime'] and all(report['signal_structure_equal'].values())
        and all(m['link_equal_after_runtime_transform'] and m['png_equal'] for m in report['markers'].values())
        and all(report['data_files_equal_to_official'].values())
        and all(v for k, v in report['scene'].items() if k != 'traffic_light')
        and report['scene']['traffic_light']['initial_state'] == 'off')
    out = HERE / 'checks/world_diff_audit.json'
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('passed', 'removed', 'added', 'world_level_equal', 'collisions')},
                     ensure_ascii=False))
    print('changed:', json.dumps(changed, ensure_ascii=False)[:1500])
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
