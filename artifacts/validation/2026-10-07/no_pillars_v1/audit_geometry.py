"""보존한 기둥 포함 월드와 새 기본 파생본의 정확한 변경 범위를 검사합니다."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
DEST = ROOT / 'src/arena_gazebo/worlds/it_arena_official'


def canonical(node):
    return ET.canonicalize(ET.tostring(node, encoding='unicode'), strip_text=True)


with ZipFile(HERE / 'before_supports.zip') as archive:
    old = ET.fromstring(archive.read('it_arena_official/world.sdf'))
    new = ET.parse(DEST / 'world.sdf').getroot()
    model = old.find("./world/model[@name='it_arena_track_static']")
    removed_names = ['tl_post', *[f'team_gantry_support_{i}' for i in (0, 20, 30, 45)]]
    removed = []
    for name in removed_names:
        link = model.find(f"link[@name='{name}']")
        assert link is not None, name
        removed.append({'name': name, 'collisions': len(link.findall('collision')), 'visuals': len(link.findall('visual'))})
        model.remove(link)
    assert canonical(old) == canonical(new), 'world changed beyond the registered five links'
    unchanged = []
    for member in archive.namelist():
        if not member.startswith('it_arena_official/') or member.endswith('/'):
            continue
        relative = member.removeprefix('it_arena_official/')
        if relative in ('world.sdf', 'scene.json', 'provenance.json'):
            continue
        assert archive.read(member) == (DEST / relative).read_bytes(), relative
        unchanged.append(relative)
    source = json.loads(archive.read('it_arena_official/scene.json'))
    scene = json.loads((DEST / 'scene.json').read_text())
    for key in ('track', 'branches', 'aruco_markers', 'starting_grid', 'start_finish'):
        assert source[key] == scene[key], key
result = {
    'passed': True, 'removed_links': removed,
    'removed_collisions': sum(row['collisions'] for row in removed),
    'world_other_content_canonically_identical': True,
    'unchanged_resources': unchanged,
    'before_zip_sha256': hashlib.sha256((HERE / 'before_supports.zip').read_bytes()).hexdigest(),
    'after_world_sha256': hashlib.sha256((DEST / 'world.sdf').read_bytes()).hexdigest(),
}
with (HERE / 'geometry_audit.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
print(json.dumps(result, ensure_ascii=False, indent=2))
