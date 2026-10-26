"""이번 갱신의 충돌 형상·공식 마커·원본 보존 대조. 평가 전용."""
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
RUNTIME = ROOT / 'src/arena_gazebo/worlds/it_arena_official'

def digest(data):
    return hashlib.sha256(data).hexdigest()

def model(data):
    return ET.fromstring(data).find("./world/model[@name='it_arena_track_static']")

def collisions(tree):
    return {f"{link.attrib['name']}/{c.attrib['name']}":
            [link.findtext('pose'), ET.canonicalize(ET.tostring(c, encoding='unicode'), strip_text=True)]
            for link in tree.findall('link') for c in link.findall('collision')}

with ZipFile(OUT / 'previous_runtime.zip') as archive:
    names = [n for n in archive.namelist() if n.endswith('world.sdf')]
    assert len(names) == 1
    previous = model(archive.read(names[0]))
current = model((RUNTIME / 'world.sdf').read_bytes())
before, after = collisions(previous), collisions(current)
diff = sorted(k for k in set(before) | set(after) if before.get(k) != after.get(k))
source = ROOT / 'assets/track/official'
with ZipFile(source / 'v2026.09.14/it_arena_track_v2026.09.14.zip') as a, ZipFile(source / 'v2026.10.24/it_arena_track_v2026.10.24.zip') as b:
    left, right = model(a.read('output_final/world.sdf')), model(b.read('output_final/world.sdf'))
    common = sorted(set(a.namelist()) & set(b.namelist()))
    changed = [n for n in common if a.read(n) != b.read(n)]
    marker_same = all(ET.canonicalize(ET.tostring(left.find(f"link[@name='aruco_{i}']"), encoding='unicode'), strip_text=True) ==
                      ET.canonicalize(ET.tostring(right.find(f"link[@name='aruco_{i}']"), encoding='unicode'), strip_text=True)
                      and a.read(f'output_final/aruco/aruco_id{i}.png') == b.read(f'output_final/aruco/aruco_id{i}.png')
                      for i in (0, 20, 30, 45))
    official_collisions_same = collisions(left) == collisions(right)
    geometry_same = all(a.read(n) == b.read(n) for n in ('design_final.json', 'output_final/centerline.csv')) if 'output_final/centerline.csv' in common else a.read('design_final.json') == b.read('design_final.json')
result = dict(runtime_collision_count=len(after), runtime_collision_differences=diff,
              old_to_new_official_changed_files=changed,
              official_collision_map_identical=official_collisions_same,
              official_design_identical=geometry_same, official_marker_links_and_png_identical=marker_same,
              passed=not diff and marker_same and geometry_same)
(OUT / 'geometry_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(result, ensure_ascii=False))
raise SystemExit(0 if result['passed'] else 1)
