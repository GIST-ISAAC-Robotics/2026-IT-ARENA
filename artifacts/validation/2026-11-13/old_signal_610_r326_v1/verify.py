"""생성된 월드와 STL 선택의 구조·출처 검증. 주행 성능 검사가 아니다."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
report = json.loads((A/'fixture.json').read_text())
base = ET.parse(ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf').getroot()
new = ET.parse(A/'world_trial.sdf').getroot()
old_model = base.find("world/model[@name='it_arena_track_static']")
new_model = new.find("world/model[@name='it_arena_track_static']")


def normalized(element):
    clone = ET.fromstring(ET.tostring(element))
    for e in clone.iter():
        if e.tag in ('albedo_map', 'uri') and e.text:
            e.text = Path(e.text).name
        if e.text is not None:
            e.text = e.text.strip() or None
        e.tail = None
    return ET.tostring(clone)


preserved = [e.get('name') for e in old_model.findall('link') if e.get('name') != 'tl_beam']
checks = dict(
    selected_source_hash=report['source_sha256'] == '695621d5dcd3e0278e17c9f036ae5084975ac388897d7ffd1fe507fa4f95e957',
    original_files_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == v
                                 for p, v in report['original_sha256'].items()),
    duplicate_box_beam_removed=new_model.find("link[@name='tl_beam']") is None,
    other_track_links_unchanged=all(normalized(old_model.find(f"link[@name='{n}']")) ==
                                   normalized(new_model.find(f"link[@name='{n}']")) for n in preserved),
    same_three_provisional_positions=len(report['locations']) == 3,
    no_sampled_road_or_wall_overlap=all(s['road_overlap_m2'] < 1e-9 and s['wall_overlap_m2'] < 1e-9
                                       for p in report['locations'] for s in p['sections']),
    exact_support_triangle_count=report['support_triangles'] == 127,
    all_added_models_static=all(m.findtext('static') == 'true'
        for m in new.findall('world/model') if m.get('name', '').startswith('trial_old610_')),
    three_added_models=len([m for m in new.findall('world/model')
                            if m.get('name', '').startswith('trial_old610_')]) == 3,
)
result = dict(checks=checks, passed=all(checks.values()), preserved_track_links=len(preserved),
              scope='static structure and provenance; not physical/driving proof')
(A/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
assert result['passed']
