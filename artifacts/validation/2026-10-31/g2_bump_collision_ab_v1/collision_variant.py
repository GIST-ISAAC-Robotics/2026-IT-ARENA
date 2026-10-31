"""평탄 진단 월드의 방지턱 충돌 20조각만 제외한다. 시각/그 외 물성은 보존."""
from copy import deepcopy
import hashlib
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def shape(node):
    return node.tag, sorted(node.attrib.items()), (node.text or '').strip(), [shape(c) for c in node]


def remove_bump_collisions(root):
    links=root.findall(".//model/link[@name='safety_bump_0']")
    if len(links)!=1:raise ValueError('exactly one registered bump required')
    bump=links[0];collisions=bump.findall('collision')
    if len(collisions)!=20:raise ValueError('registered 20-strip bump required')
    if not bump.findall('visual'):raise ValueError('bump visual missing')
    for c in collisions:
        if c.find('geometry/box/size') is None:raise ValueError('unexpected bump shape')
        bump.remove(c)
    return len(collisions)


def collision_contract(actual_path, baseline_path):
    actual=ET.parse(actual_path).getroot(); baseline=ET.parse(baseline_path).getroot()
    expected=deepcopy(baseline);count=remove_bump_collisions(expected)
    def collision_model(root):
        model=root.find(".//model[@name='it_arena_track_static']")
        if model is None:return None
        return (model.findtext('pose'),[(link.get('name'),link.findtext('pose'),[shape(c) for c in link.findall('collision')]) for link in model.findall('link')])
    return dict(registered_bump_removed=count==20,
        static_track_collision_contract=collision_model(actual)==collision_model(expected),
        bump_visual_present=bool(actual.findall(".//link[@name='safety_bump_0']/visual")),
        grass_level_unchanged=actual.findtext(".//link[@name='grass']/pose")=='0 0 0.001 0 0 0')


def build_fixture(source,target):
    source,target=Path(source),Path(target)
    if target.exists():raise FileExistsError(target)
    before=(source/'world.sdf').read_bytes();original=ET.fromstring(before);variant=deepcopy(original)
    count=remove_bump_collisions(variant)
    # Reconstruct the expected tree independently of serialization and compare all nodes.
    check=deepcopy(original);link=check.find(".//link[@name='safety_bump_0']")
    for c in list(link):
        if c.tag=='collision':link.remove(c)
    assert shape(check)==shape(variant)
    shutil.copytree(source,target)
    ET.ElementTree(variant).write(target/'world.sdf',encoding='utf-8',xml_declaration=True)
    assert (source/'world.sdf').read_bytes()==before
    others=[p for p in source.rglob('*') if p.is_file() and p.name!='world.sdf']
    assert all(p.read_bytes()==(target/p.relative_to(source)).read_bytes() for p in others)
    assert all(collision_contract(target/'world.sdf',source/'world.sdf').values())
    return dict(removed_collision_strips=count,visual_preserved=True,other_fixture_files_unchanged=len(others),
        baseline_world_sha256=hashlib.sha256(before).hexdigest(),
        variant_world_sha256=hashlib.sha256((target/'world.sdf').read_bytes()).hexdigest(),
        only_bump_collisions_removed=True,diagnostic_not_official_or_default_world=True)
