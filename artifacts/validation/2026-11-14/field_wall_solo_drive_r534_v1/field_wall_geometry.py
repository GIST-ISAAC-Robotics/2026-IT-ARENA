"""r520 형상을 유지하는 r530 자원 경로 보정본의 검증 API. 새 기하 생성 없음."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from world_resources import verify_resources,ORIGINAL as SOURCE,ORIGINAL_SHA as SOURCE_SHA,OUTPUT as DERIVED

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def canonical(e):return ET.canonicalize(ET.tostring(e,encoding='unicode'),strip_text=True)
def walls_element(path):return ET.parse(path).getroot().find("world/model[@name='it_arena_track_static']/link[@name='walls']")
def collision_digest(path):return hashlib.sha256('\n'.join(canonical(c) for c in walls_element(path).findall('collision')).encode()).hexdigest()
def verify_derived():
    info=read(A/'field_geometry_audit.json')
    assert verify_resources()['passed']
    assert sha(SOURCE)==info['source_sha256']==SOURCE_SHA
    assert sha(DERIVED)==info['derived_sha256']
    assert collision_digest(DERIVED)==info['walls_collision_sha256']
    assert len(walls_element(DERIVED).findall('collision'))==475
    assert info['path_only_update_from_r520'] and info['static_geometry_passed']
    return info
