"""실행 전 시설 원본 보존·실행 사본 삽입·고저 및 배치 확인. 주행 시험 아님."""
import hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
from replay_tools import load_launch,WORLD
from surface_contact_eval import SurfaceMap,rectangle
A=Path(__file__).resolve().parent

def test_current_marker_gantry_fixture_is_grounded_and_clear_of_road():
    p=A/'marker_posts_provisional.sdf';meta=json.loads((A/'marker_posts_provisional.json').read_text())
    assert hashlib.sha256(p.read_bytes()).hexdigest()==meta['fixture_sha256']
    surface=SurfaceMap.from_world(WORLD)
    assert len(meta['links'])==8
    for row in meta['links']:
        poly=rectangle(*row['center'],row['yaw'],row['length'],row['width'])
        assert surface.point_class(*row['center'])=='grass'
        assert poly.intersection(surface.road).area<1e-10
        assert poly.intersection(surface.walls).area<1e-10
        if row['part']=='post':assert row['bottom']<=.075<=row['top']
        else:assert abs(row['bottom'])<1e-10 and row['top']==.01

def test_runtime_combines_posts_and_blank_markers_without_touching_original(tmp_path):
    before=WORLD.read_bytes();launch=load_launch();out=tmp_path/'world.sdf'
    report=launch._runtime_world_overrides(WORLD,out,'configured',True,False,
        evaluation_models=A/'marker_posts_provisional.sdf',evaluation_course_markers='blank_20_30',
        signal_posts='stl_61cm_provisional',track='official_v2026_11_06')
    root=ET.parse(out).getroot().find('world')
    for m in ET.parse(A/'marker_posts_provisional.sdf').getroot().findall('model'):
        actual=root.find(f"model[@name='{m.get('name')}']")
        assert ET.canonicalize(ET.tostring(actual,encoding='unicode'),strip_text=True)==ET.canonicalize(ET.tostring(m,encoding='unicode'),strip_text=True)
    original=ET.fromstring(before).find("world/model[@name='it_arena_track_static']")
    model=root.find("model[@name='it_arena_track_static']")
    for link in original.findall('link'):
        name=link.get('name');actual=model.find(f"link[@name='{name}']")
        assert actual is not None
        # 마커 무늬는 평가 목적의 가림. 원래 자세와 충돌체는 모든 링크에서 같다.
        for tag in ('pose','collision'):
            a,b=link.find(tag),actual.find(tag)
            assert (a is None)==(b is None)
            if a is not None:assert ET.canonicalize(ET.tostring(a,encoding='unicode'),strip_text=True)==ET.canonicalize(ET.tostring(b,encoding='unicode'),strip_text=True)
    assert report['hidden_ids']==[20,30]
    assert WORLD.read_bytes()==before
