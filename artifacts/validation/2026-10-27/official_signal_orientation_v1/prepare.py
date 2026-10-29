"""공식 원본 보존, 가로 배열·연결된 지지 구조의 팀 정지 시험본을 만든다."""
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_start_signal_geometry import lamps_from_sdf, project


def set_text(parent,tag,value):
    node=parent.find(tag)
    if node is None:node=ET.SubElement(parent,tag)
    node.text=value


def corrected_fixture(stage):
    old=ROOT/f'artifacts/validation/2026-10-27/camera_mount_preparation_v1/render_h25_{stage}'
    root=ET.parse(old/'fixture.sdf').getroot();spec=json.loads((old/'report.json').read_text())
    model=root.find("world/model[@name='it_arena_track_static']")
    source=ROOT/'assets/track/official/v2026.10.24/it_arena_track_v2026.10.24.zip'
    with zipfile.ZipFile(source) as z:
        scene=json.loads(z.read(next(n for n in z.namelist() if n.endswith('scene.json'))))
    center=scene['traffic_light']['pose'];yaw=center['yaw_rad']
    forward=np.array([math.cos(yaw),math.sin(yaw),0.])
    right=np.array([math.sin(yaw),-math.cos(yaw),0.])
    origin=np.array([center['x'],center['y'],1.2])
    expected={'tl_beam',*(f'lamp_red_{i}' for i in range(1,5)),'lamp_green'}
    before={l.get('name'):ET.tostring(l) for l in model.findall('link')}
    # 기둥은 왼쪽에 그대로 두고 가로보의 장축만 도로 횡방향으로 돌린다.
    beam=model.find("link[@name='tl_beam']")
    for part in ('visual','collision'):
        child=beam.find(part)
        values=list(map(float,child.findtext('pose').split()))
        values[5]=yaw-math.pi/2
        child.find('pose').text=' '.join(f'{v:.10f}' for v in values)
    # 기존 광원 15mm/30mm 피치/1.08m 높이는 유지, 출발차를 향한 판 앞에 배치.
    panel_center=origin+np.array([0.,0.,-.075])
    for index,name in enumerate(('lamp_red_1','lamp_red_2','lamp_red_3','lamp_red_4','lamp_green')):
        xyz=origin+right*((index-2)*.03)-forward*.014+np.array([0.,0.,-.12])
        model.find(f"link[@name='{name}']/pose").text=' '.join(f'{v:.10f}' for v in (*xyz,0.,0.,0.))
    panel=ET.SubElement(model,'link',name='team_signal_panel')
    for part in ('visual','collision'):
        child=ET.SubElement(panel,part,name='team_signal_panel_'+part)
        set_text(child,'pose',' '.join(f'{v:.10f}' for v in (*panel_center,0.,0.,yaw-math.pi/2)))
        box=ET.SubElement(ET.SubElement(child,'geometry'),'box');set_text(box,'size','.16 .012 .15')
        if part=='visual':
            mat=ET.SubElement(child,'material')
            set_text(mat,'ambient','.06 .06 .06 1');set_text(mat,'diffuse','.06 .06 .06 1')
    changed=[l.get('name') for l in model.findall('link') if l.get('name') in before and ET.tostring(l)!=before[l.get('name')]]
    assert set(changed)==expected
    assert {l.get('name') for l in model.findall('link')}-set(before)=={'team_signal_panel'}
    # 기둥 상단은 가로보 범위 안이고 판 상단은 가로보에 연결된다.
    post=np.array(list(map(float,model.find("link[@name='tl_post']/visual/pose").text.split()))[:3])
    local=np.array([np.dot(post-origin,right),np.dot(post-origin,forward)])
    assert abs(local[0])<.375 and abs(local[1])<.03
    assert 1.2-.03<=panel_center[2]+.075<=1.2+.03
    raw=ET.tostring(root);lamps=lamps_from_sdf(raw)
    projections={}
    for name,v in spec['render_views'].items():
        if name=='overview':continue
        camera=v['camera'];projections[name]=[project(lamps,slot,camera) for slot in scene['starting_grid']['slots']]
        assert all(p['red_to_green_left_to_right'] for p in projections[name])
    spec.update(scope='team_horizontal_signal_fixture_official_color_order_corrected_geometry_static_no_drive',
        changed_links=sorted(changed),added_links=['team_signal_panel'],
        orientation_basis='issue3 manufacturer: left to right red red red red green',
        retained_dimensions='official provisional lamp z1.08m, beam z1.20m, 15mm sphere, 30mm pitch',
        support_intersection_check=True,
        height_status='10/6 minutes say planned 40cm; its reference point and actual installation remain unresolved',
        sources={str(source.relative_to(ROOT)):hashlib.sha256(source.read_bytes()).hexdigest(),
                 str((old/'fixture.sdf').relative_to(ROOT)):hashlib.sha256((old/'fixture.sdf').read_bytes()).hexdigest(),
                 str(Path(__file__).relative_to(ROOT)):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
    return root,spec,projections


if __name__=='__main__':
    for stage in ('red4','green'):
        root,spec,projections=corrected_fixture(stage)
        target=OUT/('horizontal_'+stage);target.mkdir(exist_ok=False)
        ET.indent(root);ET.ElementTree(root).write(target/'fixture.sdf',encoding='utf-8',xml_declaration=True)
        (target/'report.json').write_text(json.dumps(spec,ensure_ascii=False,indent=2),encoding='utf-8')
        (target/'projection.json').write_text(json.dumps(projections,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(dict(stage=stage,changed=spec['changed_links'],support_connected=True,
            cameras={k:dict(horizontal_order=sum(p['red_to_green_left_to_right'] for p in v),
                all_inside=sum(p['all_inside_image'] for p in v)) for k,v in projections.items()})))
