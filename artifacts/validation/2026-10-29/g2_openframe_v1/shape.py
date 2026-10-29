"""센서 가시 단면과 충돌 외형을 구분하는 평가용 개방형 상대차.

실제 팀 차량 CAD가 아니다. 20x15 cm 바닥판의 전체 footprint를 유지하므로
기존 사각형 충돌/도로 평가는 그대로 정확한 수평 투영 범위다.
"""
import math
import xml.etree.ElementTree as ET


PARTS = (
    ('base', (0., 0., .01), (.20, .15, .02)),
    ('post_fl', (.08, .055, .07), (.02, .02, .10)),
    ('post_fr', (.08, -.055, .07), (.02, .02, .10)),
    ('post_rl', (-.08, .055, .07), (.02, .02, .10)),
    ('post_rr', (-.08, -.055, .07), (.02, .02, .10)),
    ('roof', (0., 0., .13), (.20, .15, .02)),
)


def visible_parts(height):
    if not math.isfinite(height):
        raise ValueError('finite scan height required')
    return [name for name, xyz, size in PARTS if xyz[2]-size[2]/2 <= height <= xyz[2]+size[2]/2]


def replace_box(sdf, actor):
    """기존 actor의 pose/publisher를 보존하며 body 형상과 센서만 교체한다."""
    if any(actor[k] != v for k,v in dict(length_m=.20, width_m=.15, height_m=.14, bottom_m=0.).items()):
        raise ValueError('registered 20x15x14 cm actor required')
    root=ET.fromstring(sdf)
    model=root.find('model')
    if model is None or model.get('name') != actor['model_name'] or model.findtext('static') != 'true':
        raise ValueError('expected static evaluation actor')
    body=model.find("link[@name='body']")
    if body is None or len(body.findall('collision')) != 1:
        raise ValueError('expected single-box base actor')
    topic=body.findtext('sensor/contact/topic')
    if not topic:
        raise ValueError('existing contact topic required')
    for child in list(body):
        if child.tag in ('collision','visual','sensor'):
            body.remove(child)
    zref=actor['height_m']/2
    for name, xyz, size in PARTS:
        for kind in ('collision','visual'):
            part=ET.SubElement(body,kind,name=name)
            ET.SubElement(part,'pose').text=' '.join(map(str,(*xyz[:2],xyz[2]-zref,0,0,0)))
            ET.SubElement(ET.SubElement(ET.SubElement(part,'geometry'),'box'),'size').text=' '.join(map(str,size))
            if kind=='visual':
                material=ET.SubElement(part,'material')
                for tag in ('ambient','diffuse'):ET.SubElement(material,tag).text='.55 .08 .08 1'
        sensor=ET.SubElement(body,'sensor',name='contact_'+name,type='contact')
        ET.SubElement(sensor,'always_on').text='true'
        ET.SubElement(sensor,'update_rate').text='200'
        contact=ET.SubElement(sensor,'contact')
        ET.SubElement(contact,'collision').text=name
        ET.SubElement(contact,'topic').text=topic
    return ET.tostring(root,encoding='unicode')


def audit(sdf,actor):
    root=ET.fromstring(sdf);body=root.find("model/link[@name='body']")
    collisions={x.get('name'):x for x in body.findall('collision')}
    visuals={x.get('name'):x for x in body.findall('visual')}
    checks={}
    for name,xyz,size in PARTS:
        c,v=collisions[name],visuals[name]
        expected=(*xyz[:2],xyz[2]-.07,0,0,0)
        checks[name]=all(abs(a-b)<1e-12 for a,b in zip(map(float,c.findtext('pose').split()),expected))
        checks[name] &= tuple(map(float,c.findtext('geometry/box/size').split()))==size
        checks[name] &= c.findtext('pose')==v.findtext('pose') and c.findtext('geometry/box/size')==v.findtext('geometry/box/size')
    sensors=body.findall('sensor')
    checks['sensor_coverage']=len(sensors)==len(PARTS) and {s.findtext('contact/collision') for s in sensors}==set(collisions)
    checks['parts_exact']=set(collisions)==set(visuals)=={p[0] for p in PARTS}
    return dict(passed=all(checks.values()),checks=checks,scan_height_m=.095,
                visible_parts=visible_parts(.095),total_footprint_m=[.20,.15],
                footprint_basis='base and roof both span full rectangle; not a point-return convex hull',
                scan_section_area_m2=4*.02*.02,box_section_area_m2=.20*.15,
                blind_protrusion_m=dict(front_back=.01,left_right=.01),
                simulation_geometry_only=True)
