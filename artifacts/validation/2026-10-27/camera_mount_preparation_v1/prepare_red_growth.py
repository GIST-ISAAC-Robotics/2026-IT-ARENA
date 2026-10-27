"""기존 정지 장면의 0~3개 빨강 자료 준비. 순차 실시간 출발 시험은 아니다."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

OUT=Path(__file__).resolve().parent
source=OUT/'render_h25_red4/fixture.sdf'
spec_path=OUT/'render_h25_red4/report.json'
for count in range(4):
    root=ET.parse(source).getroot()
    world=root.find('world')
    for link in world.findall("model[@name='it_arena_track_static']/link"):
        name=link.get('name','')
        if not name.startswith('lamp_'):
            continue
        on=name.startswith('lamp_red_') and int(name.rsplit('_',1)[1])<=count
        material=link.find('visual/material')
        for field in ('ambient','diffuse','emissive'):
            node=material.find(field)
            assert node is not None
            node.text='1 0 0 1' if on else '0 0 0 1' if field=='emissive' else '.004 .004 .004 1'
    directory=OUT/f'render_h25_red{count}'
    directory.mkdir(exist_ok=False)
    ET.indent(root)
    ET.ElementTree(root).write(directory/'fixture.sdf',encoding='utf-8',xml_declaration=True)
    spec=json.loads(spec_path.read_text(encoding='utf-8'))
    spec['stage']=f'red{count}'
    spec['derivation']={'source':source.relative_to(OUT).as_posix(),
        'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'changed':'lamp materials only; all poses, cameras, nominal boxes preserved'}
    (directory/'report.json').write_text(json.dumps(spec,ensure_ascii=False,indent=2),encoding='utf-8')
    print(directory.name)
