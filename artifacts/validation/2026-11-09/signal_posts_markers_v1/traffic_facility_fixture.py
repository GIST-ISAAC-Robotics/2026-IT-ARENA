"""교통 평가 SDF와 임시 마커 시설을 합친다. 제어 노드에는 위치를 전달하지 않는다."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent

def merge(actor_path, output):
    actor_path, output=Path(actor_path),Path(output)
    fixture=A/'marker_posts_provisional.sdf'
    meta=json.loads((A/'marker_posts_provisional.json').read_text(encoding='utf8'))
    assert hashlib.sha256(fixture.read_bytes()).hexdigest()==meta['fixture_sha256']
    root=ET.fromstring(actor_path.read_bytes())
    assert root.tag=='sdf' and root.find('world') is None
    models=ET.parse(fixture).getroot().findall('model')
    existing={m.get('name') for m in root.findall('model')}
    assert all(m.get('name') not in existing for m in models)
    for model in models: root.append(copy.deepcopy(model))
    assert not output.exists()
    output.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True))
    return dict(original_actor_sha256=hashlib.sha256(actor_path.read_bytes()).hexdigest(),
        fixture_sha256=meta['fixture_sha256'],combined_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        original_actors=sorted(existing),added_models=[m.get('name') for m in models],
        original_actor_file_preserved=True,controller_truth_input=False)
