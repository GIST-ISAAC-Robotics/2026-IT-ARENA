"""계산 전달 실패와 시설 경로를 분리하는 r310 전용 벽시계 배율."""
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
A=Path(__file__).resolve().parent.parent;ROOT=A.parents[3]
spec=importlib.util.spec_from_file_location('r310_simulation',ROOT/'src/arena_bringup/launch/simulation.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
original=module._runtime_world_overrides

def isolated(*args,**kwargs):
    report=original(*args,**kwargs)
    destination=Path(args[1] if len(args)>1 else kwargs['destination'])
    before=destination.read_bytes();root=ET.fromstring(before)
    factors=root.findall('world/physics/real_time_factor')
    assert len(factors)==1 and float(factors[0].text)==1.0,'registered factor1 required'
    step=root.findtext('world/physics/max_step_size')
    original_text=factors[0].text;factors[0].text='0.25'
    after=ET.tostring(root,encoding='utf-8',xml_declaration=True)
    factors[0].text=original_text
    assert ET.tostring(root)==ET.tostring(ET.fromstring(before)),'non-factor geometry changed'
    (A/'rtf_audit.json').write_text(json.dumps(dict(before_sha256=hashlib.sha256(before).hexdigest(),after_sha256=hashlib.sha256(after).hexdigest(),real_time_factor_before=1.0,real_time_factor_after=.25,max_step_size_unchanged=step,only_factor_changed=True),indent=2)+'\n')
    destination.write_bytes(after)
    return report

module._runtime_world_overrides=isolated
generate_launch_description=module.generate_launch_description
