"""계산 전달 실패와 시설 경로를 분리하는 r428 전용 벽시계 배율."""
import hashlib
import importlib.util
import json
import inspect
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
A=Path(__file__).resolve().parent.parent;ROOT=A.parents[3]
spec=importlib.util.spec_from_file_location('r428_simulation',ROOT/'src/arena_bringup/launch/simulation.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
original=module._runtime_world_overrides
sys.path.insert(0,str(A))
from field_wall_geometry import DERIVED,SOURCE_SHA,verify_derived,collision_digest

def isolated(*args,**kwargs):
    info=verify_derived()
    bound=inspect.signature(original).bind(*args,**kwargs)
    assert hashlib.sha256(Path(bound.arguments['source']).read_bytes()).hexdigest()==SOURCE_SHA
    bound.arguments['source']=DERIVED
    report=original(*bound.args,**bound.kwargs)
    destination=Path(args[1] if len(args)>1 else kwargs['destination'])
    assert collision_digest(destination)==info['walls_collision_sha256']
    (A/'field_runtime_receipt.json').write_text(json.dumps(dict(derived_sha256=info['derived_sha256'],runtime_walls_collision_sha256=collision_digest(destination),same_derived_world_for_evaluation=True,world_override_used=False,source_injected_before_visual_batch=True),indent=2)+'\n')
    original_bytes=destination.read_bytes();root=ET.fromstring(original_bytes)
    model=root.find("world/model[@name='it_arena_track_static']")
    beam=model.find("link[@name='tl_beam']")
    assert beam is not None
    removed=ET.tostring(beam,encoding='unicode');model.remove(beam)
    (A/'beam_replacement.json').write_text(json.dumps(dict(removed_link=removed,source_world_sha256=hashlib.sha256(original_bytes).hexdigest(),replacement='r326 exact old610 STL crossbar, no duplicate beam'),indent=2)+'\n')
    before=ET.tostring(root,encoding='utf-8')
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
