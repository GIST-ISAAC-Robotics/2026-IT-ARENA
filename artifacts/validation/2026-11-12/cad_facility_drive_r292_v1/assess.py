"""제한 주행의 시설 삽입·평가 거리·종료를 원 판정과 구분해 확인한다."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import run
from shapely.affinity import rotate,translate
from build_experimental_track import rectangle

A=Path(__file__).resolve().parent
source=json.loads((run.F/'fixture.json').read_bytes())
root=ET.parse(A/'run1/runtime_world.sdf').getroot().find('world')
models={m.get('name'):m for m in root.findall('model') if m.get('name','').startswith('eval_20261112_')}
assert len(models)==3
assert root.find("plugin[@name='gz::sim::systems::Contact']") is not None
for place in source['locations']:
    model=models['eval_20261112_'+place['name']+'_support']
    assert model.findtext('link/collision/geometry/mesh/uri')==str(run.F/'support_frame_m.stl')
    assert model.findtext('link/sensor/contact/collision')=='support_collision'
_,_,tri,_=run.fixture.analysis.audit.load(run.F/'support_frame_m.stl')
local=run.slab_projection(tri,.0325,.0925)
trace=json.loads((A/'run1/trace.json').read_bytes())
clearance={}
for place in source['locations']:
    shape=translate(rotate(local,place['yaw'],origin=(0,0),use_radians=True),place['x'],place['y'])
    values=[rectangle(r['x'],r['y'],r['yaw'],.2,.15).distance(shape) for r in trace]
    clearance[place['name']]=min(values)
report=json.loads((A/'run1/report.json').read_bytes())
execution=json.loads((A/'fixture_execution.json').read_bytes())
assert report['passed'] and report['source_unchanged'] and not execution['changed_inputs']
assert report['shutdown_clean'] and report['gazebo_stop_service']['verified']
result=dict(scope='12m CAD profile and new fixture on PC; no full lap/traffic/Jetson timing proof',passed=True,
    progress_m=report['progress_m'],peak_ground_speed_mps=report['peak_ground_speed_mps'],
    minimum_body_to_support_projection_m=clearance,stop_displacement_m=report['stop_displacement_m'],
    stop_stable=report['stop_stable'],body_overlap_samples=report['collision_samples']+report['stop_collision_samples'],
    native_sample_counts=report['native_sample_counts'],source_inputs=execution['input_count'],
    physical_contact_evidence='all3readers returned0 with0messages; sensors and Contact plugin installed; no positive contact/control heartbeat in this run, so not independent no-contact proof',
    normal_shutdown=True)
(A/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
