"""명령 없이 실행 입력·두 launch 차이·시야를 검사한다."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
OUT=BASE/'preflight_v1.json'
if OUT.exists():raise FileExistsError(OUT)
spec=importlib.util.spec_from_file_location('departure_wrapper',BASE/'run.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
launch,parameters,args=module.wrapper_launch(BASE/'run1')
for p in (BASE/'run.py',BASE/'signal_driver.py',BASE/'fixture/simulation.launch.py'):
    compile(p.read_text(encoding='utf-8'),str(p),'exec')
compile(launch,'trial launch','exec')
assert parameters['start_signal_mode']=='official_4red_candidate'
assert parameters['partial_observation_policy']=='bounded_local_v4'
assert args['actuation_mode']=='virtual_mcu' and args['traffic_light']=='false'
manifest=json.loads((BASE/'fixture/manifest.json').read_text())
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==sha for p,sha in manifest['sources'].items())
world=ET.parse(BASE/'fixture/world.sdf').find('world')
model=world.find("model[@name='it_arena_track_static']")
assert world.get('name')=='it_arena_track'
assert len([l for l in model.findall('link') if l.get('name').startswith('trial_lamp_')])==10
scene=json.loads((BASE/'fixture/scene.json').read_text())
slot=next(s for s in scene['starting_grid']['slots'] if s['index']==3)
yaw=slot['yaw_rad'];c,s=math.cos(yaw),math.sin(yaw)
camera=[slot['x']-.075*c,slot['y']-.075*s,.30]
focal=1280/(2*math.tan(math.radians(75)/2))
projection=[]
for lamp in manifest['lamps']:
    xyz=list(map(float,model.find(f"link[@name='{lamp['name']}']/pose").text.split()))[:3]
    dx,dy,dz=[a-b for a,b in zip(xyz,camera)];forward=c*dx+s*dy;left=-s*dx+c*dy
    assert forward>0
    u,v=640-focal*left/forward,360-focal*dz/forward;radius=focal*.0075/forward
    assert radius>=1 and 0<u-radius and u+radius<1280 and 0<v-radius and v+radius<720*.58
    projection.append(dict(name=lamp['name'],pixel_xy=[u,v],radius_px=radius,forward_m=forward))
for row in range(2):
    columns=[r['pixel_xy'][0] for i,r in enumerate(projection) if i%2==row]
    assert all(b>a for a,b in zip(columns,columns[1:]))
# 신호 외 link와 model은 위치·충돌·재질을 포함하여 재귀적으로 같은지 검사한다.
def signature(e):return (e.tag,sorted(e.attrib.items()),(e.text or '').strip(),[signature(x) for x in e])
source=ET.parse(ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf').find('world')
old=source.find("model[@name='it_arena_track_static']")
for link in old.findall('link'):
    if link.get('name') not in manifest['removed_links']:
        assert signature(link)==signature(model.find(f"link[@name='{link.get('name')}']"))
for m in source.findall('model'):
    if m.get('name')!='it_arena_track_static':assert signature(m)==signature(world.find(f"model[@name='{m.get('name')}']"))
result=dict(passed=True,scope='static fixture/configuration preflight, no optical or driving proof',
    production_and_official_sources_unchanged=True,unrelated_geometry_unchanged=True,
    parameters=parameters,launch_arguments=args,projection=projection,
    source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [Path(__file__),BASE/'run.py',BASE/'signal_driver.py',*sorted((BASE/'fixture').glob('*'))] if p.is_file()})
OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(passed=True,minimum_radius_px=min(r['radius_px'] for r in projection),
    image_and_roi_contained=True,unrelated_geometry_unchanged=True)))
