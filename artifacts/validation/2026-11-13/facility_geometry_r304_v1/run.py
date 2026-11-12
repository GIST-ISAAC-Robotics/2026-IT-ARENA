"""r297 선택형 벽 조각 연결의 CAD+시설 단독1랩 주행. 시설 위치는 시험 가정, 정답은 평가 전용."""
import hashlib
import importlib.util
import inspect
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from shapely.affinity import rotate, translate
from shapely.geometry import Polygon
from shapely.ops import unary_union

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
F=ROOT/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1'
sys.path[:0]=[str(ROOT/'scripts'),str(F)]
import validate_basic_autonomy as target
import prepare_fixture as fixture
from launch_prefix import instrument_start


def slab_projection(tri,lo,hi):
    """닫힌 mesh의 높이 구간 투영. 외접상자로 버팀대 빈틈을 채우지 않는다."""
    polygons=[fixture.section(tri,lo),fixture.section(tri,hi)]
    for face in tri:
        vertices=list(face)
        for z,above in ((lo,True),(hi,False)):
            clipped=[]
            for p,q in zip(vertices,vertices[1:]+vertices[:1]):
                inside=lambda v:v[2]>=z if above else v[2]<=z
                ip,iq=inside(p),inside(q)
                if ip:clipped.append(p)
                if ip!=iq:clipped.append(p+(q-p)*(z-p[2])/(q[2]-p[2]))
            vertices=clipped
            if not vertices:break
        if len(vertices)>=3:
            poly=Polygon([p[:2] for p in vertices])
            if poly.area>1e-14:polygons.append(poly)
    result=unary_union(polygons)
    assert result.is_valid
    return result


def main():
    out=A/'run1';registration=A/'registered.json'
    assert not out.exists() and not registration.exists(),'preserve previous execution'
    assert not sys.argv[1:],'one bounded registered trial'
    spec=json.loads((F/'fixture.json').read_bytes())
    mesh=F/'support_frame_m.stl'
    assert hashlib.sha256(mesh.read_bytes()).hexdigest()==spec['mesh_sha256']
    assert json.loads((F/'mesh_collision/report.json').read_bytes())['passed']
    root=ET.parse(F/'supports_trial.sdf').getroot()
    ET.SubElement(root,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
    topics=[]
    for model in root.findall('model'):
        topic='/r304/'+model.get('name')+'/contact';topics.append(topic)
        sensor=ET.SubElement(model.find('link'),'sensor',name='contact_probe',type='contact')
        ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='100'
        contact=ET.SubElement(sensor,'contact');ET.SubElement(contact,'collision').text='support_collision'
        ET.SubElement(contact,'topic').text=topic
    ET.indent(root);world=A/'facilities.sdf';ET.ElementTree(root).write(world,encoding='utf8',xml_declaration=True)
    _,_,tri,_=fixture.analysis.audit.load(mesh)
    local=slab_projection(tri,.0325,.0925)
    placed=[translate(rotate(local,p['yaw'],origin=(0,0),use_radians=True),p['x'],p['y']) for p in spec['locations']]
    original_obstacles=target.obstacle_polygons
    target.obstacle_polygons=lambda path:[*original_obstacles(path),*placed]
    original_start=instrument_start(target.start_demo_process,A/'launch_command.json');readers=[];handles=[]
    def start(*args,**kwargs):
        bound=inspect.signature(original_start).bind(*args,**kwargs);bound.apply_defaults()
        extra=list(bound.arguments['extra_arguments'])
        assert not any(x.startswith(('evaluation_actor_sdf:=','signal_posts:=')) for x in extra)
        extra.append('evaluation_actor_sdf:='+str(world))
        bound.arguments['extra_arguments']=extra
        (A/'launch_extra.json').write_text(json.dumps(extra,indent=2)+'\n')
        for i,topic in enumerate(topics):
            handle=(A/f'contacts_{i}.jsonl').open('w');handles.append(handle)
            readers.append(subprocess.Popen(['gz','topic','-e','--json-output','-t',topic],
                stdin=subprocess.DEVNULL,stdout=handle,stderr=subprocess.DEVNULL,start_new_session=True))
        return original_start(*bound.args,**bound.kwargs)
    target.start_demo_process=start
    parent=ROOT/'artifacts/validation/2026-11-11/rear_structure_memory_r77_v1/registered.json'
    args=list(json.loads(parent.read_bytes())['args'])
    for flag,value in [('--output',str(out)),('--max-sim-seconds','90'),
            ('--local-sensor-config',str(ROOT/'src/arena_description/config/b0390_c1_cad_20261112_frontwire_4mm.yaml')),
            ('--lidar-native-sampling-profile',str(ROOT/'config/tests/c1_standard10_native_angles_v1.json'))]:
        args[args.index(flag)+1]=value
    args.extend(['--target-progress-m','46.64','--g1-narrow-occluder-bridge','--g1-junction-continuity','--course-marker-visibility','blank_20_30'])
    paths=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.xml','.xacro','.sdf','.cpp','.hpp','.stl')]
    paths.extend((ROOT/'scripts').glob('*.py'))
    paths.extend([ROOT/'config/tests/c1_standard10_native_angles_v1.json',Path(__file__),A/'candidate.py',A/'node_entry.py',A/'launch_prefix.py',A/'course.launch.py',*sorted((A/'launch').glob('*.py')),world,mesh,F/'fixture.json',F/'prepare_fixture.py'])
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(paths))}
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for path in hashes:archive.write(ROOT/path,path)
    registration.write_text(json.dumps(dict(args=args,input_sha256=hashes,scope='single46.64m course target including ID20/30 assumed supports, real_time_factor0.25 geometry isolation; no artificial Jetson delay; not real-time, traffic or physical proof',
        changes=['CAD C1 X75.4/Z134.6 mm and 4 mm front wiring','camera axis198.363mm, X55mm remains provisional','new support mesh; marker placement user assumption, ID20 known road overlap retained','ID20/30 patterns blank; ID49 centre140mm retained'],
        evaluation='body .0325-.0925m mesh slab projections and physical support contact topics; not controller input',
        launch_extras=['evaluation_actor_sdf:='+str(world)],current_controller='current common code; r282 artifact-only timing/process candidates not loaded',
        unchanged=['native CW C1 10Hz and both compensation','single motor28counts,2.5*81/17 ratio,65mm wheels','drive independent guard safe boundary,STOP,physics and gains']),indent=2)+'\n')
    code=1
    try:
        sys.argv=args
        code=target.main()
    finally:
        for process in readers:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        for handle in handles:handle.close()
        contacts=[]
        for i,topic in enumerate(topics):
            count=bad=messages=0;names=set()
            for line in (A/f'contacts_{i}.jsonl').read_text().splitlines():
                try:
                    messages+=1
                    for c in json.loads(line).get('contact',[]):
                        pair=[c[k]['name'] for k in ('collision1','collision2')]
                        if any(n.startswith('arena_car::') for n in pair):count+=1;names.update(pair)
                except (ValueError,KeyError):bad+=1
            contacts.append(dict(topic=topic,messages=messages,vehicle_contacts=count,malformed=bad,names=sorted(names)))
        changed=[p for p,h in hashes.items() if not (ROOT/p).is_file() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        result=dict(validation_returncode=code,changed_inputs=changed,input_count=len(hashes),contacts=contacts,reader_returncodes=[p.returncode for p in readers])
        (A/'fixture_execution.json').write_text(json.dumps(result,indent=2)+'\n')
    return code or int(bool(changed))


if __name__=='__main__':raise SystemExit(main())
