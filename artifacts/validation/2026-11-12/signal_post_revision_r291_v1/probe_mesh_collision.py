"""새 STL 지지 틀의 실제 정적 mesh 충돌: 문 중앙/버팀대 빈틈과 실제 재료를 구분한다."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import box

OUT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('fixture',OUT/'prepare_fixture.py')
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)


def main():
    out=OUT/'mesh_collision';out.mkdir(exist_ok=False)
    mesh=OUT/'support_frame_m.stl'
    _,_,tri,_=fixture.analysis.audit.load(mesh)
    # 선택 위치는 검사 정답일 뿐이며 어떤 자율주행 제어기에도 제공하지 않는다.
    section=fixture.section(tri,.1346)
    solids=[p for p in section.geoms if p.centroid.y>0 and abs(p.centroid.x)<.02]
    solid=max(solids,key=lambda p:p.area).centroid
    gap=next(x for x in np.arange(.04,.14,.005) if all(
        not fixture.section(tri,z).intersects(box(x-.003,solid.y-.003,x+.003,solid.y+.003)) for z in (.1316,.1346,.1376)))
    cases=[dict(name='central_clear',position=[0,0,.1346],size=[.2,.15,.06],expected_contact=False),
           dict(name='brace_gap',position=[float(gap),solid.y,.1346],size=[.006]*3,expected_contact=False),
           dict(name='post_overlap',position=[solid.x,solid.y,.1346],size=[.03]*3,expected_contact=True)]
    root=ET.Element('sdf',version='1.10');world=ET.SubElement(root,'world',name='new_support_probe')
    ET.SubElement(world,'gravity').text='0 0 0'
    physics=ET.SubElement(world,'physics',name='p',type='ode');ET.SubElement(physics,'max_step_size').text='.001'
    ET.SubElement(physics,'real_time_factor').text='1'
    for filename,name in [('physics','Physics'),('user-commands','UserCommands'),('contact','Contact')]:
        ET.SubElement(world,'plugin',filename=f'gz-sim-{filename}-system',name=f'gz::sim::systems::{name}')
    model=ET.SubElement(world,'model',name='frame');ET.SubElement(model,'static').text='true'
    link=ET.SubElement(model,'link',name='structure')
    collision=ET.SubElement(link,'collision',name='support')
    geometry=ET.SubElement(ET.SubElement(collision,'geometry'),'mesh');ET.SubElement(geometry,'uri').text=str(mesh.resolve())
    sensor=ET.SubElement(link,'sensor',name='contacts',type='contact')
    ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='200'
    contact=ET.SubElement(sensor,'contact');ET.SubElement(contact,'collision').text='support'
    ET.SubElement(contact,'topic').text='/new_support_probe/contact'
    for case in cases:
        model=ET.SubElement(world,'model',name=case['name']);ET.SubElement(model,'pose').text=' '.join(map(str,[*case['position'],0,0,0]))
        link=ET.SubElement(model,'link',name='body');inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass').text='.1'
        inertia=ET.SubElement(inertial,'inertia')
        for axis in ('ixx','iyy','izz'):ET.SubElement(inertia,axis).text='.001'
        shape=ET.SubElement(link,'collision',name='test')
        ET.SubElement(ET.SubElement(ET.SubElement(shape,'geometry'),'box'),'size').text=' '.join(map(str,case['size']))
    ET.indent(root);path=out/'world.sdf';ET.ElementTree(root).write(path,encoding='utf8',xml_declaration=True)
    env=dict(os.environ,GZ_PARTITION=f'new_support_probe_{os.getpid()}')
    report=dict(cases=cases,scope='isolated mesh physics; no vehicle driving or hardware proof',passed=False)
    log=(out/'gz.log').open('w');contacts=(out/'contacts.jsonl').open('w');server=None;reader=None
    try:
        server=subprocess.Popen(['gz','sim','-s','-v','2',str(path)],env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
        time.sleep(3)
        reader=subprocess.Popen(['gz','topic','-e','--json-output','-t','/new_support_probe/contact'],env=env,
            stdin=subprocess.DEVNULL,stdout=contacts,stderr=log,start_new_session=True)
        time.sleep(1)
        response=subprocess.run(['gz','service','-s','/world/new_support_probe/control','--reqtype','gz.msgs.WorldControl',
            '--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: false'],env=env,capture_output=True,text=True,timeout=5)
        assert response.returncode==0 and 'true' in response.stdout, response.stdout+response.stderr
        time.sleep(3)
    except Exception as error:report['error']=repr(error)
    finally:
        if reader is not None:
            os.killpg(reader.pid,signal.SIGINT)
            try:reader.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(reader.pid,signal.SIGTERM);reader.wait(timeout=4)
        if server is not None:
            result=subprocess.run(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean',
                '--timeout','3000','--req','stop: true'],env=env,capture_output=True,text=True,timeout=5)
            report['server_stop_response']=result.stdout
            try:server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                report['forced_stop']=True;os.killpg(server.pid,signal.SIGTERM);server.wait(timeout=5)
            report['server_returncode']=server.returncode
        log.close();contacts.close()
        counts={c['name']:0 for c in cases};malformed=0
        for line in (out/'contacts.jsonl').read_text().splitlines():
            try:
                for contact in json.loads(line).get('contact',[]):
                    names=[contact[k]['name'] for k in ('collision1','collision2')]
                    for key in counts:
                        if any(n.startswith(key+'::') for n in names):counts[key]+=1
            except (ValueError,KeyError):malformed+=1
        report.update(contact_counts=counts,malformed_contact_lines=malformed)
        report['passed']=bool(not report.get('error') and not report.get('forced_stop') and report.get('server_returncode')==0
            and all(bool(counts[c['name']])==c['expected_contact'] for c in cases) and malformed==0)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
