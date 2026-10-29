"""여섯 부품 각각의 실제 Gazebo 접촉 메시지를 확인하는 격리 시험."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from shape import PARTS, replace_box, audit
from test_shape import sample, ACTOR


def make_world():
    root=ET.Element('sdf',version='1.9');world=ET.SubElement(root,'world',name='g2_shape_probe')
    ET.SubElement(world,'gravity').text='0 0 0'
    physics=ET.SubElement(world,'physics',name='p',type='ode')
    ET.SubElement(physics,'max_step_size').text='.001'
    ET.SubElement(physics,'real_time_factor').text='1'
    for filename,name in [('physics','Physics'),('contact','Contact')]:
        ET.SubElement(world,'plugin',filename='gz-sim-'+filename+'-system',name='gz::sim::systems::'+name)
    actor=ET.fromstring(replace_box(sample(),ACTOR)).find('model')
    actor.find('pose').text='0 0 .07 0 0 0'
    for plugin in actor.findall('plugin'):actor.remove(plugin)
    world.append(actor)
    for name,xyz,size in PARTS:
        # 각 부품 바깥 면에5 mm 겹치고 다른 부품/프로브의 간섭을 피한다.
        # base/roof는 앞면, 기둥은 좌우 바깥면.
        x,y,z=xyz
        if name in ('base','roof'):x=size[0]/2+.005
        else:y+=(1 if y>0 else -1)*(size[1]/2+.005)
        model=ET.SubElement(world,'model',name='probe_'+name)
        ET.SubElement(model,'pose').text=f'{x} {y} {z} 0 0 0'
        link=ET.SubElement(model,'link',name='body')
        inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass').text='.01'
        inertia=ET.SubElement(inertial,'inertia')
        for k in ('ixx','iyy','izz'):ET.SubElement(inertia,k).text='0.000001'
        collision=ET.SubElement(link,'collision',name='collision')
        ET.SubElement(ET.SubElement(ET.SubElement(collision,'geometry'),'box'),'size').text='.02 .02 .01'
    return ET.tostring(root,encoding='unicode')


def main(output):
    output.mkdir(exist_ok=False)
    (output/'world.sdf').write_text(make_world(),encoding='utf-8')
    env=dict(os.environ,GZ_PARTITION='g2_shape_'+str(os.getpid()))
    contacts=(output/'contacts.jsonl').open('w');log=(output/'gz.log').open('w')
    processes=[];forced=[]
    try:
        reader=subprocess.Popen(['gz','topic','-e','--json-output','-t','/eval/contact'],env=env,stdout=contacts,stderr=log,start_new_session=True)
        processes.append(('reader',reader));time.sleep(.5)
        server=subprocess.Popen(['gz','sim','-s','-v','2',str(output/'world.sdf')],env=env,stdout=log,stderr=log,start_new_session=True)
        processes.append(('server',server));time.sleep(3)
        control=subprocess.run(['gz','service','-s','/world/g2_shape_probe/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000',
            '--req','pause: false'],env=env,capture_output=True,text=True,timeout=10)
        (output/'control_receipt.txt').write_text(control.stdout+control.stderr,encoding='utf-8')
        time.sleep(4)
    finally:
        for name,process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    forced.append(name);os.killpg(process.pid,signal.SIGKILL);process.wait()
        contacts.close();log.close()
    pairs=set();errors=0
    for line in (output/'contacts.jsonl').read_text(encoding='utf-8').splitlines():
        try:
            row=json.loads(line)
            for contact in row.get('contact',[]):pairs.add(tuple(sorted(contact[f'collision{i}']['name'] for i in (1,2))))
        except (ValueError,KeyError):errors+=1
    checks={name:tuple(sorted((f'eval_frame::body::{name}',f'probe_{name}::body::collision'))) in pairs for name,_,_ in PARTS}
    control_ok=control.returncode==0 and 'data: true' in control.stdout
    result=dict(passed=all(checks.values()) and not errors and not forced and server.returncode==0 and control_ok,
                control_started=control_ok,
                checks=checks,pairs=sorted(pairs),parse_errors=errors,forced_termination=forced,
                process_exit_codes={n:p.returncode for n,p in processes},
                scope='isolated collision/sensor plumbing probe; not autonomous driving or hardware proof')
    (output/'report.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    raise SystemExit(main(parser.parse_args().output.resolve()))
