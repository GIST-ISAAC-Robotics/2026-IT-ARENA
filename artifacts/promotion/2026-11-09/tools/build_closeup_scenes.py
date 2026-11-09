"""홍보용 정지 렌더 장면. 저장 자세와 시각 모델만 사용; 물리·제어 실행 없음."""
from pathlib import Path
import argparse,copy,hashlib,importlib.util,json,math,re,shutil
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
HELPER=ROOT/'artifacts/videos/2026-11-09/feature_showcase/tools/build_replay_scene.py'
spec=importlib.util.spec_from_file_location('replay',HELPER)
replay=importlib.util.module_from_spec(spec);spec.loader.exec_module(replay)
replay.CAM_W,replay.CAM_H=3840,2400
replay.CAM_RATE=2

RACE='artifacts/validation/2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1'
SOLO='artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1'
LEGACY=ROOT/'artifacts/validation/2026-11-03/solo_post_free_v1/run1/runtime_vehicle.sdf'
GROUPS=[
 dict(name='racing',run=RACE,actor='red_car',shots=[
  dict(name='01_racing_front_close',title='나란히 달리는 두 차량 · 앞 사선',time=13.35,eye=[.58,-.10,.32],fov=.94),
  dict(name='02_racing_rear_close',title='추월 순간 · 뒤 사선',time=13.55,eye=[-.62,.14,.35],fov=.90),
  dict(name='03_racing_side_close',title='두 차량의 접전 · 사선 근접',time=13.35,eye=[-.45,.28,.55],fov=.90)]),
 dict(name='solo',run=SOLO,actor=None,shots=[
  dict(name='04_solo_front_close',title='단독 직선 주행 · 앞 사선',time=10.8,eye=[.52,.235,.30],fov=.90),
  dict(name='05_solo_rear_close',title='단독 주행 · 뒤 사선',time=10.8,eye=[-.52,.285,.35],fov=.90),
  dict(name='06_solo_corner_close',title='코너 주행 · 앞바퀴 조향',time=13.913,eye=[.50,-.145,.32],fov=.90)]),
 dict(name='original',run=RACE,actor='original',shots=[
  dict(name='07_original_pass_close',title='시험용 상대 추월 · 근접',time=13.35,eye=[-.56,.13,.36],fov=.96),
  dict(name='08_original_pass_front',title='시험용 상대와 나란히 · 앞 사선',time=13.35,eye=[.57,-.12,.34],fov=.96)])]

def sha(p):return hashlib.file_digest(p.open('rb'),'sha256').hexdigest()
def lookat(pos,target):
    d=target-pos;return [*pos,0,math.atan2(-d[2],math.hypot(d[0],d[1])),math.atan2(d[1],d[0])]
def pose_at(arr,t):return np.array(replay.interp_pose(arr,np.array([t]))).ravel()

def promotional_box_model():
    model,count=replay.vehicle_visual_model(LEGACY)
    # 구형 박스 차체의 시각 간섭/미연결 부품을 홍보용 파생본에서만 정리한다.
    for visual in model.iter('visual'):
        if visual.get('name','').startswith('lidar_link__'):
            p=replay.parse_pose(visual.find('pose'));p[2]+=.024;visual.find('pose').text=replay.fmt(p)
    link=model.find('link')
    def add(name,pose,geometry,color):
        v=ET.SubElement(link,'visual',name=name);ET.SubElement(v,'pose').text=pose
        v.append(ET.fromstring('<geometry>'+geometry+'</geometry>'))
        m=ET.SubElement(v,'material')
        for key in ('ambient','diffuse'):ET.SubElement(m,key).text=color
        ET.SubElement(m,'specular').text='.35 .35 .35 1'
    add('promo_camera_support','.055 0 .113 0 0 0','<box><size>.012 .018 .043</size></box>','.12 .14 .17 1')
    add('promo_camera_lens','.067 0 .145 0 1.5707963 0','<cylinder><radius>.0065</radius><length>.004</length></cylinder>','.015 .055 .08 1')
    for x in (-.0675,.0675):
        for y in (-.074,.074):
            add(f'promo_hub_{x}_{y}',f'{x} {y} .0325 1.5707963 0 0','<cylinder><radius>.011</radius><length>.001</length></cylinder>','.45 .49 .54 1')
    return model,count+6

def detach_steering_parts(model,world):
    """앞바퀴/너클/허브를 관찰용 정적 모델로 분리하여 기록 조향 자세를 적용한다."""
    parts=[];link=model.find('link')
    for visual in list(link.findall('visual')):
        name=visual.get('name','');side=None
        if name.startswith(('front_left_wheel__','front_left_steering_link__')):side='left'
        if name.startswith(('front_right_wheel__','front_right_steering_link__')):side='right'
        if name.startswith('promo_hub_0.0675_'):side='left' if name.endswith('_0.074') else 'right'
        if side is None:continue
        local=replay.parse_pose(visual.find('pose'));link.remove(visual)
        unique=model.get('name')+'_'+name
        part=ET.SubElement(world,'model',name=unique);ET.SubElement(part,'static').text='true'
        pl=ET.SubElement(part,'link',name='part');pl.append(visual)
        visual.find('pose').text='0 0 0 0 0 0'
        parts.append(dict(name=unique,local=local,side=side,pivot=np.array([.0675,.0675 if side=='left' else -.0675,.0325])))
    return parts

def steer_part(part,angle,vehicle_pose):
    local=part['local'];rz=replay.rpy_matrix(0,0,angle)
    translated=part['pivot']+rz@(local[:3]-part['pivot'])
    rotated=replay.matrix_rpy(rz@replay.rpy_matrix(*local[3:]))
    return replay.compose(np.asarray(vehicle_pose),np.r_[translated,rotated])

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--plugin-dir',type=Path,required=True);a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    allmeta=[]
    for group in GROUPS:
        run=ROOT/group['run'];out=a.out/group['name'];out.mkdir();(out/'frames').mkdir()
        inputs=[run/'runtime_world.sdf',run/'runtime_vehicle.sdf',run/'ego_poses.jsonl',run/'signal_sequence.jsonl',run/'trace.json',HELPER]
        if group['actor']:inputs.extend([run/'actor_poses.jsonl',run/'evaluation_actor.sdf'])
        raw=(run/'runtime_world.sdf').read_text(encoding='utf-8')
        for obj in run.glob('batched_*.obj'):
            shutil.copy2(obj,out/obj.name);inputs.append(obj)
        raw=re.sub(r'<uri>/tmp/arena_world_[^/]+/(batched_[^<]+\.obj)</uri>',lambda m:f'<uri>{(out/m.group(1)).as_posix()}</uri>',raw)
        root=ET.fromstring(raw);world=root.find('world')
        for plugin in list(world.findall('plugin')):
            if plugin.get('filename')!='gz-sim-sensors-system':world.remove(plugin)
        for model in list(world.findall('model')):
            if model.get('name')=='eval_g1b_actor':world.remove(model)
        signal=replay.apply_signal_state(world,run/'signal_sequence.jsonl',min(s['time'] for s in group['shots']),max(s['time'] for s in group['shots']))
        promotional=group['name']!='original'
        if promotional:
            inputs.append(LEGACY);ego_model,visuals=promotional_box_model()
        else:ego_model,visuals=replay.vehicle_visual_model(run/'runtime_vehicle.sdf')
        world.append(ego_model)
        if group['actor']=='red_car':
            actor_model=copy.deepcopy(ego_model);actor_model.set('name','replay_actor')
            for v in list(actor_model.iter('visual')):
                if v.get('name')=='chassis__visual':
                    for attr in ('ambient','diffuse'):
                        el=v.find('material/'+attr)
                        if el is not None:el.text='.80 .045 .065 1'
                # 같은 ID49가 두 번 보이지 않게 홍보용 상대에서는 뒤 식별판을 뺀다.
                if 'rear_marker_id49' in v.get('name',''):actor_model.find('link').remove(v)
            world.append(actor_model)
        elif group['actor']=='original':world.append(replay.actor_visual_model(run/'evaluation_actor.sdf')[0])
        ego_parts=detach_steering_parts(ego_model,world)
        actor_parts=detach_steering_parts(actor_model,world) if group['actor']=='red_car' else []
        camera=replay.camera_model();camera.find('.//horizontal_fov').text=str(group['shots'][0]['fov'])
        camera.find('.//clip/near').text='.015';world.append(camera)
        # 한 장면 묶음의 렌즈는 고정. 카메라 거리/방향만 컷마다 바꾼다.
        fov=group['shots'][0]['fov']
        ego_arr=replay.load(run/'ego_poses.jsonl');actor_arr=replay.load(run/'actor_poses.jsonl') if group['actor'] else None
        telemetry=json.loads((run/'trace.json').read_text(encoding='utf-8'))
        dyn=[r['dynamics'] for r in telemetry if 'left_steering_rad' in r.get('dynamics',{})]
        dyn_t=np.array([r['sim_time_s'] for r in dyn])
        if not dyn:
            inputs.append(run/'actuation_trace.json')
            mcu=json.loads((run/'actuation_trace.json').read_text(encoding='utf-8'))
            mcu_t=np.array([r['now_us']/1e6 for r in mcu])
            mcu_delta=np.array([r['steering_rad'] for r in mcu])
        lines=[];shotmeta=[]
        for i,shot in enumerate(group['shots']):
            ego=pose_at(ego_arr,shot['time']);actor=pose_at(actor_arr,shot['time']) if actor_arr is not None else None
            center=ego[:2] if actor is None else (ego[:2]+actor[:2])/2
            heading=ego[2];forward=np.array([math.cos(heading),math.sin(heading)]);left=np.array([-forward[1],forward[0]])
            ex,ey,ez=shot['eye'];target=np.r_[center,.070]
            eye=np.r_[center+ex*forward+ey*left,ez]
            cam=lookat(eye,target)
            stage=i+1.
            values={'replay_ego':[*ego[:2],.003,0,0,ego[2]],'replay_cam':cam}
            if actor is not None:values['replay_actor']=[*actor[:2],.003 if group['actor']=='red_car' else .06,0,0,actor[2]]
            if dyn:
                steering={side:float(np.interp(shot['time'],dyn_t,[r[side+'_steering_rad'] for r in dyn])) for side in ('left','right')}
                steering_source='recorded_measured_joint_angles'
            else:
                delta=float(np.interp(shot['time'],mcu_t,mcu_delta))
                steering={side:math.atan2(2*.135*math.sin(delta),2*.135*math.cos(delta)+sign*.135*math.sin(delta)) for side,sign in [('left',-1),('right',1)]}
                steering_source='recorded_MCU_target_Ackermann; measured joint angles not retained'
            for part in ego_parts:values[part['name']]=steer_part(part,steering[part['side']],values['replay_ego'])
            for part in actor_parts:values[part['name']]=steer_part(part,0.,values['replay_actor'])
            for time in ([0.] if i==0 else [])+[stage,stage+.8]:
                for name,value in values.items():lines.append(f'{name} {time:.4f} {replay.fmt(value)}')
            shotmeta.append(dict(**shot,frame_time_s=stage+.5,ego_pose=ego.tolist(),actor_pose=actor.tolist() if actor is not None else None,camera_pose=cam,actual_hfov=fov,
                                 ego_steering_source=steering_source,ego_steering_rad=steering,ego_steering_deg={k:math.degrees(v) for k,v in steering.items()}))
        duration=len(group['shots'])+1.
        (out/'poses.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
        plugin=ET.SubElement(world,'plugin',filename=str(a.plugin_dir/'libshowcase-pose-replay.so'),name='showcase::PoseReplay')
        for key,value in [('pose_file',str(out/'poses.txt')),('frame_dir',str(out/'frames')),('done_file',str(out/'done.txt')),('end_time',str(duration)),('image_topic','/showcase/cam')]:ET.SubElement(plugin,key).text=value
        ET.ElementTree(root).write(out/'render_world.sdf',encoding='utf-8',xml_declaration=True)
        # 이 파생본에 물리/제어 플러그인이 없는지 확인한다.
        assert all(p.get('filename')=='gz-sim-sensors-system' or p.get('name')=='showcase::PoseReplay' for p in world.findall('plugin'))
        assert not world.findall('.//joint')
        meta=dict(name=group['name'],run=group['run'],actor_style=group['actor'],ego_style='legacy_box_promotional' if promotional else 'recorded',width=3840,height=2400,signal=signal,shots=shotmeta,
                  source_sha256={str(p.relative_to(ROOT)):sha(p) for p in inputs},
                  scope='static recorded-pose promotional rendering; no physics/controller test; ego steering from measured joints (solo) / MCU target Ackermann (traffic); no wheel-spin animation',
                  actor_visual_note=('11/3 박스 차체 기반 홍보용 외관; LiDAR 높이·카메라 받침·렌즈·휠 허브의 시각 요소 정리. '+('상대는 같은 차체의 빨간색; 원 시험 상대는 오목 블록.' if group['actor']=='red_car' else '단독 기록 자세.')) if promotional else '원 실행 시각 모델 보존')
        (out/'meta.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8');allmeta.append(meta)
    (a.out/'all_meta.json').write_text(json.dumps(allmeta,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'groups':len(allmeta),'shots':sum(len(m['shots']) for m in allmeta),'output':str(a.out)}))

if __name__=='__main__':main()
