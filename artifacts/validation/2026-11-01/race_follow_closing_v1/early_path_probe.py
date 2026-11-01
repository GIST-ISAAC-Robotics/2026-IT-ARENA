"""새 분기 없이 저장 판단만 비교: 현재 조향 원호 대신 현재 추종 경로에 걸친 전방 조각."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from early_face_probe import face
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.scene_motion import motion_scene

BASE=Path(__file__).resolve().parent


def prospective(scene,path,cap,cfg):
    patch=face(scene)
    if patch is None or len(path)<3:return cap,'patch_missing'
    rear=np.asarray(path)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(rear,axis=0),axis=1))]
    nearest=int(np.argmin(np.linalg.norm(rear,axis=1)))
    points=rear[nearest:]
    delta=patch[None,:,:]-points[:,None,:]
    dx=np.gradient(points,axis=0);norm=np.maximum(1e-9,np.linalg.norm(dx,axis=1))
    tangents=dx/norm[:,None]
    along=np.sum(delta*tangents[:,None,:],axis=2)
    cross=delta[:,:,0]*tangents[:,1:2]-delta[:,:,1]*tangents[:,0:1]
    margin=.025+cfg.measurement_padding_m+cfg.object_motion_bound_mps*(scene.t_ref_s-scene.source_first_s)
    hit=(abs(along)<cfg.length_m/2+margin)&(abs(cross)<cfg.width_m/2+margin)
    if not hit.any():return cap,'patch_outside_current_path'
    if straight_reference(scene,cfg)[0] is None:return cap,'curved_or_unconfirmed_wall'
    dist=float(np.median(patch[:,0]))+cfg.wheelbase_m/2
    # 관측 창과 미래 우회 여유는 앞선 비교와 같다. 현재 더 강한 제동을 늦추지 않는다.
    target=max(.35,(dist-1.20)/.8)
    return min(cap,target),'limited' if target<cap else 'unchanged'


def analyze(folder):
    report=json.loads((folder/'report.json').read_text())
    if 'case_window' in report:g=report['case_window']['green_sim_s'];end=report['case_window']['end_sim_s']
    else:
        metrics=json.loads((BASE.parent/'corner_approach_v1/solo_2p0_metrics.json').read_text())
        g=metrics['first_green_sample_s'];end=g+metrics['lap_s_from_first_green_sample']
    changed=[];series=[];counts={};sha=hashlib.sha256()
    with (folder/'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            sha.update(line);event=json.loads(line);data=event['data']
            if event['kind']=='header':
                cfg=FollowConfig(**data['follow_config']);motion=MotionConfig(**data['motion_config'])
            if event['kind']!='call':continue
            raw=data['input'];t=raw['reference_s']
            if not g<=t<=end:continue
            v,angle,q=data['output'];new=v;n=q.get('normal',q);why='other'
            if v>.35 and q.get('handoff_state','FOLLOW')=='FOLLOW' and n.get('reason')=='local_follow':
                a,scan,h=restore_call(raw,motion)
                scene=motion_scene(scan,h,t,timing_verified=a['timing_verified'],epoch=a['epoch'],rear_half_angle_deg=a['rear_half_angle_deg'])
                new,why=prospective(scene,n.get('path_points',[]),v,cfg)
                if new<v:changed.append(dict(elapsed_s=t-g,old=v,new=new))
            counts[why]=counts.get(why,0)+1
            series.append((t-g,v,new))
    a=np.asarray(series);dt=np.diff(a[:,0]);dt=np.where((dt>0)&(dt<=.10),dt,0.)
    def drop(k):
        ts=a[(a[:,0]>=.3)&(a[:,0]<4),0]
        return float(max(np.interp(ts-.3,a[:,0],a[:,k])-np.interp(ts,a[:,0],a[:,k])))
    return dict(run=folder.name,calls=len(a),changed=len(changed),reason=counts,
        first_4s_max_300ms_target_drop_mps=[drop(1),drop(2)],
        lost_command_integral_m=float(sum(dt*(a[:-1,1]-a[:-1,2]))),zero_changes=int(sum((a[:,1]==0)!=(a[:,2]==0))),
        input_sha256=sha.hexdigest(),examples=changed[:10],scope='기존 스캔/경로와 명령만으로 대조. 실제 속도·감속/우회/랩 시간 결과는 아니다.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('out',type=Path)
    args=p.parse_args()
    with args.out.open('x',encoding='utf-8') as f:json.dump(analyze(args.folder),f,ensure_ascii=False,indent=2)
