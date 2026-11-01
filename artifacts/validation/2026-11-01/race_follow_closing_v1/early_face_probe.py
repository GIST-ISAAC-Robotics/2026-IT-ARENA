"""실측 반환만으로 첫 추종 전환을 더 일찍 예고할 수 있었는지 보는 고정 입력 대조."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import swept_limit
from arena_autonomy.scene_motion import motion_scene

BASE=Path(__file__).resolve().parent


def face(scene):
    # 기존 G1/G2 정지·우회 인식의 1.60 m 창은 손대지 않는다. 똑같은 세 반환의
    # 연결을 G1이 실제 점유를 검사하는 2.5 m 이내에서만 별도로 본다.
    groups=[];current=[];idx=None
    for r in scene.rays:
        p=r.hit_xy_m
        ok=p is not None and .20<=p[0]<=2.5 and abs(p[1])<=.24
        if not ok or (idx is not None and r.source_index!=idx+1):
            if current:groups.append(np.asarray(current));current=[]
        if ok:current.append(p)
        idx=r.source_index
    if current:groups.append(np.asarray(current))
    faces=[]
    for arr in groups:
        if len(arr)<3 or not np.any(abs(arr[:,1])<.15):continue
        pts=arr[arr[:,0]<=np.quantile(arr[:,0],.1)+.08]
        if len(pts)>=3:faces.append(pts)
    faces.sort(key=lambda pts:np.median(pts[:,0]))
    if not faces or (len(faces)>1 and np.median(faces[1][:,0])-np.median(faces[0][:,0])<.20):return None
    return faces[0]


def propose(scene,v,angle,prev,cfg):
    patch=face(scene)
    if patch is None:return v,None
    x=float(np.median(patch[:,0]))+cfg.wheelbase_m/2
    cap=max(.35,(x-1.20)/.8)
    if cap>=v:return v,None
    margin=.025+cfg.measurement_padding_m+cfg.object_motion_bound_mps*(scene.t_ref_s-scene.source_first_s)
    arc=min(swept_limit(patch,s,cfg.wheelbase_m,horizon=2.5,margin=margin,
            length=cfg.length_m,width=cfg.width_m) for s in (angle,prev))
    if arc>=2.5 or straight_reference(scene,cfg)[0] is None:return v,None
    return cap,x


def analyze(folder):
    rep=json.loads((folder/'report.json').read_text(encoding='utf-8'))
    if 'case_window' in rep:
        green=rep['case_window']['green_sim_s'];end=rep['case_window']['end_sim_s']
    else:
        # 단독 코스에서는 차 없는 직선/벽에도 감속이 생기는지 본다.
        with (folder/'trajectory.jsonl').open() as f:
            rows=(x for x in map(json.loads,f) if x['light']=='green')
            last=next(rows);green=last['time'];length=rep['lap_length_m']
            for item in rows:
                if last['progress_m']<length<=item['progress_m']:
                    k=(length-last['progress_m'])/(item['progress_m']-last['progress_m'])
                    end=last['time']+k*(item['time']-last['time']);break
                last=item
            else:raise ValueError('단독 랩 종료점 누락')
    digest=hashlib.sha256();records=[];changed=[]
    with (folder/'follow_decisions.jsonl').open('rb') as stream:
        for line in stream:
            digest.update(line);r=json.loads(line);d=r['data']
            if r['kind']=='header':
                cfg=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
            if r['kind']!='call':continue
            raw=d['input'];t=raw['reference_s']
            if not green<=t<=end:continue
            v,angle,q=d['output'];new=v
            # 실제 양수 정상 추종이 첫 조각을 미확정한 때만 사용. 우회·양보·정지에는 적용하지 않는다.
            n=q.get('normal',q);prep=q.get('preparation') or {}
            if (v>.35 and n.get('reason')=='local_follow' and q.get('handoff_state','FOLLOW')=='FOLLOW'
                    and prep.get('reason','no_current_front_patch') in ('no_current_front_patch','single_scan_patch')):
                a,scan,hist=restore_call(raw,motion)
                scene=motion_scene(scan,hist,t,timing_verified=a['timing_verified'],epoch=a['epoch'],
                                   rear_half_angle_deg=a['rear_half_angle_deg'])
                new,x=propose(scene,v,angle,raw['previous_steering_rad'],cfg)
                if new<v:changed.append(dict(elapsed_s=t-green,was=v,would_be=new,rear_axle_to_hit_m=x))
            records.append((t-green,v,new))
    arr=np.asarray(records)
    def window(index):
        s=arr[(arr[:,0]>=.30)&(arr[:,0]<4.)]
        if len(s)<2:return None
        return float(max(np.interp(s[:,0]-.30,arr[:,0],arr[:,index])-s[:,index]))
    dt=np.diff(arr[:,0]);valid=(dt>0)&(dt<=.10)
    return dict(run=folder.name,calls=len(arr),changed_calls=len(changed),
        raw_hash=digest.hexdigest(),first_4s_max_300ms_target_drop_mps=[window(1),window(2)],
        lost_command_integral_m=float(np.sum(dt[valid]*(arr[:-1,1]-arr[:-1,2])[valid])),
        zero_changes=int(np.sum((arr[:,1]==0)!=(arr[:,2]==0))),examples=changed[:8])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('out',type=Path)
    a=p.parse_args()
    result=analyze(a.folder)
    with a.out.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
