"""P22 진입 끝의 센서 전용 관측·기존 경로 비교. 주행·평가 정답 입력 없음."""
import hashlib,json,math
from dataclasses import replace
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.p22_observation import segments

A=Path(__file__).resolve().parent;D=A.parent/'p22_entry_wiring_v1/run1'
def main():
    p=D/'follow_decisions.jsonl';sha=lambda:hashlib.sha256(p.read_bytes()).hexdigest();before=sha()
    first=last=None;angle=0.;armed=False;rows=[];controllers={};old_source=None
    for line in p.open():
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            cfg=FollowConfig(**d['follow_config']);mc=MotionConfig(**d['motion_config'])
            for offset in (.505,.30):
                for side in ('left','right'):
                    c=replace(cfg,wall_offset_m=offset,road_half_m=.305 if offset==.505 else .10,
                              max_speed_mps=.65,open_route_speed_mps=.65)
                    controllers[f'{offset:.3f}_{side}']=(LocalFollow(c),side)
        if e['kind']!='call':continue
        i=d['input'];t=i['reference_s'];state=d['output'][2].get('p22_trial') or {}
        if state.get('first_command_admitted'):first=last=t
        if first is None:continue
        raw,scan,h=restore_call(i,mc)
        if t!=last:
            poses,_=h.poses([last,t],t);angle+=poses[1,2]-poses[0,2];last=t
        if angle<=-math.radians(70):armed=True
        if not armed:continue
        scene=motion_scene(scan,h,t,timing_verified=raw['timing_verified'],epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
        outputs={}
        for name,(f,side) in controllers.items():
            out=f.command(scene,h,raw['speed_mps'],side,raw['previous_steering_rad'],raw['dt_s'],raw_scan=scan)
            outputs[name]=dict(speed_mps=out[0],steering_rad=out[1],reason=out[2]['reason'],
                              detail=freeze(out[2]))
        if scene.source_first_s==old_source:continue
        old_source=scene.source_first_s
        segs=[]
        for s in segments(scene):
            segs.append(dict(first=(s['m']+s['lo']*s['v']).tolist(),last=(s['m']+s['hi']*s['v']).tolist(),
                             heading_deg=s['heading_deg'],span_m=s['span'],residual_m=s['residual']))
        rows.append(dict(t=t,source_s=scene.source_first_s,measured_speed_mps=raw['speed_mps'],
                         relative_yaw_rad=angle,segments=segs,outputs=outputs,
                         points=[list(v) for v in scene.occupied_endpoints_xy_m]))
    out=dict(scope='sensor-only saved-input proposal; not new driving',source=str(p),input_sha256=before,
             input_unchanged=sha()==before,first_entry_s=first,rows=rows)
    (A/'observations.json').write_text(json.dumps(out,separators=(',',':'))+'\n',encoding='utf8')
    compact=dict(input_unchanged=out['input_unchanged'],scans=len(rows),times=[r['t'] for r in rows],
        positive={name:sum(r['outputs'][name]['speed_mps']>0 for r in rows) for name in controllers},
        reasons={name:sorted(set(r['outputs'][name]['reason'] for r in rows)) for name in controllers})
    (A/'summary.json').write_text(json.dumps(compact,indent=2)+'\n',encoding='utf8');print(compact)
if __name__=='__main__':main()
