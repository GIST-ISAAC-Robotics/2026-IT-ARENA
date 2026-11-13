"""원 실행 결과와 실제 센서 반환의 평가용 시설 위치 대조. 제어 입력으로 사용하지 않음."""
from pathlib import Path
import collections
import hashlib
import json
import math
import struct

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
REPO=ROOT.parents[3]


def main():
    rows=[json.loads(line) for line in (ROOT/'run1/follow_decisions.jsonl').read_text(encoding='utf8').splitlines()]
    calls=[r for r in rows if r['kind']=='call']
    plan=[r for r in calls if r['data'].get('compute',{}).get('stage')=='plan']
    current=[r for r in calls if r['data'].get('compute',{}).get('stage')=='current']
    reasons=lambda rs: dict(collections.Counter(r['data']['output'][2].get('reason') for r in rs))
    pubs=[r['data'] for r in rows if r['kind']=='route_publication']
    act=json.loads((ROOT/'run1/actuation_trace.json').read_text(encoding='utf8'))
    traj=[json.loads(line) for line in (ROOT/'run1/trajectory.jsonl').read_text(encoding='utf8').splitlines()]
    reps=[]
    for seq in (560,563,650,740):
        r=next(r for r in calls if r['seq']==seq);d=r['data'];t=d['input']['reference_s']
        reps.append({'seq':seq,'reference_s':t,'speed_mps':d['input']['speed_mps'],
            'output':d['output'][:2],'reason':d['output'][2].get('reason'),
            'compute':{k:v for k,v in d.get('compute',{}).items() if k!='input'},
            'delivery':d.get('delivery'),
            'nearby_publications':[p for p in pubs if abs(p.get('at_s',-100)-t)<.15],
            'nearest_trajectory':min(traj,key=lambda x:abs(x['t']-t))})
    result={'plan_count':len(plan),'current_count':len(current),'plan_reasons':reasons(plan),
        'current_reasons':reasons(current),'publication_count':len(pubs),
        'positive_route_publication_count':sum(p.get('speed_mps',0)>0 for p in pubs),
        'footer':next(r['data'] for r in rows if r['kind']=='footer'),
        'representatives':reps,'actuation_schema_example':act[0] if act else None,
        'clock_note':'reference_s/trajectory.t/publication.at_s는 sim clock; compute *_wall_s는 별도 monotonic clock, 서로 차감하지 않음'}
    (HERE/'timing.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

    mesh=REPO/'artifacts/validation/2026-11-13/old_signal_610_r326_v1/support_frame_m.stl'
    data=mesh.read_bytes(); n=struct.unpack('<I',data[80:84])[0]
    rec=np.frombuffer(data[84:],dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')]),count=n)
    segments=[]
    for tri in rec['vertices'].astype(float):
        vertices=[]
        for a,b in zip(tri,np.roll(tri,-1,axis=0)):
            da,db=a[2]-.1346,b[2]-.1346
            if da==0:vertices.append(a[:2])
            if da*db<0:vertices.append((a+(b-a)*da/(da-db))[:2])
        if len(vertices)>=2:
            segments.append([[9.8-v[1],6.145+v[0]] for v in vertices[:2]])
    t=12.22
    ego=[json.loads(line) for line in (ROOT/'run1/ego_poses.jsonl').read_text(encoding='utf8').splitlines()]
    pose=min(ego,key=lambda x:abs(x[0]-t))
    geometry=json.loads((HERE/'failure_geometry.json').read_text(encoding='utf8'))
    hit=next(r for r in geometry['representatives'] if r['seq']==563)['hit']['candidate']['points']
    match=[]
    for h in hit:
        x,y=h['xy_body_m'];c,s=math.cos(pose[3]),math.sin(pose[3]);point=np.array([pose[1]+c*x-s*y,pose[2]+s*x+c*y])
        def distance(segment):
            a,b=np.asarray(segment);delta=b-a;norm=delta@delta
            return float(np.linalg.norm(point-(a+np.clip((point-a)@delta/norm,0,1)*delta))) if norm else float(np.linalg.norm(point-a))
        segment=min(segments,key=distance)
        match.append({'ray_index':h['ray_index'],'world_xy_m':point.tolist(),'mesh_surface_distance_m':distance(segment),'nearest_mesh_section_segment':segment})
    model={'scope':'평가 pose/STL은 원인 위치 대조에만 사용; 제어기 입력/객체 인식으로 사용하지 않음',
        'model':'eval_r406_trial_old610_signal_support','mesh_sha256':hashlib.sha256(data).hexdigest(),
        'slice_height_m':.1346,'nearest_pose':pose,'reference_s':t,'hits':match,
        'limitation':'평면 pose와 수평 C1 단면 근접 대조. 광선별 3D 충돌 객체 ID 증명은 아님.'}
    (HERE/'facility_match.json').write_text(json.dumps(model,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print('plan/current',len(plan),len(current),'reasons',result['plan_reasons'],result['current_reasons'])
    print('publication',len(pubs),'positive',result['positive_route_publication_count'])
    print('post surface distance m',[h['mesh_surface_distance_m'] for h in match])
    print('actuation example',str(result['actuation_schema_example'])[:550])


if __name__=='__main__':main()
