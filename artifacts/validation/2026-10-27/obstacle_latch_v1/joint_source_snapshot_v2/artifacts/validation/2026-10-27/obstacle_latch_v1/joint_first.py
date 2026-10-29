"""최초 래치 입력의 교집합 검사. 원래 판정과 원시 입력은 바꾸지 않는다."""
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from probe_joint_missing_sweep import joint_missing_sweep

OUT=BASE/'joint_first_v1.json'
if OUT.exists():raise FileExistsError(OUT)
source=BASE/'diagnosis_v1.json'
data=json.loads(source.read_text(encoding='utf-8'))
trace=BASE.parent/'short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
with trace.open() as stream:
    for line in stream:
        row=json.loads(line)
        if row['kind']!='call':continue
        partial=row['data']['output'][2].get('partial_observation') or {}
        if partial.get('latch_evidence'):
            controller_evidence=partial['latch_evidence']
            half_angle=row['data']['input']['scan']['angle_increment']/2
            break
fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
results={}
for ax,name,report,e in zip(axes,('controller','safety'),
                          (data['first_controller'],data['first_safety']),
                          (controller_evidence,data['first_safety_witnesses'])):
    steering=e['steering_rad']
    points=np.array([w['previous_point_now_xy'] for w in e['witnesses']])
    rays={w['source_index']:w for w in e['witnesses']}
    origins=np.array([w['origin_xy'] for w in rays.values()])
    directions=np.array([w['direction_rad'] for w in rays.values()])
    radius=.03+2*(e['reference_s']-e['previous_reference_s'])
    distances,polygons=joint_missing_sweep(points,origins,directions,half_angle,
        report['history_distance_m'],e['sweep_speed_mps'],e['reference_s']-e['source_first_s'],
        steering,e['previous_steering_rad'])
    results[name]=dict(radius_m=radius,minimum_joint_distances_m=distances.tolist(),
        retained= (distances<=radius).tolist(),steering_rad=steering,
        previous_steering_rad=e['previous_steering_rad'],joint_polygons=len(polygons))
    for polygon in polygons:
        p=np.array(polygon);ax.fill(p[:,0],p[:,1],color='#0072b2',alpha=.2)
    for point,distance in zip(points,distances):
        ax.add_patch(plt.Circle(point,radius,fill=False,edgecolor='#d55e00',alpha=.45))
        ax.plot(*point,'x',color='#d55e00')
    for origin,direction in zip(origins,directions):
        v=np.array([math.cos(direction),math.sin(direction)])
        ax.plot(*np.array([origin,origin+v*.6]).T,color='#666666',ls='--')
    ax.set(title=name,xlim=(-.1,.85),ylim=(-.15,.85),aspect='equal',xlabel='Forward (m)',ylabel='Left (m)')
    ax.grid(alpha=.2)
fig.suptitle('Blue: missing wedge AND stopping sweep; orange: same old-point motion budget')
fig.savefig(BASE/'joint_first.png',dpi=160)
result=dict(scope='sensor-derived shadow geometry only; original failure remains',results=results,
    source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (source,trace,Path(__file__),ROOT/'scripts/probe_joint_missing_sweep.py')})
OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result['results']))
