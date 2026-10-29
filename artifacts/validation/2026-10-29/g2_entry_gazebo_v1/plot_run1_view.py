"""run1 궤적 보기용 그림. 진행 방향을 위로, 왼쪽을 왼쪽에 둔다(원본 run1_path.png는 좌우가 반대로 표시됨). 평가 전용."""
import json,sys
from pathlib import Path
import numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(BASE));sys.path.insert(0,str(ROOT/'scripts'))
import g2_eval
import g1_traffic_eval as ev
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
run=BASE/'run1';config=json.loads((BASE/'g2_config_v1.json').read_text(encoding='utf-8'))
result=json.loads((BASE/'run1_g2_evaluation.json').read_text(encoding='utf-8'))
world=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'
route=np.genfromtxt(world/'centerline.csv',delimiter=',',names=True)
scene=json.loads((world/'scene.json').read_text(encoding='utf-8'))
cl=ev.Centerline(route['x_m'],route['y_m'],route['s_m'],scene['track']['lap_length_m'])
s0=config['launch']['g2_spawn']['s_m'];green=result['green_sim_s']
rows=[]
for t,x,y,yaw in g2_eval.jsonl(run/'ego_poses.jsonl'):
    if green<=t<=result['stop_end_sim_s']:
        s,lat,_=g2_eval.signed_lateral(cl,x,y);rows.append((t-green,cl.unwrap(s,s0)-s0,lat))
rows=np.array(rows)
fig,ax=plt.subplots(figsize=(4.2,8))
ax.axvspan(-.425,-.225,color='tab:green',alpha=.15);ax.axvspan(.225,.425,color='tab:green',alpha=.15)
ax.plot(-rows[:,2],rows[:,1],lw=1.4,label='ego centre (truth)')
t=result['target'];_,tl,_=g2_eval.signed_lateral(cl,t['x'],t['y'])
ax.add_patch(plt.Rectangle((-tl-.075,t['rear_s']-s0),.15,t['front_s']-t['rear_s'],color='tab:red',alpha=.6,label='stationary target'))
for label,when,mark in (('pass',result['pass_time_since_green_s'],'o'),('centred',result['center_return_first_time_since_green_s'],'s')):
    i=int(np.argmin(abs(rows[:,0]-when)));ax.plot(-rows[i,2],rows[i,1],mark,color='k',ms=5)
    ax.annotate(f'{label} {when:.2f} s',(-rows[i,2],rows[i,1]),xytext=(6,-4),textcoords='offset points',fontsize=7)
i=int(np.argmin(abs(rows[:,0]-(13.43-green))));ax.plot(-rows[i,2],rows[i,1],'v',color='tab:orange',ms=6)
ax.annotate('slowdown 0.08 m/s\n(grass->road edge)',(-rows[i,2],rows[i,1]),xytext=(-95,0),textcoords='offset points',fontsize=7)
ax.set_xlim(-.45,.45);ax.set_xlabel('lateral [m] (left <- | -> right, viewed in travel direction)');ax.set_ylabel('along road from start [m]')
ax.set_title('G2 Gazebo run1: right-edge stationary target',fontsize=9);ax.legend(loc='lower right',fontsize=7)
fig.tight_layout();fig.savefig(BASE/'run1_path_view.png',dpi=120)
print('ok')
