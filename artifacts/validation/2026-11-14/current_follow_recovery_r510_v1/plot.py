from pathlib import Path
import json
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
A=Path(__file__).resolve().parent;data=json.loads((A/'diagnostic.json').read_text())
fig,axes=plt.subplots(1,3,figsize=(12,4))
for ax,key in zip(axes,(90,94,99)):
 r=next(x for x in data['comparisons'] if x['key'][1]==key)
 entry=r['original'][2].get('moving_pass',{}).get('entry',{})
 for c in entry.get('candidates',[]):
  path=np.array(c.get('path_points',[]))
  if len(path):ax.plot(path[:,0],path[:,1],'m-',label='pass guide')
 walls=entry.get('reference',{}).get('walls',{})
 for w in walls.values():
  x=np.array(w['observed_x_m']);ax.plot(x,x*w['slope']+w['intercept_m'],'k-')
 target=r['original'][2].get('moving_pass',{}).get('target',{}).get('observed_bounds_xy')
 if target:
  q=np.array(target);ax.plot(q[:,0],q[:,1],'ro',label='target bounds')
 ax.add_patch(plt.Rectangle((-.207/2,-.15/2),.207,.15,fill=False,color='blue'))
 for delta,color,label in ((r['original'][1],'orange','command'),(r['alternative'][1],'green','normal')):
  t=np.arange(0,1.21,.01);k=np.tan(delta)/.128
  x=np.sin(k*t)/k if abs(k)>1e-6 else t;y=(1-np.cos(k*t))/k if abs(k)>1e-6 else t*0
  ax.plot(x,y,color=color,label=label)
 ax.set_title(f'seq {key}, speed {r["original"][0]:.2f}');ax.set_xlim(-.2,1.6);ax.set_ylim(-.8,.8);ax.set_aspect('equal');ax.grid();ax.legend()
fig.tight_layout();fig.savefig(A/'guide_geometry.png',dpi=140)
