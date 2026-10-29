"""저장 입력에서 실제 관측점과 선호 경계 기준 경로를 표시한다. 지도 사용 없음."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

BASE = Path(__file__).resolve().parent
source = BASE.parent/'traffic_release_fringe_v1/boundary_hold_geometry.json'
result = next(r for r in json.loads(source.read_text())['results'] if abs(r['reference_s']-130.)<1e-7)
points = np.asarray(result['points'])
geometry = result['candidates']['left']['corridor']
path = np.asarray(geometry['path'])
indices = sorted({i for w in geometry['witnesses'] for i in w['source_point_indices']})
fig, ax = plt.subplots(figsize=(8,7),dpi=150)
ax.scatter(points[:,0],points[:,1],s=8,c='#6d7b8d',label='All measured occupied endpoints')
ax.scatter(points[indices,0],points[indices,1],s=18,c='#de7c20',label='Short left-side support (identity unknown)')
own = np.asarray(result['candidates']['right']['path'])
ax.plot(own[:,0],own[:,1],color='#2067ce',lw=1.5,ls='--',label='Path from preferred RIGHT boundary')
ax.plot(path[:,0],path[:,1],color='#126b48',lw=2.5,label='Shared supported prefix (geometry only)')
ax.add_patch(Rectangle((-.1,-.075),.2,.15,facecolor='#222a38',edgecolor='black',label='Vehicle footprint'))
ax.annotate('',xy=(.32,0),xytext=(.10,0),arrowprops=dict(arrowstyle='->',lw=1.8,color='#222a38'))
ax.set(xlim=(-.55,1.15),ylim=(-.7,1.2),xlabel='Forward x (m)',ylabel='Left y (m)',
       title='Recorded 130.0 s: long preferred RIGHT wall, short LEFT inner surface')
ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(loc='upper left',fontsize=8)
fig.text(.04,.015,'Sensor coordinates only. No map / actor truth. This plot does not certify clearance or driving.',fontsize=8)
fig.tight_layout(rect=(0,.04,1,1))
output=BASE/'boundary_roles_130s.png'
if output.exists():
    raise FileExistsError(output)
fig.savefig(output)
print(output)
