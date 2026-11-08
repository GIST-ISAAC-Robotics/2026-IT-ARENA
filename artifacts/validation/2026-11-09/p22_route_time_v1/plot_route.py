"""사후 평가용 실제 진입 궤적. 지도/정답은 제어 코드에 전달하지 않는다."""
from pathlib import Path
import json,math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch
from matplotlib.patches import PathPatch
from matplotlib.path import Path as PlotPath
from shapely.geometry.polygon import orient
import numpy as np
from surface_contact_eval import SurfaceMap

A=Path(__file__).resolve().parent;D=A.parent/'p22_route_connected_v1/run1'
surface=SurfaceMap.from_world(D/'runtime_world.sdf')
poses=np.array([json.loads(l) for l in (D/'ego_poses.jsonl').open()])
fig,ax=plt.subplots(figsize=(9,7),dpi=150)
for geometry,color in ((surface.grass,'#d6e9ce'),(surface.road,'#eeeae2'),(surface.walls,'#34454b')):
    for poly in getattr(geometry,'geoms',[geometry]):
        if poly.geom_type!='Polygon':continue
        poly=orient(poly,sign=1.)
        vertices=[];codes=[]
        for ring in (poly.exterior,*poly.interiors):
            v=list(ring.coords);vertices.extend(v)
            codes.extend([PlotPath.MOVETO]+[PlotPath.LINETO]*(len(v)-2)+[PlotPath.CLOSEPOLY])
        ax.add_patch(PathPatch(PlotPath(vertices,codes),facecolor=color,edgecolor='none'))
summary=json.loads((D.parent/'live_summary.json').read_bytes())
start=summary['first_admitted_s'];end=summary['evaluation_end_s']
mask=(poses[:,0]>=start-.5)&(poses[:,0]<=end)
ax.plot(poses[mask,1],poses[mask,2],color='#c04715',lw=2,label='Recorded body center')
for t in [v for v in (start,summary['exit_selected_s'],summary['first_body_exit_s'],summary['main_handoff_s'],end) if v is not None]:
    p=poses[np.argmin(abs(poses[:,0]-t))];_,x,y,h=p;c,s=math.cos(h),math.sin(h)
    corners=np.array([(x+c*dx-s*dy,y+s*dx+c*dy) for dx,dy in ((.1,.075),(.1,-.075),(-.1,-.075),(-.1,.075))])
    ax.add_patch(Patch(corners,facecolor='#dc7633',edgecolor='#5d2310',alpha=.5))
    ax.annotate(f'{t:.3f} s',(x,y),xytext=(8,9),textcoords='offset points',fontsize=9)
ax.plot([5.635]*2,[4.2,4.8],'--',color='#7b2cbf',lw=1.5,label='Entrance wall plane (evaluation only)')
ax.set(xlim=(3.5,6.8),ylim=(3.9,7.0),xlabel='World x [m]',ylabel='World y [m]',title='P22: recorded entry-exit trial (single vehicle)')
ax.set_aspect('equal');ax.legend(loc='upper left',fontsize=8);ax.grid(alpha=.15)
fig.tight_layout();fig.savefig(A/'recorded_route.png');plt.close(fig)
