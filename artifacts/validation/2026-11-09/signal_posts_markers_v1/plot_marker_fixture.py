"""확정 시설 여부와 별개의 임시 위치를 평면 기하로 확인하는 그림."""
import json,sys,math
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path.insert(0,str(R/'scripts'))
from surface_contact_eval import SurfaceMap,rectangle
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch
W=R/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
s=SurfaceMap.from_world(W);meta=json.loads((A/'marker_posts_provisional.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(10,5),layout='constrained')
def draw(ax,geom,color,z):
    for poly in getattr(geom,'geoms',[geom]):
        if poly.is_empty or not hasattr(poly,'exterior'):continue
        ax.add_patch(Patch(list(poly.exterior.coords),closed=True,facecolor=color,edgecolor='none',zorder=z))
for ax,(marker,x,y,yaw) in zip(axes,[(20,6.155,6.,0.),(30,2.48,3.505,math.pi/2)]):
    region=rectangle(x,y,0,2.,2.)
    draw(ax,s.grass.intersection(region),'#70a37f',1)
    draw(ax,s.road.intersection(region),'#d9d9d5',2)
    draw(ax,s.walls.intersection(region),'#484b50',3)
    for row in meta['links']:
        if row['id']!=marker:continue
        draw(ax,rectangle(*row['center'],row['yaw'],row['length'],row['width']),
             '#e98c29' if row['part']=='foot' else '#ae231c',5 if row['part']=='foot' else 6)
        if row['part']=='post':ax.annotate('post',row['center'],xytext=(8,8),textcoords='offset points',fontsize=8)
    ax.plot([x],[y],'bo',ms=6,zorder=8,label='original marker board (above ground)')
    if marker==20:
        ax.plot([6.5275,x],[y,y],'b--',lw=1,zorder=7)
        ax.plot([x],[5.6275],'rx',ms=9,zorder=8,label='discarded same-plane post (on road)')
    ax.set(xlim=(x-.9,x+.9),ylim=(y-.9,y+.9),aspect='equal',xlabel='world x [m]',ylabel='world y [m]',title=f'ID {marker}: provisional support placement')
    ax.legend(loc='lower left',fontsize=7);ax.grid(alpha=.15)
fig.suptitle('Presence confirmed by user; support shape and positions remain provisional',fontsize=11)
fig.savefig(A/'marker_post_layout.png',dpi=160)
print('marker_post_layout.png')
