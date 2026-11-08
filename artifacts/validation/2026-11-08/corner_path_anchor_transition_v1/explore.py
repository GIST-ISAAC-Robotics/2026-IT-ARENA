import math, sys
import numpy as np
sys.path.insert(0, "src/arena_autonomy")
from arena_autonomy.local_path import local_path
O,R=.505,.347
def polyline(*v):
    return np.vstack([np.linspace(a,b,max(6,int(np.linalg.norm(np.array(b)-a)/.02)+1))[:-1] for a,b in zip(v,v[1:])]+[np.array(v[-1:])])
def rot(cloud, x, y, h):
    c,s=math.cos(h),math.sin(h); return (cloud-[x,y])@np.array([[c,s],[-s,c]]).T
for mirror in (1,-1):
  side='right' if mirror==1 else 'left'
  for deg in (0,30,40,45):
    for lat in (0,-.2,.2):
      for clip in (None,1.0):
        wall=polyline([-1.,-.505],[1.51,-.505],[1.51,3.])
        if clip: wall=wall[wall[:,0]<=clip]
        wall=wall*[1,mirror]
        h=mirror*math.radians(deg)
        cl=rot(wall, -.3, -mirror*lat, h)
        p,i=local_path(cl, side, O, piecewise_corner=True, minimum_radius=R)
        print(mirror,deg,lat,clip,i.get('reason'),i.get('wall_model'),None if p is None else round(i['wall_distance_m'],3),None if p is None else round(i['path_anchor_error_m'],4), None if p is None else round(i['boundary_observed_ahead_m'],2))
