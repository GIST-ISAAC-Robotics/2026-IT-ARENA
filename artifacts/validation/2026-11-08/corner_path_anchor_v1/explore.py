import sys, math, numpy as np
sys.path.insert(0,'src/arena_autonomy')
from arena_autonomy.local_path import local_path
def polyline(*v):
    return np.vstack([np.linspace(a,b,max(6,int(np.linalg.norm(np.array(b)-a)/.02)+1))[:-1] for a,b in zip(v,v[1:])]+[np.array(v[-1:])])
off=.505
for mirror in (1,-1):
    side='right' if mirror==1 else 'left'
    wall=polyline([-1.,-.505],[1.51,-.505],[1.51,3.])*[1,mirror]
    for deg in range(0,91,5):
        a=math.radians(deg)
        c=np.array([.5,.505*mirror]); pos=c+.505*np.array([math.sin(a), -mirror*math.cos(a)])
        h=mirror*a
        R=np.array([[math.cos(h),math.sin(h)],[-math.sin(h),math.cos(h)]])
        local=(wall-pos)@R.T
        path,info=local_path(local,side,off,piecewise_corner=True,minimum_radius=.347)
        print(mirror,deg,info.get('wall_model'),round(info.get('wall_distance_m',-1),3),info.get('path_anchor_error_m') and round(info['path_anchor_error_m'],4),round(info.get('boundary_observed_ahead_m',-1),3))
