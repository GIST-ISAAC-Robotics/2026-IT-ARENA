"""단독 시험용 실제 직선 조각 가설. 전체 점유/미관측은 변경하지 않는다."""
import inspect,textwrap
import numpy as np
from arena_autonomy import local_follow as lf
from arena_autonomy import fragmented_wall_hypothesis as fragments
ORIGINAL=lf.LocalFollow._baseline_command

def straight_path(joined,side,offset):
    # 앞뒤 실제 벽 조각 모두 같은 직선일 때만 사용한다. 코너/짧은 끝면을 직선으로 펴지 않는다.
    p=np.asarray(joined,dtype=float)
    if len(p)<24 or not np.all(np.isfinite(p)):return None
    center=p.mean(axis=0);t=np.linalg.svd(p-center,full_matrices=False)[2][0]
    if t[0]<0:t=-t
    n=np.array([-t[1],t[0]])
    if t[0]<np.cos(np.deg2rad(30)) or np.max(np.abs((p-center)@n))>.03:return None
    along=(p-center)@t
    wall=center+np.linspace(float(along.min()),float(along.max()),36)[:,None]*t
    path=wall-(1 if side=='left' else -1)*offset*n
    if path[-1,0]-max(0.,path[0,0])<.65:return None
    return path

def build_proposer():
    source=inspect.getsource(fragments.propose)
    old='path=corner_path(joined,side,offset,minimum_radius)\n            if path is None:continue'
    new='''path=corner_path(joined,side,offset,minimum_radius)
            shape='corner'
            if path is None:
                path=straight_path(joined,side,offset);shape='straight'
            if path is None:continue'''
    assert source.count(old)==1
    source=source.replace(old,new).replace('source_indices=ids.tolist(),','shape=shape,source_indices=ids.tolist(),')
    namespace={};exec(compile(source,__file__,'exec'),{**fragments.__dict__,'straight_path':straight_path},namespace)
    return namespace['propose']

propose=build_proposer()

def install():
    source=inspect.getsource(ORIGINAL)
    old="if cfg.traffic_boundary_refresh_trial and info.get('wall_model') != 'piecewise_corner':"
    assert source.count(old)==1
    source=source.replace(old,"if (cfg.traffic_boundary_refresh_trial or (cfg.solo_route_only and cfg.narrow_occluder_bridge)) and info.get('wall_model') != 'piecewise_corner':")
    source=source.replace('from arena_autonomy.fragmented_wall_hypothesis import propose',
        'from arena_autonomy.fragmented_wall_hypothesis import propose\n                    if cfg.solo_route_only:propose=_straight_fragment_trial')
    source=source.replace("wall_model='observed_fragment_corner'","wall_model='observed_fragment_'+evidence.get('shape','corner')")
    namespace={};exec(compile(textwrap.dedent(source),__file__,'exec'),{**lf.__dict__,'_straight_fragment_trial':propose},namespace)
    lf.LocalFollow._baseline_command=namespace['_baseline_command']

def uninstall():lf.LocalFollow._baseline_command=ORIGINAL
