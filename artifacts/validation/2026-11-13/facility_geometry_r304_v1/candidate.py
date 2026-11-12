"""단독 지주 가림 시험만의 선택형 연결. 공통 코드/기본 실행은 변경하지 않는다."""
import inspect,textwrap
from arena_autonomy import local_follow as lf
ORIGINAL=lf.LocalFollow._baseline_command

def install():
    source=inspect.getsource(ORIGINAL)
    old="if cfg.traffic_boundary_refresh_trial and info.get('wall_model') != 'piecewise_corner':"
    assert source.count(old)==1
    new="if (cfg.traffic_boundary_refresh_trial or (cfg.solo_route_only and cfg.narrow_occluder_bridge)) and info.get('wall_model') != 'piecewise_corner':"
    namespace={}
    exec(compile(textwrap.dedent(source.replace(old,new)),__file__,'exec'),lf.__dict__,namespace)
    lf.LocalFollow._baseline_command=namespace['_baseline_command']

def uninstall():lf.LocalFollow._baseline_command=ORIGINAL
