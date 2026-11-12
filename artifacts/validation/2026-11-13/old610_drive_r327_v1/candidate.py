"""r306/r309 경계 후보 + 단일 경로 선택 지점. r313/r316 PP wrapper는 연결하지 않는다."""
import importlib.util
import inspect
import textwrap
from pathlib import Path
import arena_autonomy.local_follow as lf
import arena_autonomy.local_path as lp
import geometry

A=Path(__file__).resolve().parent
ORIGINAL=lf.LocalFollow._baseline_command
BREAK_ORIGINAL=lp.break_adjacent_occluders
spec=importlib.util.spec_from_file_location('r317_boundary_dependency', A.parent/'corner_window_r309_v1/candidate.py')
prior=importlib.util.module_from_spec(spec); spec.loader.exec_module(prior)


def execution_path(owner, scene, path, speed, age, speed_cap):
    cfg=owner.config
    if not cfg.solo_route_only or cfg.occupancy_timing_mode!='per_ray_candidate':
        return path, {}
    shifted, detail=geometry.choose(path, scene.occupied_endpoints_xy_m,
                                   geometry.selection_padding(scene,cfg), cfg, speed, age, speed_cap=speed_cap)
    # 변경된 경로는 현재 명령 선택에만 사용한다. 벽 이력/원 시간/현재 점유를 덮지 않는다.
    return shifted, detail


def install():
    source=textwrap.dedent(inspect.getsource(ORIGINAL))
    old="if cfg.traffic_boundary_refresh_trial and info.get('wall_model') != 'piecewise_corner':"
    assert source.count(old)==1
    source=source.replace(old,"if (cfg.traffic_boundary_refresh_trial or (cfg.solo_route_only and cfg.narrow_occluder_bridge)) and info.get('wall_model') != 'piecewise_corner':")
    source=source.replace('from arena_autonomy.fragmented_wall_hypothesis import propose',
        'from arena_autonomy.fragmented_wall_hypothesis import propose\n                if cfg.solo_route_only:propose=_straight_fragment_trial')
    source=source.replace("wall_model='observed_fragment_corner'","wall_model='observed_fragment_'+evidence.get('shape','corner')")
    anchor='        cap, desired, remaining = pursue_path(carried, speed, cfg, max(age, memory_age),'
    assert source.count(anchor)==1
    source=source.replace(anchor,
        '        carried, shift_info = _facility_execution_path(self, scene, carried, speed, max(age,memory_age), cruise if open_route else None)\n'
        "        if shift_info:detail['facility_path_offset']=shift_info\n"+anchor)
    anchor2='        if cruise:\n'
    assert source.count(anchor2)==1
    source=source.replace(anchor2,
        "        if shift_info.get('applied'):cap=min(cap,shift_info['original_cap_mps'])\n"+anchor2)
    namespace={}
    exec(compile(source,str(A/'baseline_source.py'),'exec'),
         {**lf.__dict__,'_straight_fragment_trial':prior.straight.propose,
          '_facility_execution_path':execution_path},namespace)
    lf.LocalFollow._baseline_command=namespace['_baseline_command']
    lp.break_adjacent_occluders=prior.break_adjacent_occluders
    (A/'baseline_source.py').write_text(source,encoding='utf-8')


def uninstall():
    lf.LocalFollow._baseline_command=ORIGINAL
    lp.break_adjacent_occluders=BREAK_ORIGINAL
