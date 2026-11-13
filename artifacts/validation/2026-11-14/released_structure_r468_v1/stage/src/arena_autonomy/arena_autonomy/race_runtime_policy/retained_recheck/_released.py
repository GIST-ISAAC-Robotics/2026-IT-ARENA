"""현재 입력의 경로 재검사. 명시 선택용 순수 함수이며 상태 전이/발행은 없다.

원 함수: 2026-11-12/released_current_r167_v1/released.py
이관/동등성 근거: current_recheck_module_r176_v1.
"""
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.local_follow import forward_planning_speed
from . import _geometry as base
from . import _guard as coverage


def supported(output):
    normal=output[2].get('normal') or {}
    sides=normal.get('selector_sides') or {}
    partial=normal.get('partial_observation') or {}
    return (base.finite(output[0]) and output[0]>0 and output[2].get('handoff_state')=='FOLLOW'
        and normal.get('reason')=='local_follow' and normal.get('selector_mode')=='RELEASED'
        and normal.get('selector_choice')=='baseline' and set(sides)=={'left','right'}
        and all(s.get('present') is False and type(s.get('seen')) is bool and type(s.get('passed')) is bool
                and s['seen']==s['passed']
                for s in sides.values())
        and any(s.get('seen') is True and s.get('passed') is True for s in sides.values())
        and partial.get('allowed') is True and partial.get('reason')=='fully_observed')


def rebase(output,submitted,current,scene,history,cfg):
    if not supported(output):return None
    normal=output[2]['normal']
    diagnostic=dict(rebased_selector_mode='RELEASED',state_transition=False,
        source_permission_reused=False,independent_guard_required=True,runtime_authorized=False)
    # 기존 순수 경로 계산의 적용 분류만 맞춘다. 선택기 내부 상태를 바꾸지 않는다.
    geometry_input=(output[0],output[1],{**output[2],'normal':{**normal,'selector_mode':'SOLO'}})
    result=base.rebase(geometry_input,submitted,current,scene,history,cfg)
    if result is None:raise RuntimeError('released geometry unexpectedly unsupported')
    if result['speed_mps']<=0:return {**result,**diagnostic}
    def stop(reason):
        return {**result,**diagnostic,'speed_mps':0.,'steering_rad':current['previous_steering_rad'],
                'reason':reason}
    scan=MeasurementScan(**thaw(current['scan']['measurements']))
    if not coverage.current_forward_coverage(scan,scene,history,cfg):
        return stop('released_rebase_current_front_not_covered')
    view=ps.threat_view(scene,result['path_points'],ps.lateral_config(cfg))
    if view is None:return stop('released_rebase_current_path_frame_missing')
    if any(view[name]['present'] for name,_ in ps.SIDES):
        return stop('released_rebase_current_side_present')
    body_gap = normal.get('selector_released_body_gap') or {}
    if body_gap.get('accepted') is True:
        if not cfg.released_body_gap_trial:
            return stop('released_rebase_body_gap_option_disabled')
        from arena_autonomy.released_body_gap import check
        checked = check(scene, scan, forward_planning_speed(cfg, current['speed_mps']),
            result['speed_mps'], result['steering_rad'], current['previous_steering_rad'],
            current['dt_s'], cfg, ps.lateral_config(cfg), ps.SelectorConfig())
        if not checked['accepted']:
            return {**stop('released_rebase_current_body_gap_rejected'), 'current_body_gap': checked}
        diagnostic['current_body_gap'] = checked
    return {**result,**diagnostic,'reason':'rebase_current_released',
            'current_side_presence':{name:view[name]['present'] for name,_ in ps.SIDES},
            'forward_coverage_only':True}
