"""현재 입력의 경로 재검사. 명시 선택용 순수 함수이며 상태 전이/발행은 없다.

원 함수: 2026-11-12/released_parallel_r173_v1/current.py
이관/동등성 근거: current_recheck_module_r176_v1.
"""
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from . import _parallel as parallel
from . import _released as released


def surrogate(output, **fields):
    return output[0], output[1], {**output[2], 'normal': {**output[2].get('normal', {}), **fields}}


def supported(output):
    n = output[2].get('normal') or {}
    return (n.get('selector_mode') == 'RELEASED' and n.get('selector_choice') == 'parallel'
            and n.get('reason') == 'parallel_hold_follow'
            and parallel.supported(surrogate(output, selector_mode='SOLO'))
            and released.supported(surrogate(output, reason='local_follow', selector_choice='baseline')))


def rebase(output, submitted, current, scene, history, cfg):
    if not supported(output): return None
    # 적용 분류만 맞춘다. 실제 상태·선택 경로·원 시각·횡변위 제한은 바꾸지 않는다.
    result = parallel.rebase(surrogate(output, selector_mode='SOLO'), submitted, current, scene, history, cfg)
    if result is None: raise RuntimeError('released parallel geometry unexpectedly unsupported')
    result = {**result, 'rebased_selector_mode': 'RELEASED', 'state_transition': False,
              'source_permission_reused': False}
    if result['speed_mps'] <= 0: return result

    def stop(reason):
        return {**result, 'speed_mps': 0., 'steering_rad': current['previous_steering_rad'], 'reason': reason}

    scan = MeasurementScan(**thaw(current['scan']['measurements']))
    if not released.coverage.current_forward_coverage(scan, scene, history, cfg):
        return stop('released_parallel_current_front_not_covered')
    view = ps.threat_view(scene, result['path_points'], ps.lateral_config(cfg))
    if view is None: return stop('released_parallel_current_path_frame_missing')
    if any(view[name]['present'] for name, _ in ps.SIDES):
        return stop('released_parallel_current_side_present')
    return {**result, 'reason': 'rebase_current_released_parallel',
            'current_side_presence': {name:view[name]['present'] for name,_ in ps.SIDES}}
