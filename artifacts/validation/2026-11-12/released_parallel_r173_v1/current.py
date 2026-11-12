"""통과 뒤에도 현재 차로를 유지하는 RELEASED/parallel의 현재 재계산."""
import importlib.util
from pathlib import Path
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan

ROOT = Path(__file__).resolve().parents[4]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


parallel = load('r173_parallel', ROOT/'artifacts/validation/2026-11-11/guard_recheck_r139_v1/rebase.py')
released = load('r173_released', ROOT/'artifacts/validation/2026-11-12/released_current_r167_v1/released.py')


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
