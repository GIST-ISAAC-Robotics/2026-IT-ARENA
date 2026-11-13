"""비용 계측 종료 뒤 index9 한 입력만 관찰한다. 원 함수 출력은 그대로 반환한다."""
import argparse
import contextlib
import hashlib
import io
import json
from pathlib import Path
import platform
import sys
import sysconfig

from support import A, load_comparison, verify_manifest, check_imports
from diff_utils import comparison
from run import load_r380


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timing', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    assert output.is_relative_to(A) and not output.exists()
    output.mkdir()
    verify_manifest()
    compare = load_comparison()
    imports = check_imports(compare, 'native_v1')
    import numpy as np
    from arena_autonomy import point_set, observed_lateral_trial, pass_release_trial
    record = load_r380().records_in_original_order(compare)[9]
    reference = None
    with args.timing.open(encoding='utf8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['phase'] == 'stored' and row['index'] == 9:
                reference = row
                break
    assert reference is not None, 'main diagnostic must reach index9 first'
    events, arrays = [], {}

    def array(name, value):
        data = np.asarray(value).copy()
        key = '%03d_%s' % (len(arrays), name)
        arrays[key] = data
        info = dict(key=key, shape=list(data.shape), dtype=str(data.dtype),
                    bytes_sha256=hashlib.sha256(np.ascontiguousarray(data).tobytes()).hexdigest())
        if np.issubdtype(data.dtype, np.floating):
            info['negative_zero_count'] = int(np.sum((data == 0) & np.signbit(data)))
        return info

    original_unique = point_set.unique_xy
    original_bound = observed_lateral_trial.unique_xy
    original_motion = pass_release_trial.motion_queries
    original_support = pass_release_trial.depth_and_masked_support

    def unique(points):
        result = original_unique(points)
        before = np.asarray(points)
        info = dict(stage='unique_xy', input=array('unique_input', before), output=array('unique_output', result),
                    exact_removed=int(len(before)-len(result)))
        if result.ndim == 2 and result.shape[1] == 2 and len(result) > 1:
            # 후처리 관찰값이다. 반올림한 점을 제어기/분류기로 돌려보내지 않는다.
            info['offline_rounded_unique_counts'] = {str(n):int(len(np.unique(np.round(result, n), axis=0))) for n in (12,14,16)}
            distance = np.max(np.abs(np.diff(result, axis=0)), axis=1)
            info['adjacent_positive_linf_at_most_1e_14'] = int(np.sum((distance > 0) & (distance <= 1e-14)))
        events.append(info)
        return result

    def motion(scene, speed, target, steering, previous, dt, follow_cfg, lat_cfg):
        result = original_motion(scene, speed, target, steering, previous, dt, follow_cfg, lat_cfg)
        distance, points = result
        margins = np.maximum(np.abs(points[:,0])-follow_cfg.length_m/2-1e-9,
                             np.abs(points[:,1])-follow_cfg.width_m/2-1e-9)
        events.append(dict(stage='motion_queries', speed_mps=speed,target_mps=target,
            steering_rad=steering,previous_steering_rad=previous,dt_s=dt,
            age_s=scene.t_ref_s-scene.source_first_s,distance_m=distance,queries=array('motion_queries',points),
            minimum_absolute_outside_margin_m=float(np.min(np.abs(margins))) if len(margins) else None,
            at_most_1e_14_from_outside_boundary=int(np.sum(np.abs(margins)<=1e-14))))
        return result

    def support(scene, points, padding):
        result = original_support(scene, points, padding)
        item = dict(stage='depth_and_masked_support', queries=array('classified_queries', points),
                    returned_none=result is None, padding=compare.freeze(padding))
        if result is not None:
            item['free'] = array('free',result[0]);item['masked'] = array('masked',result[1])
            item['free_count'] = int(np.sum(result[0]));item['masked_count'] = int(np.sum(result[1]))
        events.append(item)
        return result

    point_set.unique_xy = unique
    observed_lateral_trial.unique_xy = unique
    pass_release_trial.motion_queries = motion
    pass_release_trial.depth_and_masked_support = support
    error = None
    try:
        before, after = compare.evaluate(record)
    except Exception as exc:
        error = repr(exc)
        raise
    finally:
        point_set.unique_xy = original_unique
        observed_lateral_trial.unique_xy = original_bound
        pass_release_trial.motion_queries = original_motion
        pass_release_trial.depth_and_masked_support = original_support
        np.savez_compressed(output/'intermediate_arrays.npz', **arrays)
        with contextlib.redirect_stdout(io.StringIO()) as config:
            np.show_config()
        report = dict(index=9, reference_s=record['current']['reference_s'], error=error,
            events=events, numpy_version=np.__version__, numpy_configuration=config.getvalue(),
            python=sys.version, machine=platform.machine(), libc=platform.libc_ver(),
            compiler=sysconfig.get_config_var('CC'), cflags=sysconfig.get_config_var('CFLAGS'), imports=imports,
            measurement_included=False, output_unchanged_by_instrumentation=None,
            cause_confirmed=False, reason='ARM query-generation evidence only; matched x86 intermediate capture is absent',
            offline_rounding_used_by_controller=False)
        if error is None:
            report['baseline_vs_uninstrumented'] = comparison(reference['before'], before)
            report['candidate_vs_uninstrumented'] = comparison(reference['after'], after)
            report['output_unchanged_by_instrumentation'] = all(report[k]['exact_tree_equal'] for k in ('baseline_vs_uninstrumented','candidate_vs_uninstrumented'))
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    verify_manifest()
    assert report['output_unchanged_by_instrumentation'], 'probe changed output'
    print(json.dumps(dict(index=9, events=len(events), arrays=len(arrays), output_unchanged=True, cause_confirmed=False)))


if __name__ == '__main__': main()
