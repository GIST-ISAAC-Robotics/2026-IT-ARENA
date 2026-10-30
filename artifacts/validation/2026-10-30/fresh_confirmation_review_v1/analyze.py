"""기존 후보를 바꾸지 않는 계측: 점 이동·운동 보정·재개 거절을 저장 입력에서 분리한다."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
OLD = BASE.parent/'fresh_confirmation_memory_v1'
sys.path[:0] = [str(OLD), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import numpy as np
from candidate_state import FreshMemoryParallelTrial
from arena_autonomy.follow_trace import replay_call, restore_call, freeze, comparable_decision, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow, FollowConfig


class Instrumented(FreshMemoryParallelTrial):
    def _classify(self, scene, baseline, speed, previous, dt, result, now):
        info = super()._classify(scene, baseline, speed, previous, dt, result, now)
        row = dict(t=now, source=scene.source_first_s, speed=speed, previous=previous,
                   steering=result.get('steering_rad'), **info)
        record = self.confirmed
        if record is not None:
            p = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
            q = record['points']
            if p.shape == q.shape and len(p):
                delta = np.linalg.norm(p-q, axis=1)
                index = int(np.argmax(np.max(np.abs(p-q), axis=1)))
                near = np.linalg.norm(q, axis=1) < .5
                # Paired-ray SE(2) fit is diagnostic only; never fed into the controller.
                qa, pa = q-q.mean(axis=0), p-p.mean(axis=0)
                u, _, vt = np.linalg.svd(qa.T@pa)
                correction = np.eye(2)
                correction[-1, -1] = np.linalg.det(u@vt)
                rotation = u@correction@vt
                translation = p.mean(axis=0)-q.mean(axis=0)@rotation
                residual = np.linalg.norm(q@rotation+translation-p, axis=1)
                row.update(max_component_m=float(np.max(np.abs(p-q))),
                           delta_quantiles_m=np.quantile(delta, [0, .5, .9, 1]).tolist(),
                           extreme_point_range_m=float(np.linalg.norm(q[index])),
                           near_points=int(near.sum()),
                           near_max_component_m=float(np.max(np.abs((p-q)[near]))) if near.any() else None,
                           fit_yaw_rad=float(math.atan2(rotation[0, 1], rotation[0, 0])),
                           fit_translation_m=translation.tolist(),
                           fit_max_residual_m=float(residual.max()))
                # Compare to independent propagation of the recorded motor/gyro history.
                poses, _ = self.history.poses([record['t_ref'], now], now)
                a, b = poses
                angle = a[2]-b[2]
                c, s = math.cos(angle), math.sin(angle)
                r = np.array([[c, s], [-s, c]])
                c, s = math.cos(b[2]), math.sin(b[2])
                shift = (a[:2]-b[:2])@np.array([[c, -s], [s, c]])
                predicted = q@r+shift
                row.update(history_yaw_rad=float(angle), history_translation_m=shift.tolist(),
                           history_max_residual_m=float(np.linalg.norm(p-predicted, axis=1).max()))
        self.classifications.append(row)
        return info

    def apply(self, scene, baseline, speed, previous, dt):
        fresh = self.last_source is None or scene.source_first_s > self.last_source
        out = super().apply(scene, baseline, speed, previous, dt)
        trial = out[2].get('parallel_hold_trial') or {}
        self.calls.append(dict(t=scene.t_ref_s, fresh=fresh, speed=speed, command=out[0],
                               reason=out[2]['reason'], eligible=trial.get('eligible'),
                               clearance=trial.get('clearance_m'), confirmations=self.confirmations))
        return out


def main(path, output):
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    stats, selected_zero, inner_zero = Counter(), Counter(), Counter()
    foot = None
    rows = []
    with path.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        motion = MotionConfig(**header['motion_config'])
        cfg = replace(FollowConfig(**header['follow_config']), rear_lane_keep_trial=True,
                      rear_lane_keep_policy='observed_measured_v1')
        reference, diagnostic = LocalFollow(cfg), LocalFollow(cfg)
        reference.parallel_trial = FreshMemoryParallelTrial(cfg)
        diagnostic.parallel_trial = Instrumented(cfg)
        reference.reset(); diagnostic.reset()
        inst = diagnostic.parallel_trial
        inst.classifications, inst.calls = [], []
        for seq, line in enumerate(stream, 1):
            event = json.loads(line)
            assert foot is None and event['seq'] == seq, 'invalid sequence'
            if event['kind'] == 'footer':
                foot = event['data']; continue
            if event['kind'] == 'reset':
                reference.reset(); diagnostic.reset(); continue
            if event['kind'] != 'call':
                continue
            data = event['data']['input']
            _, _, inst.history = restore_call(data, motion)
            a = freeze(replay_call(reference, data, motion))
            b = freeze(replay_call(diagnostic, data, motion))
            stats['calls'] += 1
            stats['instrumentation_matches'] += equal_output(comparable_decision(a), comparable_decision(b))
            stats['matches_recorded_candidate'] += equal_output(a[:2], event['data']['output'][:2])
            if a[0] == 0:
                selected_zero[str(a[2].get('reason'))] += 1
            rows.append(dict(t=data['reference_s'], speed=data['speed_mps'], command=a[0],
                             reason=a[2].get('reason'), selector=a[2].get('selector_choice')))
    assert foot is not None and foot.get('capture_errors', 0) == 0
    for x in inst.calls:
        if x['command'] == 0:
            inner_zero[('fresh:' if x['fresh'] else 'reused:')+x['reason']] += 1
    failed = [x for x in inst.classifications if x.get('checks', {}).get('same_points') is False]
    def extent(key):
        v = [x[key] for x in failed if x.get(key) is not None]
        return [min(v), float(np.median(v)), max(v)] if v else None
    output.mkdir(exist_ok=False)
    report = dict(input=str(path.relative_to(ROOT)), input_sha256=digest,
                  unchanged=digest == hashlib.sha256(path.read_bytes()).hexdigest(),
                  stats=dict(stats), selected_zero=dict(selected_zero), inner_zero=dict(inner_zero),
                  same_points_failed=len(failed),
                  failed_with_zero_speed=sum(abs(x['speed']) < 1e-9 for x in failed),
                  failed_near_under_5mm=sum(x.get('near_max_component_m', 1) <= .005 for x in failed),
                  ranges_min_median_max={k: extent(k) for k in ('max_component_m', 'extreme_point_range_m',
                      'near_max_component_m', 'fit_yaw_rad', 'fit_max_residual_m', 'history_max_residual_m')},
                  source_hashes={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (Path(__file__), OLD/'candidate_state.py', OLD/'candidate_speed.py')},
                  footer=foot, scope='instrumentation of fixed input; no policy or trajectory change')
    report['passed'] = report['unchanged'] and stats['calls'] == stats['instrumentation_matches']
    (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    for name, values in [('classifications', inst.classifications), ('inner_calls', inst.calls), ('selected', rows)]:
        with (output/(name+'.jsonl')).open('w', encoding='utf-8') as stream:
            for row in values:
                stream.write(json.dumps(row)+'\n')
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('input', type=Path); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); raise SystemExit(main(args.input.resolve(), args.output.resolve()))
