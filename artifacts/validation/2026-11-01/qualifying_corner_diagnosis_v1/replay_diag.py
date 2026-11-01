"""저장 판단 입력을 실행 당시 소스 사본으로 재생해 원인 가설을 분리한다.

정답 속도는 '이 입력이 맞았다면 판단이 어떻게 달랐을까'를 보는 진단에만 쓴다.
제어기 소스나 기존 실행 산출물은 바꾸지 않는다. 재생은 기록된 궤적을 그대로
쓰는 개방 루프이므로 차량 궤적 변화나 완주를 증명하지 않는다.

실행: python3 -I replay_diag.py <run> <snapshot_root> <experiment> <t_start> <out.json>
"""
import bisect
import gzip
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RUNS = {'combined_failed': ROOT / 'qualifying_profile_v1/run1',
        'rise_only': ROOT / 'race_acceleration_v1/run1',
        'turn_only': ROOT / 'turn_preview_v1/run4'}


def setup(snapshot):
    snap = Path(snapshot)
    for pkg in ('arena_autonomy', 'arena_vehicle_interface'):
        sys.path.insert(0, str(snap / 'src' / pkg))


def truth_series(name):
    with gzip.open(HERE / f'extract_{name}.json.gz', 'rt', encoding='utf-8') as f:
        rows = json.load(f)['trajectory']
    pairs = sorted({(r['sim_time_s'], r['truth_longitudinal_mps']) for r in rows})
    return [p[0] for p in pairs], [p[1] for p in pairs]


def interp(ts, vs, t):
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
    return vs[i - 1] + f * (vs[i] - vs[i - 1])


def mutate(data, experiment, t0, ts, vs):
    """data['input']의 사본 복제를 피하고 그 자리에서 바꾼다(줄마다 새로 파싱됨)."""
    i = data['input']
    if experiment == 'baseline':
        return
    if experiment in ('truth_history', 'truth_both'):
        w = i['history']['wheels']
        w['values'] = [interp(ts, vs, t) if t >= t0 else v for t, v in zip(w['times'], w['values'])]
    if experiment in ('truth_speed', 'truth_both') and i['reference_s'] >= t0:
        i['speed_mps'] = interp(ts, vs, i['reference_s'])


def main():
    run, snapshot, experiment, t0, out = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), sys.argv[5]
    setup(snapshot)
    import numpy as np
    from arena_autonomy import corridor_observer as co
    from arena_autonomy import local_follow as lf
    from arena_autonomy.follow_trace import replay_call
    from arena_autonomy.lidar_motion import MotionConfig
    ts, vs = truth_series(run)
    captured = {}
    original_eval = co.CorridorObserver.evaluate

    def spy_eval(self, *a, **k):
        res = original_eval(self, *a, **k)
        captured['corridor'] = {k2: res.get(k2) for k2 in (
            'reason', 'fresh', 'confirmations', 'release_count', 'previous_path_agrees',
            'transport_displacement_m', 'transport_error', 'previous_source_age_s', 'positive',
            'geometry_supported', 'current_point_occupancy_supported')}
        return res
    co.CorridorObserver.evaluate = spy_eval
    original_agree = lf.path_agrees

    def spy_agree(candidate, reference, tolerance):
        res = original_agree(candidate, reference, tolerance)
        c, r = np.asarray(candidate), np.asarray(reference)
        cp = c[(c[:, 0] >= .05) & (c[:, 0] <= .65)]
        rp = r[(r[:, 0] >= .05) & (r[:, 0] <= .65)]
        err = None
        if len(cp) and len(rp):
            d = np.linalg.norm(cp[:, None, :] - rp[None, :, :], axis=2)
            err = float(max(np.percentile(d.min(axis=0), 80), np.percentile(d.min(axis=1), 80)))
        captured.setdefault('agree', []).append((bool(res), err))
        return res
    lf.path_agrees = spy_agree
    follow = motion = None
    rows = []
    mismatch = compared = 0
    first_mismatch = None
    with (RUNS[run] / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
        for line in f:
            r = json.loads(line)
            kind, d = r['kind'], r['data']
            if kind == 'header':
                motion = MotionConfig(**d['motion_config'])
                follow = lf.LocalFollow(lf.FollowConfig(**d['follow_config']))
            elif kind == 'reset':
                follow.reset()
            elif kind == 'call':
                captured.clear()
                recorded = d['output']
                mutate(d, experiment, t0, ts, vs)
                out_v, out_s, det = replay_call(follow, d['input'], motion)
                t = d['input']['reference_s']
                if t < t0 - 2. and experiment != 'baseline' and t < 0:
                    pass
                if experiment == 'baseline':
                    compared += 1
                    if abs(out_v - recorded[0]) > 1e-9 or abs(out_s - recorded[1]) > 1e-9:
                        mismatch += 1
                        first_mismatch = first_mismatch or (t, recorded[:2], [out_v, out_s])
                if t >= t0:
                    agree = captured.get('agree', [])
                    rows.append(dict(t=t, meas=d['input']['speed_mps'], rec_v=recorded[0], rec_s=recorded[1],
                                     v=out_v, s=out_s, reason=det.get('reason'), sel=det.get('boundary_selection'),
                                     fresh_acc=det.get('fresh_boundary_accepted'),
                                     cand_ahead=det.get('boundary_candidate_ahead_m'),
                                     remain=det.get('observed_path_ahead_m'), clear=det.get('swept_clearance_m'),
                                     tp=det.get('turn_preview'), corridor=captured.get('corridor'), agree=agree,
                                     csel=det.get('corridor_selected')))
    result = dict(run=run, experiment=experiment, t0=t0, compared=compared, mismatch=mismatch,
                  first_mismatch=first_mismatch, rows=rows,
                  scope='저장 입력 개방 루프 재생. 정답 대입은 진단 전용이며 제어 입력 후보가 아니다.')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(result, f, ensure_ascii=False)
    print(experiment, 'rows', len(rows), 'compared', compared, 'mismatch', mismatch, first_mismatch)


if __name__ == '__main__':
    main()
