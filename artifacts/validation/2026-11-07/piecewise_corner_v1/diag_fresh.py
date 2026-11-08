"""저장 판단 입력 재생 진단(격리 사본 코드, 평가 전용). 제어기를 바꾸지 않는다.

사용: diag_replay.py <run 디렉터리> <출력 json> [관심 시각 s ...]
1) 기록 전체를 follow_trace.replay_events로 재생해 기록 출력과 일치하는지 센다.
2) 관심 시각 근처 판단에서 local_follow가 부른 local_path(왼/오른쪽)의 입력 점·결과 사유를 기록하고
   스캔 점(차량 좌표)·경로 후보를 그림으로 남긴다.
"""
import json
import sys
from pathlib import Path

import numpy as np

import arena_autonomy.local_follow as lf
import arena_autonomy.corner_distance as cdm
from arena_autonomy.follow_trace import replay_events, replay_call
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig


def main(run, output, *times):
    run, output = Path(run), Path(output)
    times = [float(t) for t in times]
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        events = [json.loads(line) for line in stream if line.strip()]
    result = dict(scope='single stateful diagnostic replay; no full output-equivalence rerun')
    header = events[0]['data']
    follow = LocalFollow(FollowConfig(**header['follow_config']))
    motion = MotionConfig(**header['motion_config'])
    captured = []
    original_path, original_cde = lf.local_path, lf.corner_distance_evidence

    def spy_path(points, side, offset, **kw):
        path, info = original_path(points, side, offset, **kw)
        captured.append(dict(kind='local_path', side=side, n_points=int(len(points)),
                             reason=info.get('reason'), wall_model=info.get('wall_model'), ahead=info.get('boundary_observed_ahead_m'),
                             wall_distance=info.get('wall_distance_m'), horizon=kw.get('horizon'),
                             path=None if path is None else np.asarray(path).round(3).tolist(),
                             points=np.asarray(points).round(3).tolist()))
        return path, info

    def spy_cde(points, reference, side, config, **kw):
        out = original_cde(points, reference, side, config, **kw)
        captured.append(dict(kind='corner_distance', side=side,
                             **{k: v for k, v in out.items() if k != 'source_indices'}))
        return out
    lf.local_path, lf.corner_distance_evidence = spy_path, spy_cde
    picks = []; selected = set()
    try:
        for e in events[1:]:
            if e['kind'] == 'reset':
                follow.reset(); continue
            if e['kind'] != 'call':
                continue
            t = e['data']['input']['reference_s']
            near = any(abs(t-x) < .011 for x in times)
            captured.clear()
            out = replay_call(follow, e['data']['input'], motion)
            if captured and any(x not in selected and x <= t < x+.15 for x in times):
                selected.update(x for x in times if x <= t < x+.15)
                keys = ('boundary_selection', 'boundary_candidate_reason', 'fresh_boundary_accepted', 'path_wall',
                        'corner_distance_agrees_memory', 'bilateral_reference', 'observed_path_ahead_m', 'behavior',
                        'turn_preview', 'swept_clearance_m', 'constant_arc_target_mps', 'target_speed_before_hold_mps')
                picks.append(dict(t=round(t, 3), speed=out[0], steer=out[1], reason=out[2].get('reason'),
                                  detail=out[2],
                                  side_in=e['data']['input']['side'], calls=list(captured)))
    finally:
        lf.local_path, lf.corner_distance_evidence = original_path, original_cde
    output.with_suffix('.clouds.json').write_text(json.dumps(picks, default=str), encoding='utf8')
    result['picks'] = [dict(t=p['t'], speed=p['speed'], reason=p['reason'], detail=p['detail'],
                            calls=[{k: v for k, v in c.items() if k not in ('points', 'path')} for c in p['calls']])
                       for p in picks]
    output.write_text(json.dumps(result, ensure_ascii=False, indent=1, default=str), encoding='utf-8', newline='\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for p in picks:
        pts = next((c['points'] for c in p['calls'] if c['kind'] == 'local_path' and c['n_points'] > 100), None)
        if pts is None:
            continue
        fig, ax = plt.subplots(figsize=(6, 6), dpi=110)
        a = np.array(pts)
        ax.plot(a[:, 0], a[:, 1], '.', ms=2, color='0.3', label='scan returns (vehicle frame)')
        for c in p['calls']:
            if c['kind'] == 'local_path' and c['path']:
                q = np.array(c['path'])
                ax.plot(q[:, 0], q[:, 1], '-', lw=1.5, label=f"{c['side']} path ahead={c['ahead']}")
        ax.plot(0, 0, 'r>', ms=8)
        ax.set_aspect('equal'); ax.set_xlim(-1, 2.5); ax.set_ylim(-1.5, 2.0); ax.grid(alpha=.3)
        ax.set_title(f"t={p['t']} v={p['speed']:.2f} {p['reason']}", fontsize=8)
        ax.legend(fontsize=6, loc='lower right')
        fig.tight_layout(); fig.savefig(output.with_name(output.stem+f"_t{p['t']:.2f}.png")); plt.close(fig)
    print(json.dumps(dict(scope=result['scope'], picks=[(p['t'], round(p['speed'], 3), p['reason'],
          [(c['kind'], c['side'], c.get('reason'), c.get('ahead'), c.get('wall_distance')) for c in p['calls']])
          for p in picks]), ensure_ascii=False, default=str))


if __name__ == '__main__':
    main(*sys.argv[1:])
