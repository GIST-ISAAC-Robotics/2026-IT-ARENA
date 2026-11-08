"""사후 보조 점검(판정 비변경): 첫 검출 시점이 등록 기준 탓인지 실제 가시성 탓인지 분리.

A단계(센서 전용): 등록 관측기의 평행 각 20°→45°, 횡거리 1.2→2.0 m만 완화해 전 랩 재관측.
  반환점(차체 좌표)은 메모리에만 보관한다.
B단계(평가 전용): 완화 결과의 실제 간극 대응/오검출, 그리고 간극 안쪽(P22 통로·섬 북면)에
  떨어진 실제 반환점 수를 세어 '어떤 관측기라도 쓸 수 있는 원 근거'의 시작 시점을 본다.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import observe as ob  # noqa: E402
from observe_run import RUNS, ROOT, digest  # noqa: E402
from arena_autonomy.follow_trace import restore_call  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable  # noqa: E402
import evaluate as ev  # noqa: E402

RELAXED = dict(PARALLEL_MAX_DEG=45., LATERAL_RANGE_M=(.20, 2.00))
GAP_REGION = dict(x=(3.78, 5.62), y=(4.17, 4.83))  # 입구 간극 너머(통로·섬 북면). 평가 전용


def phase_a(name):
    for k, v in RELAXED.items():
        setattr(ob, k, v)
    source = ROOT / RUNS[name] / 'follow_decisions.jsonl'
    rows, last = [], None
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            e = json.loads(line)
            if e['kind'] == 'header':
                mc = MotionConfig(**e['data']['motion_config'])
            if e['kind'] != 'call':
                continue
            d, raw, h = restore_call(e['data']['input'], mc)
            stamp = (raw.header.stamp.sec, raw.header.stamp.nanosec)
            if stamp == last:
                continue
            last = stamp
            try:
                scene = motion_scene(raw, h, d['reference_s'], timing_verified=d['timing_verified'],
                                     epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
            except (ValueError, MotionUnavailable):
                continue
            pts = np.array([r.hit_xy_m for r in scene.rays if r.status == 'return'], np.float32)
            rows.append(dict(t=d['reference_s'], notches=ob.observe(scene)['right']['notches'], pts=pts))
    return rows


def main():
    out = dict(scope='post-hoc sensitivity; does not change the registered verdict', relaxed=RELAXED,
               gap_region_world=GAP_REGION, runs={})
    figure_rows = {}
    for name in RUNS:
        rows = phase_a(name)  # 센서 전용 단계 완료 후에만 아래 평가 입력을 읽는다
        D = ROOT / RUNS[name]
        poses = ev.Poses(ev.load_jsonl(D / 'ego_poses.jsonl'))
        trace = json.loads((D / 'trace.json').read_bytes())
        tkey = 'time' if 'time' in trace[0] else 't'
        tt = np.array([v[tkey] for v in trace])
        relaxed_entry, relaxed_false, raw_rows = [], [], []
        actors = {}
        if (D / 'actor_poses.jsonl').exists():
            actors['actor'] = ev.Poses(ev.load_jsonl(D / 'actor_poses.jsonl'))
        if (D / 'additional_actors.json').exists():
            for k, v in json.loads((D / 'additional_actors.json').read_bytes()).get('poses', {}).items():
                actors[k] = ev.Poses(v)

        def sightline_actor(t, pose, target=(5.62, 4.50)):
            # 차량 중심→간극 중앙 시선과 상대 차량 중심 거리(<=0.12 m면 가림 후보). 평가 전용
            hits = []
            for k, ap in actors.items():
                if not ap.t[0] <= t <= ap.t[-1]:
                    continue
                q = np.array(ap.at(t)[:2])
                o, e = np.array(pose[:2]), np.array(target)
                u = np.clip((q - o) @ (e - o) / ((e - o) @ (e - o)), 0, 1)
                d = float(np.hypot(*(o + u * (e - o) - q)))
                if d <= .12 and 0 < u < 1:
                    hits.append([k, round(d, 3)])
            return hits
        for r in rows:
            pose = poses.at(r['t'])
            s = trace[int(np.argmin(abs(tt - r['t'])))]['s']
            e1f = ev.to_body(pose, ev.ENTRY['E1'])[0]
            for c in r['notches']:
                w1, w2 = ev.to_world(pose, c['e1']), ev.to_world(pose, c['e2'])
                if ev.math.dist(w1, ev.ENTRY['E1']) <= ev.MATCH_M and ev.math.dist(w2, ev.ENTRY['E2']) <= ev.MATCH_M:
                    relaxed_entry.append(dict(t=r['t'], s=s, E1_forward_m=e1f))
                elif not (ev.math.dist(w1, ev.EXIT['E1']) <= ev.MATCH_M and ev.math.dist(w2, ev.EXIT['E2']) <= ev.MATCH_M):
                    relaxed_false.append(dict(t=r['t'], s=s, e1_world=w1, e2_world=w2, gap_m=c['gap_m']))
            if 16.0 <= s <= 18.7:
                c, sn = ev.math.cos(pose[2]), ev.math.sin(pose[2])
                w = np.column_stack((pose[0] + c*r['pts'][:, 0] - sn*r['pts'][:, 1],
                                     pose[1] + sn*r['pts'][:, 0] + c*r['pts'][:, 1]))
                inside = ((w[:, 0] >= GAP_REGION['x'][0]) & (w[:, 0] <= GAP_REGION['x'][1])
                          & (w[:, 1] >= GAP_REGION['y'][0]) & (w[:, 1] <= GAP_REGION['y'][1]))
                raw_rows.append(dict(t=r['t'], s=s, E1_forward_m=e1f, speed_mps=poses.speed(r['t']),
                                     gap_returns=int(inside.sum()), actor_on_sightline=sightline_actor(r['t'], pose)))
        first_raw = next((x for x in raw_rows if x['gap_returns'] >= ob.DEPTH_MIN_RAYS and x['E1_forward_m'] > 0), None)
        uniq = lambda xs: len({round(x['t'], 4) for x in xs})
        out['runs'][name] = dict(
            relaxed_entry_scans_before_E1=uniq([x for x in relaxed_entry if x['E1_forward_m'] > 0]),
            relaxed_entry_first=min(relaxed_entry, key=lambda x: x['t'], default=None),
            relaxed_false_scans=uniq(relaxed_false), relaxed_false_examples=relaxed_false[:5],
            raw_gap_returns_first_ge3_before_E1=first_raw, raw_gap_returns_by_scan=raw_rows,
            sensor_sha256=digest(D / 'follow_decisions.jsonl'))
        # 대표 그림용: 등록 기준 첫 입구 간극 스캔
        evaluation = json.loads((HERE / 'evaluation.json').read_bytes())['runs'][name]
        first = evaluation['entry_notch_first']
        if first:
            r = min(rows, key=lambda x: abs(x['t'] - first['t']))
            figure_rows[name] = (r, first, poses.at(r['t']))
    (HERE / 'sensitivity.json').write_bytes((json.dumps(out, ensure_ascii=False, indent=1) + '\n').encode('utf-8'))
    plot(figure_rows)
    print(json.dumps({n: {k: v for k, v in r.items() if k != 'raw_gap_returns_by_scan' and k != 'relaxed_false_examples'}
                      for n, r in out['runs'].items()}, ensure_ascii=False, indent=1))


def plot(figure_rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(figure_rows), figsize=(5.2*len(figure_rows), 5.2), squeeze=False)
    for ax, (name, (r, first, pose)) in zip(axes[0], figure_rows.items()):
        p = r['pts']
        keep = (p[:, 0] > -0.6) & (p[:, 0] < 3.0) & (abs(p[:, 1]) < 2.2)
        ax.scatter(-p[keep, 1], p[keep, 0], s=4, c='#4a5568', label='C1 returns (motion-compensated)')
        # 등록 기준 관측(센서) — 같은 시각 등록 관측기로 다시 표시
        reg = json.loads((HERE / f'observations_{name}.json').read_bytes())['rows']
        row = min(reg, key=lambda x: abs(x['t'] - r['t']))
        for c in row['right']['notches']:
            (x1, y1), (x2, y2) = c['e1'], c['e2']
            ax.plot([-y1, -y2], [x1, x2], 'o--', c='#d97706', ms=6, label=f"observed gap E1->E2 {c['gap_m']:.3f} m")
        e1, e2 = ev.to_body(pose, ev.ENTRY['E1']), ev.to_body(pose, ev.ENTRY['E2'])
        ax.plot([-e1[1], -e2[1]], [e1[0], e2[0]], 'x', c='#2563eb', ms=9, label='true E1/E2 (evaluation only)')
        ax.plot([0], [0], '^', c='k', ms=9)
        ax.set_title(f"{name} t={first['t']:.2f}s  E1 ahead {first['E1_forward_m']:.2f} m\n"
                     f"speed {first['speed_mps']:.2f} m/s -> {first['E1_time_left_s']:.2f} s to E1", fontsize=9)
        ax.set_xlabel('right + [m] (body frame)')
        ax.set_ylabel('forward [m]')
        ax.set_aspect('equal')
        ax.grid(alpha=.3)
        ax.legend(fontsize=7, loc='upper left')
    fig.tight_layout()
    fig.savefig(HERE / 'first_entry_notch.png', dpi=110)


if __name__ == '__main__':
    main()
