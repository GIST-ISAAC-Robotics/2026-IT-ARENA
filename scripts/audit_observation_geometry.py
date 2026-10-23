#!/usr/bin/env python3
"""고정 반례(seq 3156/3181/3501)의 관측 실패를 질의·광선·몸체 역할로 분해한다.

저장된 10/17 판단 입력만 재생한다. 정답 자세/지도·ROS·운영 파일을 사용하거나
바꾸지 않는다. 조향 격자는 실제 직전각(기본)과 가정 직전각(=명령각)을 구분한다.
결과는 기존 2 cm 표본 질의의 분해이며 연속 외곽·동적 침입·실제 진행 증명이 아니다.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.observation_support import transport_scene
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest
import observation_witness as ow

ROOT = Path(__file__).resolve().parents[1]
GRID = (-.18, -.12, -.06, -.03, -.01, -.001, -1e-4, 0., 1e-4, .001, .01, .03, .06, .12, .18)


def events(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def collect(capture, targets):
    """판단 재생으로 대상 seq의 현재/최근 장면을 제어기 캐시와 같은 순서로 얻는다."""
    found, checked, mismatches = {}, 0, []
    for event in events(capture):
        kind = event['kind']
        if kind == 'header':
            cfg = FollowConfig(**event['data']['follow_config'])
            motion = MotionConfig(**event['data']['motion_config'])
            follow = LocalFollow(cfg)
        elif kind == 'reset':
            follow.reset()
        elif kind == 'call':
            data, scan, history = restore_call(event['data']['input'], motion)
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                 epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            output = follow.command(scene, history, data['speed_mps'], data['side'],
                                    data['previous_steering_rad'], data['dt_s'])
            checked += 1
            if not equal_output(freeze(output), event['data']['output']):
                mismatches.append(event['seq'])
            if event['seq'] in targets:
                sources = [scene]
                for old in reversed(follow.observation_cache.scenes):
                    if old.source_first_s == scene.source_first_s:
                        continue
                    try:
                        sources.append(transport_scene(old, history, scene.t_ref_s))
                    except MotionUnavailable:
                        continue
                found[event['seq']] = dict(data=data, sources=sources, output=event['data']['output'])
                if len(found) == len(targets):
                    break
    return cfg, motion, found, checked, mismatches


def summarize(rows):
    return {' | '.join(k): n for k, n in Counter((r['role'], r['cause'], r['distance_class']) for r in rows).most_common()}


def scene_summary(scene):
    rows = []
    for ray in scene.rays:
        if ray.status != 'unknown':
            continue
        neighbours = []
        for k in (ray.source_index-1, ray.source_index+1):
            if not 0 <= k < len(scene.rays):
                continue  # 회전 이음부를 시간적으로 인접한 광선으로 가정하지 않는다.
            other = scene.rays[k]
            neighbours.append(dict(index=k, status=other.status, depth_m=None if other.hit_xy_m is None
                                   else math.dist(other.ray_origin_xy_m, other.hit_xy_m)))
        rows.append(dict(**ow.ray_row(ray, scene), neighbours=neighbours))
    masked = [math.degrees(math.atan2(r.direction_xy[1], r.direction_xy[0])) % 360 for r in scene.rays
              if r.status == 'masked']
    return dict(t_ref_s=scene.t_ref_s, source_first_s=scene.source_first_s,
                age_s=scene.t_ref_s-scene.source_first_s, unknown=rows, masked_count=len(masked),
                masked_bearing_deg_range=[min(masked), max(masked)] if masked else None)


def analyse(cfg, motion, case, seq):
    data, sources = case['data'], case['sources']
    pose = (motion.lidar_x_m, motion.lidar_y_m, motion.lidar_yaw_rad)
    speed, previous = data['speed_mps'], data['previous_steering_rad']
    report = dict(seq=seq, t_ref_s=sources[0].t_ref_s, measured_motor_speed_mps=speed,
                  recorded_previous_steering_rad=previous, preferred_side=data['side'],
                  recorded_command=case['output'][:2], recorded_reason=case['output'][2]['reason'],
                  current=scene_summary(sources[0]),
                  sources=[dict(source_first_s=s.source_first_s,
                                unknown=[r.source_index for r in s.rays if r.status == 'unknown'])
                           for s in sources])
    witnesses = {}
    recorded = (case['output'][2].get('recent_observation') or {})
    if recorded.get('reason') in ('sweep_unobserved', 'sampled_recent_observation_support'):
        # 원래 호출과 같은 명령각을 재현한다. 기록된 지지 수와 대조한다.
        proposed = case['output'][2].get('path_points') is not None
        steer = None
        if seq == 3156:
            steer = 0.18252839482686553
        if steer is not None:
            result, rows, consistent = ow.failure_witnesses(sources, speed, .2, steer, previous, cfg, pose)
            report['recorded_call'] = dict(steer_rad=steer, previous_rad=previous, supported=result['supported'],
                reason=result['reason'], queries=result.get('queries'), supported_queries=result.get('supported_queries'),
                recorded_queries=recorded.get('queries'), recorded_supported_queries=recorded.get('supported_queries'),
                matches_recorded=(result.get('queries'), result.get('supported_queries')) ==
                                 (recorded.get('queries'), recorded.get('supported_queries')),
                consistent=consistent, distance_parts=result.get('distance_parts'),
                failures=len(rows), summary=summarize(rows), path_points_present=proposed)
            witnesses['recorded'] = rows
    grid = []
    for mode in ('actual_previous', 'previous_equals_command'):
        for steer in GRID:
            prev = previous if mode == 'actual_previous' else steer
            result, rows, consistent = ow.failure_witnesses(sources, speed, .2, steer, prev, cfg, pose)
            entry = dict(mode=mode, steer_rad=steer, previous_rad=prev, supported=result['supported'],
                         reason=result['reason'], queries=result.get('queries'),
                         supported_queries=result.get('supported_queries'),
                         required_distance_m=result.get('required_distance_m'), consistent=consistent,
                         failures=len(rows), summary=summarize(rows),
                         minimum_entry_travel_m=min((r['entry_travel_m'] for r in rows
                                                     if r['entry_travel_m'] is not None), default=None),
                         outside_independent_sweep=sum(r['distance_class'] == 'outside_independent_sweep'
                                                       for r in rows))
            grid.append(entry)
            witnesses[f'{mode}:{steer:+.4f}'] = rows
    report['grid'] = grid
    report['supported_steering'] = {mode: [g['steer_rad'] for g in grid if g['mode'] == mode and g['supported']]
                                    for mode in ('actual_previous', 'previous_equals_command')}
    return report, witnesses


def figure(cfg, case, rows, steer, previous, path, title):
    """평면 기하 SVG. 축 비율 1:1, 원시 반환·쐐기·몸체·실패 질의를 같은 좌표에 그린다."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.patches import Polygon as Patch
    for font in (Path('/mnt/c/Windows/Fonts/malgun.ttf'), Path('C:/Windows/Fonts/malgun.ttf')):
        if font.is_file():
            font_manager.fontManager.addfont(str(font))
            plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
            break
    plt.rcParams['axes.unicode_minus'] = False
    current = case['sources'][0]
    parts = ow.distance_parts(case['data']['speed_mps'], .2, current.t_ref_s-current.source_first_s, cfg)
    colors = {'unknown_wedge': '#2a78d6', 'unknown_bracket': '#7548b0',
              'lateral_reach': '#eb6834', 'masked_wedge': '#1baf7a', 'masked_bracket': '#167466',
              'bracket_unsupported': '#52514e'}
    markers = {'unknown_wedge': 'o', 'unknown_bracket': 'D', 'lateral_reach': '^',
               'masked_wedge': 's', 'masked_bracket': 'd', 'bracket_unsupported': 'x'}
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.2))
    for ax, (xlim, ylim) in zip(axes, (((-.3, .55), (-.35, .4)), ((-.12, .14), (.05, .13)))):
        ax.set_facecolor('#fcfcfb')
        for ray in current.rays:
            o = np.asarray(ray.ray_origin_xy_m)
            d = np.asarray(ray.direction_xy)
            if ray.status == 'return':
                h = np.asarray(ray.hit_xy_m)
                if abs(h[0]) < 1. and abs(h[1]) < 1.:
                    ax.plot(*h, '.', color='#8a8984', ms=2)
                continue
            for half in (-ow.HALF_CELL, ow.HALF_CELL):
                c, s = math.cos(half), math.sin(half)
                e = o+1.5*np.array([c*d[0]-s*d[1], s*d[0]+c*d[1]])
                ax.plot([o[0], e[0]], [o[1], e[1]], color='#2a78d6' if ray.status == 'unknown' else '#1baf7a',
                        lw=.6, alpha=.8 if ray.status == 'unknown' else .25)
        for a in dict.fromkeys((steer, previous)):
            for travel, style in ((parts['future_motion_m'], '-'), (parts['total_m'], ':')):
                for t in np.linspace(0., travel, 6)[1:]:
                    body = ow.arc_body(a, t, cfg)
                    ax.add_patch(Patch(body, closed=True, fill=False, ec='#52514e', lw=.5, ls=style, alpha=.6))
        ax.add_patch(Patch(ow.body_corners(cfg), closed=True, fill=True, fc='#e6e5e1', ec='#0b0b0b', lw=1.))
        ax.plot(.06, 0., marker='+', color='#0b0b0b', ms=8)
        for cause in colors:
            pts = np.array([r['query_xy_m'] for r in rows if r['cause'] == cause]).reshape(-1, 2)
            if len(pts):
                ax.scatter(pts[:, 0], pts[:, 1], s=14 if cause != 'bracket_unsupported' else 18,
                           c=colors[cause], marker=markers[cause], label=f'{cause} ({len(pts)})',
                           edgecolors='#fcfcfb' if markers[cause] != 'x' else None, linewidths=.4, zorder=5)
        ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect('equal')
        ax.grid(True, color='#e6e5e1', lw=.5)
        ax.set_xlabel('x base_link [m]'); ax.set_ylabel('y base_link [m]')
    axes[0].set_title(title, fontsize=9)
    axes[1].set_title('확대: 몸체 좌측 / 무응답 쐐기', fontsize=9)
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        axes[0].legend(handles, labels, fontsize=7, loc='lower left')
    fig.text(.01, .01, '회색 실선: 미래 반응+제동 몸체, 점선: 기존 전체 요구 거리 몸체, 파랑 선: unknown 쐐기, '
             '초록 선: 후방 mask 쐐기, 회색 점: 실제 반환', fontsize=7)
    fig.tight_layout(rect=(0, .03, 1, 1))
    fig.savefig(path, format='svg')
    fig.savefig(path.with_suffix('.png'), dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seqs', default='3151,3156,3181,3501')
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    targets = {int(s) for s in args.seqs.split(',')}
    before = digest(args.capture)
    cfg, motion, found, checked, mismatches = collect(args.capture, targets)
    reports, all_witnesses = [], {}
    for seq in sorted(found):
        report, witnesses = analyse(cfg, motion, found[seq], seq)
        reports.append(report)
        all_witnesses[str(seq)] = witnesses
    figures = []
    for seq, key, steer in ((3156, 'recorded', .18252839482686553), (3501, 'actual_previous:+0.0600', .06),
                            (3501, 'actual_previous:-0.0600', -.06), (3501, 'actual_previous:+0.0010', .001)):
        if seq in found and key in all_witnesses[str(seq)]:
            name = f'seq{seq}_{key.replace(":", "_").replace("+", "p").replace("-", "m")}.svg'
            data = found[seq]['data']
            figure(cfg, found[seq], all_witnesses[str(seq)][key], steer, data['previous_steering_rad'],
                   output/name, f'seq {seq} 명령 {steer:+.4f} rad / 직전 {data["previous_steering_rad"]:+.4f} rad, '
                   f'모터 환산 {data["speed_mps"]:.3f} m/s')
            figures.append(name)
    paths = {Path(__file__).resolve(), ROOT/'scripts/observation_witness.py'}
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            path = Path(name).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py':
                paths.add(path)
    sources = {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(paths)}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    (output/'witnesses.json').write_text(json.dumps(all_witnesses, allow_nan=False)+'\n', encoding='utf-8')
    result = dict(scope='stored_input_query_decomposition_not_new_driving', replayed=checked,
                  replay_mismatches=mismatches, cases=reports, figures=figures,
                  all_consistent=all(g['consistent'] for r in reports for g in r['grid']) and
                  all(r.get('recorded_call', {}).get('consistent', True) for r in reports),
                  independent_entry_missing=sum(g['outside_independent_sweep'] for r in reports for g in r['grid']),
                  unknown_filled=False, free_area_certified=False, continuous_sweep_verified=False,
                  dynamic_cut_in_verified=False, actual_progress_verified=False,
                  source_sha256=sources, source_unchanged=all(digest(ROOT/n) == h for n, h in sources.items()),
                  input_sha256=before, input_unchanged=digest(args.capture) == before)
    (output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('cases', 'source_sha256')}))
    for r in reports:
        print(r['seq'], json.dumps(r['supported_steering']), json.dumps(r.get('recorded_call', {}).get('summary')))
    ok = (not mismatches and result['all_consistent'] and result['independent_entry_missing'] == 0
          and result['input_unchanged'] and result['source_unchanged'])
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
