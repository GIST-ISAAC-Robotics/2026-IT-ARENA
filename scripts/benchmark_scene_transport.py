#!/usr/bin/env python3
"""과거 장면 이송의 동일 계산·생성자 호출 비교. 운영 함수는 바꾸지 않는다."""
import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import platform
import time
import zipfile

import numpy as np

from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.observation_support import transport_scene
from arena_autonomy.scene_motion import motion_scene, relative_poses

ROOT = Path(__file__).resolve().parents[1]


def reference_replace_transport(scene, history, now):
    """209ecff7 기준의 원래 replace 방식. 운영 적용 뒤에도 독립 대조를 유지한다."""
    dx, dy, yaw = relative_poses(history, [scene.t_ref_s], now)[0]
    c, s = math.cos(yaw), math.sin(yaw)

    def vector(p):
        return c*p[0]-s*p[1], s*p[0]+c*p[1]

    def point(p):
        a, b = vector(p)
        return a+dx, b+dy

    rays = tuple(replace(r, ray_origin_xy_m=point(r.ray_origin_xy_m), direction_xy=vector(r.direction_xy),
                         hit_xy_m=None if r.hit_xy_m is None else point(r.hit_xy_m)) for r in scene.rays)
    return replace(scene, t_ref_s=now, rays=rays)


def direct_constructor_transport(scene, history, now):
    """replace의 필드 조회만 생략한다. 원래 생성자/유효성 검사와 계산 순서를 유지한다."""
    dx, dy, yaw = relative_poses(history, [scene.t_ref_s], now)[0]
    c, s = math.cos(yaw), math.sin(yaw)

    def vector(p):
        return c*p[0]-s*p[1], s*p[0]+c*p[1]

    def point(p):
        a, b = vector(p)
        return a+dx, b+dy

    rays = tuple(type(ray)(point(ray.ray_origin_xy_m),
                          None if ray.hit_xy_m is None else point(ray.hit_xy_m),
                          ray.acquired_at_s, ray.status, ray.source_frame_id,
                          ray.source_epoch, ray.source_index, vector(ray.direction_xy))
                 for ray in scene.rays)
    return replace(scene, t_ref_s=now, rays=rays)


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024*1024), b''):
            h.update(data)
    return h.hexdigest()


def collect(capture):
    targets = set(range(3151, 3157))
    found, mismatches, count = [], [], 0
    with capture.open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                follow = LocalFollow(FollowConfig(**event['data']['follow_config']))
            elif event['kind'] == 'reset':
                follow.reset()
            elif event['kind'] == 'call':
                data, scan, history = restore_call(event['data']['input'], motion)
                current = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                       epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                output = follow.command(current, history, data['speed_mps'], data['side'],
                                        data['previous_steering_rad'], data['dt_s'])
                count += 1
                if not equal_output(freeze(output), event['data']['output']):
                    mismatches.append(event['seq'])
                if event['seq'] in targets:
                    old = [s for s in follow.observation_cache.scenes
                           if s.source_first_s != current.source_first_s]
                    for source in old:
                        found.append((event['seq'], source, history, current.t_ref_s))
                    targets.remove(event['seq'])
                if not targets:
                    break
    if targets or mismatches or not found:
        raise ValueError(f'fixture mismatch: missing={sorted(targets)}, mismatches={mismatches}, pairs={len(found)}')
    return found, count


def distribution(values):
    return dict(count=len(values), p50_ms=float(np.percentile(values, 50)),
                p95_ms=float(np.percentile(values, 95)), max_ms=max(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=40)
    args = parser.parse_args()
    if not 4 <= args.cycles <= 100:
        parser.error('cycles must be in 4..100')
    out = args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts') or out == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    out.mkdir(parents=True, exist_ok=False)
    paths = sorted([*ROOT.glob('src/arena_autonomy/arena_autonomy/*.py'), Path(__file__).resolve()])
    before = {p.relative_to(ROOT).as_posix(): digest(p) for p in paths}
    input_hash = digest(args.capture)
    with zipfile.ZipFile(out/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in before:
            archive.write(ROOT/name, name)
    pairs, count = collect(args.capture)
    rows, mismatches = [], []
    for seq, scene, history, now in pairs:
        reference = reference_replace_transport(scene, history, now)
        candidate = direct_constructor_transport(scene, history, now)
        if reference != candidate:
            mismatches.append(seq)
        for cycle in range(args.cycles):
            functions = [('reference', reference_replace_transport), ('candidate', direct_constructor_transport)]
            if cycle % 2:
                functions.reverse()
            for label, function in functions:
                begin = time.perf_counter_ns()
                result = function(scene, history, now)
                duration = (time.perf_counter_ns()-begin)/1e6
                # 전체 필드 비교는 측정 밖이다. 시각·원점·마스크까지 완전히 같아야 한다.
                if result != reference:
                    mismatches.append(seq)
                rows.append(dict(seq=seq, cycle=cycle, version=label, elapsed_ms=duration))
    report = dict(scope='same_input_AB_BA_transport_only_not_control_deadline',
                  input_sha256=input_hash, input_unchanged=digest(args.capture) == input_hash,
                  source_sha256=before, source_unchanged=all(digest(ROOT/n) == h for n, h in before.items()),
                  replayed=count, fixture_pairs=len(pairs), exact_mismatches=mismatches,
                  warmup_calls_per_pair_per_version=1, cycles=args.cycles, python=platform.python_version(),
                  summary={label: distribution([r['elapsed_ms'] for r in rows if r['version'] == label])
                           for label in ('reference', 'candidate')}, samples=rows,
                  runtime_modified=False, unknown_filled=False)
    (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in ('samples', 'source_sha256')}))
    return 0 if not mismatches and report['source_unchanged'] and report['input_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
