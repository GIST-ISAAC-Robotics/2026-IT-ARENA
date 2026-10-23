#!/usr/bin/env python3
"""ROS 직선 결측 실패의 기하 분리. 합성 입력이며 폐루프/실차 판정이 아니다."""
import argparse
from collections import Counter
from dataclasses import replace
import json
import math
from pathlib import Path
import zipfile

import numpy as np

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_support import sweep_queries, sweep_support
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_visibility import bracket_depth
from audit_corridor_runtime import digest
from test_local_follow import history, scan
from validate_pipeline_faults_ros import corridor_range

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    names = ['scripts/probe_recent_straight_precision.py',
             'src/arena_autonomy/arena_autonomy/observation_support.py',
             'src/arena_autonomy/arena_autonomy/observation_lateral.py']
    hashes = {name: digest(ROOT/name) for name in names}
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for name in names: z.write(ROOT/name, name)
    cfg = FollowConfig(wheelbase_m=.135)
    rows = []
    # 현재 ROS LaserScan의 float32 각도/거리 양자화를 포함한다.
    for quantized in (False, True):
        raw = scan(.5)
        if quantized:
            raw.angle_min = float(np.float32(raw.angle_min))
            raw.angle_increment = float(np.float32(raw.angle_increment))
        raw.ranges = [corridor_range(raw.angle_min+i*raw.angle_increment, .06) for i in range(500)]
        if quantized: raw.ranges = np.asarray(raw.ranges, dtype=np.float32).tolist()
        raw.ranges[375] = math.inf
        scene = motion_scene(raw, history(), .635, timing_verified=True, rear_half_angle_deg=30.)
        padding = np.array([.055+.5*(scene.t_ref_s-r.acquired_at_s) for r in scene.rays])
        for angle in (0., -9.6137665e-9, 9.6137665e-9, -1e-7, 1e-7, -.001, .001, -.175, .175):
            result = sweep_support([scene], 0., .2, angle, angle, cfg, protected_sensor_pose=(.06,0.,0.))
            q, thin = sweep_queries(scene, angle, .22+.2*(.135+.1)+.2**2/4, cfg)
            details = bracket_depth(scene, q, padding)
            failed = [d for d in details if not d['supported']]
            rows.append(dict(quantized=quantized, steering_rad=angle, result=result,
                depth_failure_reasons=dict(Counter(d['reason'] for d in failed)),
                failed_queries=failed[:30], thin=thin))
    report = dict(scope='synthetic_straight_precision_not_driving', rows=rows, source_sha256=hashes,
                  source_unchanged=all(digest(ROOT/name)==h for name,h in hashes.items()))
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps([dict(quantized=r['quantized'], angle=r['steering_rad'],
        supported=r['result']['supported'], queries=r['result'].get('queries'),
        supported_queries=r['result'].get('supported_queries'), failures=r['depth_failure_reasons']) for r in rows]))


if __name__ == '__main__': main()
