"""보존한 수정 전 IMU 어댑터와 현재 어댑터의 동일 입력 비교 및 순수 게이트 비용."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import runpy
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.lidar_motion_ros import MotionInput
from arena_autonomy.lidar_observation import audit_front_observation

support = runpy.run_path(str(ROOT/'tests/test_lidar_motion_ros.py'))
NodeDouble, imu, wheel = (support[k] for k in ('NodeDouble', 'imu', 'wheel'))
spec = importlib.util.spec_from_file_location('imu_before', HERE/'before_sources/lidar_motion_ros.py')
before = importlib.util.module_from_spec(spec)
spec.loader.exec_module(before)
results = []
for case in ('single_spike', 'two_spikes', 'normal_ramp', 'slow_false_ramp', 'fresh_stuck_zero', 'small_noise'):
    row = {'case': case}
    for label, adapter_type in [('before', before.MotionInput), ('after', MotionInput)]:
        node = NodeDouble()
        adapter = adapter_type(node)
        rng = np.random.default_rng(20261004)
        for i in range(41):
            node.t = i * .005
            if case == 'single_spike':
                rate = 4. if i == 10 else 0.
            elif case == 'two_spikes':
                rate = 4. if i in (10, 11) else 0.
            elif case in ('normal_ramp', 'slow_false_ramp'):
                rate = 20. * node.t
            elif case == 'small_noise':
                rate = float(rng.normal(0., .001))
            else:
                rate = 0.
            adapter.on_wheels(wheel(node.t))
            adapter.on_imu(imu(node.t, z=rate))
        poses, _ = adapter.history.poses([0., .2], .2)
        row[label] = {'accepted': adapter.counts['gyro'], 'rejected': adapter.counts['rejected'],
                      'integrated_yaw_rad': float(poses[-1, 2]-poses[0, 2]),
                      'rejection_reasons': getattr(adapter, 'gyro_rejections', {})}
    row['scope'] = ('same sensor values cannot distinguish real from false ramp' if 'ramp' in case else
                    'fresh zero can mean straight driving or a stuck sensor' if case == 'fresh_stuck_zero' else
                    'synthetic 200 Hz input; not measured hardware error distribution')
    results.append(row)

ranges = np.full(500, 2.)
samples = []
for i in range(2200):
    began = time.perf_counter()
    result = audit_front_observation(ranges, -math.pi, 2*math.pi/500, .05, 12.)
    duration = (time.perf_counter()-began)*1000
    assert result.reason == 'ok'
    if i >= 200:
        samples.append(duration)
report = {'scope': 'synthetic adapter comparison and host-only helper microbenchmark; not Gazebo or Jetson',
          'imu': results, 'front_audit_ms': {str(q): float(np.percentile(samples, q)) for q in (50, 95, 99, 100)},
          'benchmark_iterations': len(samples),
          'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in (
              Path(__file__), HERE/'before_sources/lidar_motion_ros.py',
              ROOT/'src/arena_autonomy/arena_autonomy/lidar_motion_ros.py',
              ROOT/'src/arena_autonomy/arena_autonomy/lidar_observation.py')}}
with (HERE/'input_probe.json').open('x', encoding='utf-8') as f:
    json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
