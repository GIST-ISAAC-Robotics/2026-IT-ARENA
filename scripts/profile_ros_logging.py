#!/usr/bin/env python3
"""제어기 없는 ROS 로거 단독 계측. 설치된 로거를 수정하거나 우회하지 않는다."""
import argparse
import cProfile
import json
from pathlib import Path
import pstats
import sys
import time

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import quantiles, sha256_file


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    import rclpy.logging
    import rclpy.impl.rcutils_logger as implementation
    rclpy.init()
    logger = rclpy.logging.get_logger('arena_logger_profile')
    profiler = cProfile.Profile()
    durations = []
    try:
        profiler.enable()
        for index in range(25):
            began = time.perf_counter_ns()
            logger.info('Profiling state log '+str(index))
            durations.append((time.perf_counter_ns()-began)/1e6)
        profiler.disable()
    finally:
        rclpy.shutdown()
    profiler.dump_stats(str(args.output/'logging.prof'))
    stats = pstats.Stats(profiler)
    rows = [dict(file=k[0], line=k[1], function=k[2], calls=v[1],
                 own_s=v[2], cumulative_s=v[3]) for k, v in stats.stats.items()]
    result = dict(duration_ms=quantiles(durations),
        top_cumulative=sorted(rows, key=lambda r: r['cumulative_s'], reverse=True)[:25],
        source_sha256={str(p): sha256_file(p) for p in (Path(__file__), Path(implementation.__file__))},
        scope='logger-only cProfile probe; profiling overhead included')
    (args.output/'report.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
