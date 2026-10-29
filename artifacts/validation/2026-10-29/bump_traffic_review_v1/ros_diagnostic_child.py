"""독립 보호 콜백의 시간·운동 입력 나이만 관측. 운영 소스/조건 미변경."""
import json
import os
from pathlib import Path
import time
from arena_autonomy.lidar_safety import LidarSafety,main

rows=[]
original=LidarSafety.control

def measured(self):
    start=time.monotonic()
    now=self.get_clock().now().nanoseconds*1e-9
    h=self.motion.history
    row=dict(start_wall_s=start,reference_s=now,
        wheel_age_s=now-h.wheels.times[-1] if h.wheels.times else None,
        gyro_age_s=now-h.gyro.times[-1] if h.gyro.times else None,
        previous_control_gap_s=start-rows[-1]['start_wall_s'] if rows else None,
        latched_before=getattr(self.partial_policy,'latched',None))
    try:
        return original(self)
    finally:
        row.update(duration_ms=(time.monotonic()-start)*1000,
            latched_after=getattr(self.partial_policy,'latched',None))
        rows.append(row)

LidarSafety.control=measured
try:
    main()
finally:
    with Path(os.environ['ARENA_DIAGNOSTIC_CALLBACKS']).open('x',encoding='utf-8') as stream:
        json.dump(rows,stream,indent=1)
