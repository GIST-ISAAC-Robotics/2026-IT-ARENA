"""PR #3의 미해결 위험 재현. ROS 메시지/실제 투영·보호 계산, 노드 I/O는 대역.

저장소 루트에서 ROS Jazzy를 source한 뒤 python3 <이 파일>로 실행한다.
센서/모터/ROS 토픽에는 연결하지 않는다. 통과 시험이 아니라 현재 동작 계측이다.
"""
import hashlib
import json
import math
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT / "src/arena_autonomy"), str(ROOT / "src/arena_vehicle_interface")]
from arena_autonomy.lidar_motion import MotionHistory
from arena_autonomy.lidar_motion_ros import MotionInput
from arena_autonomy.lidar_safety import LidarSafety

node_double = runpy.run_path(str(ROOT / "tests/test_lidar_safety.py"))["node_double"]
results = []
for case, value in [("finite_obstacle", .15), ("nan_sector", math.nan),
                    ("positive_inf_sector", math.inf), ("all_positive_inf", math.inf)]:
    node, commands, statuses = node_double()
    node.scan.header.frame_id = "laser_frame"
    node.scan.time_increment = .1 / 500
    history = MotionHistory()
    for stamp in np.arange(9.99, 10.101, .005):
        history.add_wheels(float(stamp), 0., 0.)
        history.add_gyro(float(stamp), 0.)
    motion = SimpleNamespace(clock=lambda: 10.1, history=history, mode="both", verified=True)
    motion.project = lambda scan: MotionInput.project(motion, scan)
    node.motion = motion
    angles = node.scan.angle_min + np.arange(500) * node.scan.angle_increment
    selected = np.ones(500, dtype=bool) if case == "all_positive_inf" else np.abs(angles) < math.radians(20)
    for index in np.flatnonzero(selected):
        node.scan.ranges[int(index)] = value
    LidarSafety.control(node)
    results.append({"case": case, "modified_rays": int(selected.sum()),
                    "finite_rays": int(np.isfinite(node.scan.ranges).sum()),
                    **json.loads(statuses[-1].data)})

reproduced = (results[0]["safe_speed_mps"] == 0.
              and results[1]["reason"] == "front_unobserved"
              and results[2]["safe_speed_mps"] > 0.
              and results[3]["reason"] == "scan_invalid")
payload = {
    "scope": "synthetic ROS messages; actual MotionInput.project and LidarSafety.control; no ROS graph or vehicle",
    "driver_refs": {"sllidar_ros2": "34300099fadfc772965962dec837bf436706188f",
                    "rplidar_ros": "24cc9b6dea97e045bda1408eaa867ce730fd3fc3"},
    "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in [ROOT / "src/arena_autonomy/arena_autonomy/lidar_safety.py",
                                   ROOT / "src/arena_autonomy/arena_autonomy/lidar_motion.py",
                                   ROOT / "src/arena_autonomy/arena_autonomy/lidar_motion_ros.py"]},
    "no_return_risk_reproduced": reproduced,
    "cases": results,
}
print(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False))
