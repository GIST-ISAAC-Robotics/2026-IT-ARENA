#!/usr/bin/env bash
# 새 비공개 시험 폴더에서만 한 번 실행. ROS/센서 발행 전 입력 무결성을 검사한다.
set -eo pipefail
cd /home/q/it_arena_a3_20260928
python3 - <<'PY'
import hashlib, json, pathlib, tarfile
root = pathlib.Path.cwd()
manifest = json.loads((root/'bundle_manifest.json').read_text())
archive = root/'source_inputs.tar.gz'
assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['archive_sha256']
with tarfile.open(archive) as tar:
    members = tar.getmembers()
    assert {m.name for m in members} == set(manifest['files'])
    assert all(m.isfile() and (root/m.name).resolve().is_relative_to(root) for m in members)
print('BUNDLE_ARCHIVE_SHA256_VERIFIED', len(manifest['files']))
PY
tar --keep-old-files -xzf source_inputs.tar.gz
python3 - <<'PY'
import hashlib, json, pathlib
root = pathlib.Path.cwd()
manifest = json.loads((root/'bundle_manifest.json').read_text())
for name, entry in manifest['files'].items():
    path = root/name
    assert path.stat().st_size == entry['size'], name
    assert hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256'], name
print('ALL_BUNDLE_FILES_VERIFIED')
PY
source /opt/ros/humble/setup.bash
colcon build --packages-select arena_vehicle_interface arena_autonomy --executor sequential
source install/setup.bash
python3 - <<'PY'
import cv2, numpy, platform, rclpy, rosbag2_py, json
print(json.dumps(dict(python=platform.python_version(), opencv=cv2.__version__, numpy=numpy.__version__,
                     opencv_path=cv2.__file__, opencv_threads=cv2.getNumThreads(),
                     aruco_detector=hasattr(cv2.aruco, 'ArucoDetector'))))
PY
# 기록된 과거 코너 영상에 의존하는 2개만 이번 소형 번들에서 제외한다.
python3 -m pytest -q tests/test_bag_contract.py tests/test_timing_probe.py tests/test_queued_log.py \
  tests/test_lidar_motion.py tests/test_lidar_motion_ros.py tests/test_lidar_safety.py \
  tests/test_local_path_range_noise.py -k 'not overshot_corner'
mkdir -p artifacts/jetson_a3
python3 scripts/audit_sensor_bag.py artifacts/validation/2026-09-24/record_replay/capture_v2/sensor_recording \
  --output artifacts/jetson_a3/input_audit.json
echo 'JETSON_REPLAY_PREPARATION_PASSED'
