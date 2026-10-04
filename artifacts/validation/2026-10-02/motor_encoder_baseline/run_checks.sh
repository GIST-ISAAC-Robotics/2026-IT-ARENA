#!/usr/bin/env bash
# 모터축 엔코더 기준선 변경의 정적·단위·모델 생성 검사. Gazebo/ROS launch/노드 프로세스/bag 재생 없음.
# 사용: bash run_checks.sh <log-suffix>. 마지막에 단계별 종료 코드와 전체 verdict를 남긴다.
# v1~v4 로그는 이 판정 집계 보완 이전 실행본이며 덮어쓰지 않는다.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
OUT="$REPO/artifacts/validation/2026-10-02/motor_encoder_baseline"
SUFFIX="${1:-v1}"
SUMMARY="$OUT/verdict_$SUFFIX.txt"
cd "$REPO" || exit 2
set +u
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
set -u
# 설치본보다 작업 트리 소스를 우선한다(새 drive_feedback 모듈 포함).
export PYTHONPATH="$REPO/src/arena_vehicle_interface:$REPO/src/arena_autonomy:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
: > "$SUMMARY"
FAILED=0
step() {  # step <name> <exit>
  echo "$1_exit=$2" | tee -a "$SUMMARY"
  [ "$2" -eq 0 ] || FAILED=1
}

TARGETED=(
  tests/test_drive_feedback.py tests/test_lidar_motion_ros.py tests/test_lidar_motion.py
  tests/test_actuation_contract.py tests/test_bag_contract.py tests/test_replay_continuity.py
  tests/test_motion_impairment.py tests/test_drivetrain_model.py tests/test_track_variants.py
  tests/test_basic_autonomy.py tests/test_rear_identification_marker.py tests/test_tof_ring.py
  tests/test_d435i_model.py tests/test_official_track.py tests/test_lidar_safety.py tests/test_tof_safety.py
  tests/test_actuation_ros_nodes.py tests/test_applied_guard.py tests/test_bag_reader_compat.py
  src/arena_vehicle_interface/test
)
python3 -m pytest -q -p no:cacheprovider "${TARGETED[@]}" > "$OUT/pytest_targeted_$SUFFIX.log" 2>&1
step targeted $?
tail -3 "$OUT/pytest_targeted_$SUFFIX.log" | tee -a "$SUMMARY"

# 외형 진단: 실험 트랙 정적 점검을 직진/최대 조향 bbox로 계산(출발칸 겹침 쌍 포함).
python3 - > "$OUT/geometry_diagnostic_$SUFFIX.json" 2>&1 <<'PY'
import json, sys
from pathlib import Path
import numpy as np, yaml
sys.path.insert(0, 'scripts')
from build_experimental_track import DESTINATION, PROFILE, inspect_geometry, prepare
_, result, _, footprint, _ = prepare(PROFILE)
drive = yaml.safe_load(Path('src/arena_description/config/vehicle.yaml').read_text())['vehicle']['drivetrain']
angles = np.linspace(0, drive['max_steering_angle_rad'], 2001)
half_x = max(drive['wheel_radius_m'] * np.cos(angles) + drive['wheel_width_m'] / 2 * np.sin(angles))
half_y = max(drive['wheel_radius_m'] * np.sin(angles) + drive['wheel_width_m'] / 2 * np.cos(angles))
out = {'drivetrain': {k: drive[k] for k in ('wheelbase_m', 'track_width_m', 'wheel_radius_m', 'wheel_width_m', 'max_steering_angle_rad')},
       'grid_slots': [{k: float(g[k]) for k in ('x', 'y', 'yaw')} for g in result['grid_slots']]}
for name, fp in (('straight_footprint', footprint),
                 ('max_steer_wheel_envelope', {'length_m': max(footprint['length_m'], drive['wheelbase_m'] + 2 * half_x),
                                               'width_m': max(footprint['width_m'], drive['track_width_m'] + 2 * half_y)})):
    report = inspect_geometry(DESTINATION / 'world.sdf', result, fp)
    out[name] = {'footprint': {k: float(v) for k, v in fp.items()}, 'pass': report['static_footprint_checks_pass'],
                 'blocked_grid_slots': report['blocked_grid_slots'],
                 'overlapping_grid_pairs': report['overlapping_grid_pairs'],
                 'routes': {k: {'samples': v['samples'], 'blocked': len(v['blocked_samples'])} for k, v in report['routes'].items()}}
print(json.dumps(out, indent=2))
PY
step geometry_diagnostic $?

# 나머지 회귀(rclpy 노드/하위 프로세스를 실제로 띄우는 5개 파일 제외).
python3 -m pytest -q -p no:cacheprovider tests src/arena_vehicle_interface/test \
  --ignore=tests/test_lidar_shutdown.py --ignore=tests/test_local_pursuit.py \
  --ignore=tests/test_replay_imu_probe.py --ignore=tests/test_replay_observer.py \
  --ignore=tests/test_wsl_d3d12_lifetime.py > "$OUT/pytest_broad_$SUFFIX.log" 2>&1
step broad $?
tail -8 "$OUT/pytest_broad_$SUFFIX.log" | tee -a "$SUMMARY"

# C++: 순수 방정식 단위 검사를 저장소 밖 임시 경로에서 빌드·실행. Gazebo 플러그인은 실행하지 않는다.
TMP="$(mktemp -d)"
g++ -std=c++17 -Wall -Wextra -I src/arena_gazebo/include src/arena_gazebo/test/test_drivetrain.cpp \
  -o "$TMP/test_drivetrain" > "$OUT/cpp_$SUFFIX.log" 2>&1
step cpp_unit_build $?
"$TMP/test_drivetrain" >> "$OUT/cpp_$SUFFIX.log" 2>&1
step cpp_unit_run $?
# 플러그인은 임시 build/install 경로에서 컴파일만 한다(저장소 build/install 미변경, 실행 없음).
colcon --log-base "$TMP/log" build --packages-select arena_gazebo --base-paths src/arena_gazebo \
  --build-base "$TMP/build" --install-base "$TMP/install" > "$OUT/colcon_arena_gazebo_$SUFFIX.log" 2>&1
step plugin_compile $?
rm -rf "$TMP"

if [ "$FAILED" -eq 0 ]; then echo "VERDICT=PASS" | tee -a "$SUMMARY"; else echo "VERDICT=FAIL" | tee -a "$SUMMARY"; fi
exit "$FAILED"
