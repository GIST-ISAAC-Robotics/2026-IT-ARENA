# r401 최종 기하·선정 정책의 ROS profile

상태: **Windows에서 데이터/AST 합성과 파일 생성만 완료. pytest·ROS·WSL·장치 실행은 하지 않았다.** 공통 및 r377/r399/r400 산출물을 수정하지 않았다. 이 profile은 실제센서 비구동 결합을 위한 준비이며 주행 채택·실물 교정·운영 완성 판정이 아니다.

현재 검사 대상은 **prepared_v2**다. v1 생성물과 당시 `manifest_pre_frame_v1.json`은 frame 연결 전 준비 기록으로 보존한다. v2의 매개변수 변경은 양쪽의 `motion_lidar_frame_id: c1_unmounted_preview`와 기대 MotionConfig의 같은 문자열뿐이다. r399 원 generated는 불변이다. r403 공통 이관 완료본을 `manifest.json`에 고정했으며 v1 당시 manifest를 현재 소스 검사에 사용하지 않는다. `bindings.json`의 frame 수용 코드 대기 표시는 false로 갱신했지만 profile의 실제 ROS 검사와 producer 결합은 아직 실행하지 않았다. `stream_contracts`는 r399 생성 당시의 원 기록을 보존한다.

[prepared_v2/runtime_profile.yaml](prepared_v2/runtime_profile.yaml)은 `local_pursuit` 122개, `lidar_safety` 40개 매개변수를 가진 한 파일이다. r399 `generated_v1/runtime_geometry.yaml`의 소유 필드를 그대로 포함하고, r377 저장 schema5 header에서 현재 선택한 follow/execution 정책을 ROS 매개변수로 복원했다. simulation launch나 동적 prefix를 import하지 않는다. 가변 환경 경로인 `g1_trace_path`만 실행 명령에서 새 절대 파일로 지정해야 한다.

- [expected_header.json](prepared_v2/expected_header.json): 실제 노드 생성 후 `FollowConfig`, `HandoffExecutionConfig`, 두 노드의 `MotionConfig`와 대조할 전체 값.
- [parameter_mapping.json](prepared_v2/parameter_mapping.json): 모든 follow/execution 필드의 ROS 매핑, 현재 생성자 기본값을 유지하는 항목, 파생 필드의 근거.
- [bindings.json](prepared_v2/bindings.json): 원 source frame, 두 노드 topic remap, r399 motor JSON과 host 전달값. host를 실행하거나 새 host schema를 등록하는 파일은 아니다.
- [source_evidence.json](prepared_v2/source_evidence.json): 원 `sources.zip` 안 simulation launch와 실제 launch 명령의 해시. 생성기 `prepare_profile.py`는 이 기록과 AST/JSON만 읽으며 제어기를 실행하지 않는다.

## 원 기록과 최소 차이

원본은 r377 `run1/follow_decisions.jsonl` 첫 header와 `sources.zip`의 실제 launch 코드, `launch_command.json`이다. r377 launch가 선택한 `g1_open_1p5`의 ROS max/open **1.5 m/s**, min **.20 m/s**, acceleration **1.0 m/s²**를 유지한다. FollowConfig의 기본 경계 속도는 기존 `min(1.0, ROS max)`인 **1.0 m/s**다. motor feedback의 **5.0 m/s** 입력 유효성 한계와 혼동하지 않는다. speed를 임의의 저속 운영 기본값으로 바꾸거나 vehicle의 시뮬 최고속도를 가져오지 않았다.

| 항목 | r377 저장/실행 | 이번 선택과 이유 |
|---|---|---|
| `track_m`, `wheel_radius_m`, `wheel_width_m` | 당시 header에 필드 없음 | 현재 FollowConfig에 최종 `.1239/.0325/.0261 m` 명시. r388/r399 final 기하 연결 |
| `known_structure_sectors_json` | marker Z `.14 m`, 5구간 | r399 final vehicle YAML의 marker Z `.09 m`, 4구간. 아래 설명 |
| 시계/표현 | sim time, `native_simulation_v1`, verified true | 실 ROS clock, `native_estimated_v1`, `host_estimated_v1`, verified false. 모든 MotionConfig 숫자·gear는 유지 |
| 계산 정책 선택 | r377 전용 동적 prefix | 공통 r398/r400의 `g1_runtime_policy=cad_runtime_r377_v1` 명시. process/fresh/worker/native 조건 유지 |
| 실제 제출 방식 기록 | 전용 두 슬롯 | `runtime_dual_latest_v1` receipt. `fresh_native_v1`은 등록 선택 조건이며 실제 latest 제출과 구분 |
| 센서 topic/frame | 시뮬 생산자 | r399 runtime topic 제안과 원 실물 frame 보존. r402 명시 domain/topic·r403 frame 수용 코드 이관 완료, 실제 결합은 후속 |
| `MotionConfig.lidar_frame_id` | 필드 없음, 당시 laser_frame 고정 | r403 명시 계약의 `c1_unmounted_preview`. 원 scan frame을 재표시하지 않음 |

위 외 FollowConfig 값은 원 header와 같고 **ExecutionConfig 전체 및 MotionConfig 숫자는 동일**하다. `current_lane_headroom_mps=.061225042014404256`도 원 header를 유지한다. 원 `sensor_wall_timeout_s=30.0`은 양쪽에 보존하며 실물 적합성을 새로 검증한 수치가 아니다. scan/source/command/경로 TTL이나 점유·미관측 판정은 변경하지 않는다.

r347/r377의 marker Z `.14`는 50 mm 판의 높이 `.115..165 m`가 C1 광학면 `.1346 m`와 교차한다. 그래서 후방 중심180°·반폭9.5918298933°의 다섯째 구간이 있다. 최종 YAML의 Z `.09`는 `.065..115 m`이므로 그 평면과 교차하지 않는다. 첫 네 구간의 각도·거리대는 그대로이고 marker 구간만 빠진다. 원 r377의 5구간 결과나 r396/r398 동결 비교를 새 4구간 결과로 소급하지 않는다. 거리대 밖 유효 반환을 보존하며 unknown을 free로 채우지 않는다.

공통 `g1_runtime_policy` 기본값은 계속 `off`다. 이 파일에서만 명시 선택한다. `g1_g2_simulation_trial=true`는 기존 알고리즘 생성자 선택 이름이며 이 profile의 `use_sim_time=false`를 바꾸지 않는다. 원 출발 방식인 `vision_execution=process`, `official_4red_local_candidate`, `race_control_mode=legacy`를 보존한다. B3/운영 START·재개 연결까지 완료했다고 표시하지 않는다.

## 실행 담당에게 넘길 고정 검사

현재 r400 및 r403이 이관된 공통 소스와 기존 native 확장이 있는 WSL 환경에서, repository root를 cwd로 한다. 센서/host/serial/차량을 시작하지 않는 **한 묶음**이다. domain219 사용은 root가 기존 작업 종료를 확인한 후 배정한다. 외부 상한180초, 실패 시 다음 명령을 진행하지 않고 원 로그·소유 자식 종료를 회수한다.

```bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q -p no:cacheprovider artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/test_profile.py
python3 artifacts/validation/2026-11-13/runtime_profile_r399_v1/generate_profile.py check \
  --directory artifacts/validation/2026-11-13/runtime_profile_r399_v1/generated_v1 \
  --profile artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml \
  --motor-config artifacts/validation/2026-11-13/runtime_profile_r399_v1/generated_v1/motor_feedback.json
ROS_DOMAIN_ID=219 python3 -m pytest -q -p no:cacheprovider \
  artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/test_profile_ros.py
```

마지막 검사는 실제 ROS 두 노드를 만들고 모든 명시 parameter와 전체 config를 exact 비교한다. 입력 메시지를 보내지 않으며 G1의 두 계산 자식과 기존 vision 자식을 정상 회수한다. trace header/footer, `runtime_dual_latest_v1` receipt, `/drive → /drive/safe`와 sensor remap을 확인한다. 양수 계산/센서→명령 통합/처리시간 검사가 아니다. source/profile hash가 다르면 manifest를 자동으로 고쳐 계속하지 않는다. 현재 v2는 r403 이관 뒤 동결한 common 소스를 기준으로 실행한다. ROS setup이 넣은 PYTHONPATH를 보존하여 source 경로만 앞에 추가하며, 별도 실행기에서 환경을 지우거나 교체하지 않는다.

## 이후 실제 입력에 연결할 준비 명령

아래는 후속 실행 형태를 고정한 것이며 실행하지 않았다. root가 producer와 같은 ROS domain을 배정하고 위 검사를 끝낸 다음, 두 터미널에서 같은 profile을 쓰면 된다. 먼저 새 `observed_v1` 디렉터리를 만들고 기존 trace가 없는지 확인한다. 기존 파일을 덮어쓰지 않는다.

```bash
ros2 run arena_autonomy lidar_safety --ros-args \
  --params-file "$PWD/artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml" \
  -r /scan/native:=/hardware_runtime/c1/native

ros2 run arena_autonomy g1_g2_handoff_trial --ros-args \
  --params-file "$PWD/artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml" \
  -p g1_trace_path:="$PWD/artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/observed_v1/follow_decisions.jsonl" \
  -r /scan/native:=/hardware_runtime/c1/native \
  -r /camera/color/image_raw:=/hardware_runtime/camera/image_raw
```

이 명령은 host나 포트를 열지 않는다. 같은 domain에 실제 wire가 활성인 host를 연결하는 것은 이번 비구동 준비에 포함되지 않는다. `bindings.json`의 host 제안은 max speed1.5/최종 steering/기존 motor JSON/IMU topic만이며 `--enable-drive-wire`, ARM, DRIVE wire 활성화와 firmware flash는 없다. common host의 wire 기본 OFF를 유지한다.

남은 실제 연결은 다음 세 가지다.

1. r402가 공통 이관되어 C1/camera producer에 `--ros-domain`/`--topic`을 명시할 수 있다. 기본은 여전히 domain216·`/hardware_preview/...`다. 운영 소비자와 같은 domain 및 위 runtime topic으로 실제 결합하는 시험은 남아 있다. 기존 producer 재시작이나 topic 교체를 이 작업에서 수행하지 않았다.
2. 원 C1 frame `c1_unmounted_preview`를 공통 이관된 r403의 `motion_lidar_frame_id`로 양쪽에 명시했다. r403 이전에는 `laser_frame`만 받아 topic remap만으로 계산이 열리지 않는다. 원 frame을 재표시하지 않는다. 이 profile의 실제 검사와 센서 결합이 필요하며 lidar yaw0, IMU roll/pitch0, encoder sign+1, camera 위치·축은 r399의 임시 가정으로 실장 교정이 아니다.
3. B3 최초 ARM/시각 출발/STOP→재개와 실제 MCU 상태의 운영 연결은 후속이다. 계산 정책·설정 파일·ROS 생성 검사 통과를 물리 출력 허가나 실차 완주로 확대하지 않는다.
