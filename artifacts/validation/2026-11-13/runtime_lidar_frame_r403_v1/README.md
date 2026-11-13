# r403 LiDAR 원출처 frame 선택 — 검토·회귀 준비

2026-11-13. 공통 소스와 장치에 쓰지 않고 staged7모듈과 필요한 검사를 준비했다. Windows에서 Python AST·정적 import 폐포·묶음 해시·검사 대상 이름만 확인했다. **후보/pytest/WSL/Jetson/ROS 노드/주행은 실행하지 않았다.** 지정 실행자가 아래 동결 묶음을1회 검증한다. 인계 후 root의 명시적 반환 승인 전에는 문서도 수정하지 않는다.

## 검토 결론과 변경

처음 staged 변경에서 등록 frame을 두 개로 늘리면서 최근 관측을 frame별로 나누지 않는 P2 부작용을 발견했다. `RecentScenes.observe`는 기존 단일 `laser_frame` 전제로 epoch만 비교했으며, 직접 `sweep_support`에 전달한 과거 장면도 frame을 비교하지 않았다. 따라서 같은 epoch의 서로 다른 등록 frame 장면이 혼합될 수 있었다. 정상 고정 `MotionInput`의 ingress에서는 다른 frame을 거절하지만 공개 하류 함수의 경계도 보존해야 한다. root 승인에 따라 staged `observation_support.py`에서 frame 변경 시 cache reset과 sweep의 현재/source frame 일치 필터만 추가했다. 두 반례와 기존 같은 frame 회귀를 준비했다.

그 외 검토한 전달 경로에서는 추가 P1/P2 결함을 발견하지 못했다. 판정 한계·TTL·점유·공백·원 취득시각·운동수치·속도는 바꾸지 않았다.

| 공통 이관 후보 | 변경과 확인할 계약 |
|---|---|
| `lidar_source_frame.py` | 등록 문자열은 `laser_frame`, `c1_unmounted_preview` 두 개뿐이다. 타입을 문자열로 강제 변환하거나 임의 이름을 허용하지 않는다. 이름은 장착·축 교정 증거가 아니다. |
| `lidar_motion.py` | `MotionConfig` 마지막에 `lidar_frame_id='laser_frame'`를 추가한다. 앞선 positional 필드와 옛 mapping 기본값을 유지한다. 숫자 유한성 검사에서 이 문자열만 분리한다. |
| `lidar_motion_ros.py` | `motion_lidar_frame_id`를 문자열 원값으로 전달하며, project의 입력 frame을 설정과 exact 비교한다. 기존 숫자 파라미터와 시간환경 검사는 그대로다. |
| `scene_motion.py` | native/legacy 장면 변환 모두 설정 frame과 exact 일치해야 한다. host estimate 선택·false verified·epoch 결합·미래/만료·측정 배열 순서는 그대로다. |
| `observation_support.py` | 두 등록 source frame만 인정한다. 최근 장면 캐시는 frame 변경 시 비우며 sweep에서 다른 frame을 지지 근거로 쓰지 않는다. 같은 frame 결과/TTL은 유지한다. |
| `corridor_observer.py` | 이미 base_link로 변환한 장면의 출처 허용 목록만 변경한다. 기존 `(source_frame_id, source_epoch, side)` 기억 분리도 그대로다. |
| `linear_observation.py` | 출처 허용 목록만 변경하고 기존 관측 폭/거리/시각/점유 검사를 유지한다. |

`staged/.../__init__.py`는 `extend_path`를 사용하는 수동 overlay용 보조이며 공통 이관 대상이 아니다. 고정 실행 묶음에서는 기존 common `__init__.py`와 실제 package 배치를 사용한다. [source_changes.patch](source_changes.patch)는 위7모듈만 포함하며, 원6파일은 `before_sources.zip`에 보존했다(등록 helper는 새 파일).

## 남은 고정 frame과 연결 요구

`src/`, `scripts/`, `config/`에서 `laser_frame`, `c1_unmounted_preview`, `lidar_frame_id`를 검색했다. 실제 운영 변환의 기존 exact gate는 위 staged 세 경로(MotionInput project/native scene/legacy scene)에 모두 반영했다. 나머지 `MeasurementScan`, `scene_from_scan`, `scene_from_measurements`의 `laser_frame` 기본값은 옛 기록·합성 입력 계약이므로 유지한다. `native_traffic_contract.py`와 `validate_basic_autonomy.py`, `validate_lidar_control_lab.py` 등의 고정 `laser_frame`은 simulation/evaluation 계약이며 이번 운영 consumer 연결에 사용하지 않는다. 최종 vehicle YAML의 frame 표시는 시뮬레이션 센서 링크 계약이고 C1 producer 이름을 자동으로 덮어쓸 근거가 아니다.

운영 planner와 독립 guard **양쪽**에 `motion_lidar_frame_id: c1_unmounted_preview`를 명시해야 한다. 이 파라미터가 빠지면 기본 `laser_frame` 기대값이 C1을 거절한다. r399 초기 프로필에는 이 consumer 파라미터가 없었으므로 프로필 생성/검증 연결은 root 후속 범위다. 이 폴더는 r399나 공통 설정을 수정하지 않는다.

`C1NativePreview`/`C1SampleClock`의 원 `c1_unmounted_preview`, `unverified_estimate` 및 false 검증 표시는 변경하지 않는다. sensor→base_link의 수치 변환은 명시 MotionConfig를 쓰며 TF나 프레임 이름으로 설치 자세를 추정하지 않는다. 실제 설치 위치·IMU축·시각 정확도와 구동 허가는 미검증이다. `LocalScene`에는 원래 time_basis 필드가 없으므로 새 필드가 생겼다고 주장하지 않는다. 원 scan·trace에는 time_basis를, 각 ray에는 frame/epoch/취득시각을, projection 진단에는 `hardware_timing_verified=false`를 보존하는 기존 표현을 확인한다.

## 지정 검사

[test_selection.json](test_selection.json)의14개 선택 항목만 실행한다. 새2개 파일 전체, 기존 `test_native_measurement_motion.py`의 원 native 배열 계약, numeric MotionConfig1검사, MotionInput legacy4검사, 관측 cache/점유3검사, linear 관측3개 항목이다. 매개변수화된 항목은 여러 검사로 수집되므로 실제 개수는 결과 report에서 확인한다. 다른 기존 파일은 fixture import만 되며 전체 검사를 수집하지 않는다.

- 기본 C1 거절과 명시 문자열 설정, 잘못된 타입·다른 등록 frame·알 수 없는 frame 거절.
- 명시 C1+`host_estimated_v1`에서 동일 CW/CCW 기하, 각 ray 원 frame/epoch/시각·원시 순서 유지. raw source epoch와 기존 Cantor 결합 scene epoch를 구분한다.
- 미래/만료/잘못된 시간정책·basis, sim clock·verified 승격·환경 혼합·이력 부재 거절.
- snapshot+JSON에서 문자열 설정/원 배열/false verified 복원, 옛 설정 자동 승격 금지, legacy LaserScan 계약 유지.
- 실제 C1 producer 함수의 합성 SDK 입력→JSON→실제 `std_msgs/String` CDR→decode→MotionInput/scene. 원 취득시각·무반환·미검증 표시 보존. ROS 메시지 라이브러리만 쓰며 `rclpy.init`, 노드, spin, publisher는 없다.
- 하류 두 frame의 수치/unknown 검사 동등성, 등록되지 않은 frame 거절, 같은 epoch의 교차 frame cache/sweep 반례2개.

## 고정 실행 명령·종료

실행 자원을 배정받은 담당자만 아래1회 실행한다. 새 root와 receipt가 이미 있으면 중단하며 자동 삭제/재시도하지 않는다. 준비 이후 common이 바뀌어도 실행은 독립 `bundle_sources.zip`의148개 파일만 읽는다. native kernel build나 장치 연결은 필요 없다. 여기서 native 검사는 원천 측정 배열 표현을 뜻하며 C++ backend 성능 시험이 아니다.

```bash
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
python3 -B artifacts/validation/2026-11-13/runtime_lidar_frame_r403_v1/execute_isolated.py \
  --root /home/jinhyeong/it-arena-r403-frame-20261113-a \
  --receipt artifacts/validation/2026-11-13/runtime_lidar_frame_r403_v1/verify_v1
```

pytest 전체 상한180초, timeout은 프로세스 그룹 TERM5초 후 필요시 KILL5초로 회수한다. PID/종료코드/강제 종료 여부를 보존한다. 실패·skip·source hash 불일치·다른 설치본의 arena import는 성공으로 처리하지 않는다. source manifest가 다른 경우 실행을 시작하지 않는다. 설치·빌드·추가 시험은 이 명령에 포함하지 않는다.

`verify_v1/execution.json`, `results.zip` 안 `tests.log`와 `results/report.json`을 회수한다. report에는 실제 수집/통과/skip 수, 실패 원문, staged7모듈 및 실제 import된 arena 모듈 경로/해시, source 변경 목록을 남긴다. 준비 manifest와 추출 source 모두 전후 비교한다. 단위검사 성공을 실물 frame/설치 교정·타임스탬프 정확도·물리 정지나 주행 성공으로 확대하지 않는다.

동결 bundle SHA256: `b87f08f49193669a1ac6aea9df25de35654ac8f5ddcf7d389665a2b110869eb2`.
묶음 manifest SHA256: `18a8f9d671f850a2bc5cdbae0a4af3b74cb10bf572c2b0d51beb06409bf79c1b`.
전체 준비 파일 해시는 `manifest.json`에서 관리한다. 정적 준비 재현용 `prepare_sources.py`는 이미 존재하는 묶음을 덮어쓰지 않으며, 인계 후 재실행하지 않는다.
