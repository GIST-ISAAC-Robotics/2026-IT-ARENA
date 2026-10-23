# 센서 점검 전용 휴대 묶음 사용

목적은 **이미 발행 중인 센서 ROS 메시지를 제어기 없이 보존**하는 것이다. 설치·장치/포트 연결·모터/서보 구동·Jetson 전원 변경을 자동 실행하지 않는다. 전체 저장소의 상세 결과와 실패는 `docs/hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md` §10에 있으며 최소 묶음에는 그 긴 이력을 중복하지 않는다.

## 포함·제외

- 포함: 기록기, 시계/토픽 프로필, Humble/Jazzy 메타데이터 호환층, 파일 감사기, 설치 환경 사전점검·시계별 간격 진단·센서 입력 형식 관찰, 관련 순수 회귀와 계약 모듈.
- 제외: 제어기/MCU/보호층 실행기, 장치 드라이버, 캘리브레이션 계수, 접속 자격 증명, 실제 원시 bag, 대용량 로그, 펌웨어, SDK 빌드.
- `bag_contract.py`에 비교용 명령 토픽 이름이 정의돼 있어도 `sensor_probe` 허용 목록에는 없다. 모터축 센서는 `/drive_motor/encoder`의 단일 `JointState`이며 좌우 바퀴 두 채널을 만들어내지 않는다.

## 1. 전달 전후 파일 확인

PC에서 전체 저장소의 `prepare_jetson_bundle.py --sensor-probe-tools-only`로 만들고 `verify_sensor_probe_bundle.py`로 archive/manifest를 대조한다. 검증기는 압축을 풀거나 내부 코드를 실행하지 않는다. 해시는 전달 무결성 확인이며 서명/실행 허가가 아니다. Python 3.10 문법 검사는 실제 Python 3.10·ARM 실행 검사가 아니다.

```bash
python3 scripts/verify_sensor_probe_bundle.py BUNDLE.tar.gz MANIFEST.json --output new_check.json
```

사용자가 전달을 승인한 뒤에만 새 빈 작업 폴더에 풀어 사용한다. 기존 저장소에 덮어쓰지 않는다. 최소 묶음은 colcon 빌드가 아니라 명시 Python 소스 경로로 실행한다.

## 2. 준비 환경

설치된 ROS 2 Humble(실물) 또는 Jazzy(PC)의 setup을 현재 셸에서 읽고 Python 의존성을 확인한다. 새 패키지를 자동 설치하지 않는다. 필요한 항목은 `rclpy`, `rosbag2_py` 및 MCAP writer, `rosidl_runtime_py`, `sensor_msgs`, 테스트용 `pytest`다. 선택 profile의 기록·감사에는 `ackermann_msgs`/제어기 실행이 필요하지 않는다. `audit_sensor_bag.py`는 호환 reader 함수의 의존 파일로만 포함된다.

```bash
# 예: 이미 설치된 Humble만 사용. 시스템 전체 설정을 고치지 않는다.
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 -B -m pytest -q -p no:cacheprovider tests
```

`SequentialWriter.open/write/create_topic/close`가 모두 필요하다. 지원하지 않는 API나 MCAP plugin이 없으면 미완료/오류로 남기며, 다른 저장 형식·암묵적 소멸자 종료로 자동 대체하지 않는다. 공개 Humble 소스의 TopicMetadata에는 `id`가 없어 설치 API의 속성 유무에 따라 해당 인수만 분리한다. 설치된 Jetson 버전과 writer 동작은 현장에서 별도 확인한다.

## 3. 노드 없이 사전점검

현재 23파일 묶음의 `sensor_probe_tools_version=4`에는 다음 사전점검·간격 진단·입력 형식 관찰·선택형 세션 실행이 포함된다. 기존 15파일(버전 미기재/1)·19파일(버전 2)·21파일(버전 3) 묶음은 덮어쓰지 않으며 각 고정 목록으로 검증한다.

```bash
python3 -B scripts/sensor_probe_preflight.py \
  --capture artifacts/first_input/sensor_recording --clock-mode system \
  --sensor-topic /imu/data --sensor-topic /scan
```

ROS 모듈·명시적 writer 종료 API·MCAP 등록·메시지 직렬화·새 경로·디스크 여유를 읽어 확인한다. 노드/장치/writer를 만들거나 부족한 패키지를 설치하지 않는다. `checks_passed`는 실제 전송·저장·교정·주행 승인이 아니다. 이 예시는 stdout만 사용하며 capture 디렉터리도 생성하지 않는다.

## 4. 구동 없는 기록

출력 비활성·드라이버 단독 시험 계획과 발행 토픽/단위/시각 출처를 먼저 확인한다. 기록기는 기본적으로 현재 ROS 도메인을 수신하므로, 의도한 센서가 있는 도메인에서 실행한다. 임의 포트를 열거나 생산자를 실행하지 않는다. system/시뮬레이션 시계는 명시적으로 선택한다.

```bash
python3 -B scripts/record_sensor_bag.py \
  --profile sensor_probe --clock-mode system \
  --sensor-topic /imu/data --sensor-topic /scan \
  --output artifacts/first_input/sensor_recording \
  --max-wall-seconds 60 --max-raw-gib 0.1
```

필요 구간 뒤 Ctrl+C로 정상 종료한다. 시간/용량 상한은 자동 성공 종료가 아니라 **미완료 자료 보존 한도**다. 기본 디스크 여유 검사 10 GiB·운영 중 2 GiB 예약을 유지한다. 요청한 센서가 하나라도 없거나 기록 오류/수신 시계 역전이 있으면 `complete=false`다. `.gitignore`로 원시 CDR/상세 receipt의 우발적 공개를 막지만 외부 업로드 승인까지 대체하지 않는다.

## 5. 파일 감사와 별도 센서 검증

```bash
python3 -B scripts/audit_sensor_probe.py artifacts/first_input/sensor_recording \
  --output artifacts/first_input/audit.json
```

- 파일/토픽/직렬화 바이트·원래 header와 receipt 대응을 확인한다. `record_integrity_verified=true`는 이 의미뿐이다.
- 원래 header 0/역전과 값 자체를 고쳐 쓰지 않는다. 원천 clock·취득 시각·축/단위·range/quality는 센서별로 추가 검증한다.
- `sensor_acquisition_time_verified`, `controller_replay_eligible`, `hardware_verified`, `ready_to_drive`는 false다. 기존 제어기 재생기로 돌릴 수 있는 자료라고 표시하지 않는다.
- C1 정렬/복제 전 점과 SDK timestamp, B0183 노출/PTS, IMU 원시/FIFO 시각 등이 ROS 메시지에 없으면 별도로 보존한다. 이 기록기가 잃어버린 상류 자료를 복원하지는 않는다.
- 중단/비정상 종료 자료는 버리지 말고 원본 경로와 오류를 남긴다. 성공할 때까지 기존 폴더를 덮어쓰지 않는다.

## 6. 시계별 간격과 데이터 누락 해석

```bash
python3 -B scripts/analyze_sensor_probe_timing.py artifacts/first_input/sensor_recording \
  --output artifacts/first_input/timing.json
```

전체 파일/CDR/receipt 감사가 먼저 성립해야 진단 결과를 저장한다. callback 단조 시각·recorder ROS 시각·원래 header 간격을 따로 집계한다. header 0/중복/역전·복수 생산자 등으로 단일 시계 주기를 해석할 수 없으면 그 비율을 정상 취득률로 제공하지 않는다. callback부터 writer receipt까지는 큐·역직렬화·쓰기·해시/통계 처리까지의 구간이지 순수 큐 지연이나 디스크 영구 저장 시간은 아니다.

`complete=true`여도 생산자가 보냈지만 DDS/드라이버에서 유실된 표본까지 없다는 뜻은 아니다. 10/23 PC 합성 시험에서는 큰 RGB 영상의 상류 유실이 있었고, 같은 호스트 전용 SHM 16 MiB 시험 설정에서만 제한 20장 전량을 확인했다. 이 설정은 휴대 묶음에 자동 적용하지 않는다. 실제 첫 기록에서는 생산자 순번·촬영 시각·해상도/인코딩·QoS·전송 환경과 수신 수를 함께 보존하고, 고속/장시간 적합성은 별도 판정한다.

## 7. 센서 입력 형식 관찰

```bash
python3 -B scripts/inspect_sensor_probe_inputs.py artifacts/first_input/sensor_recording \
  --expect-frame /scan=laser_frame --output artifacts/first_input/inputs.json
```

전체 CDR 감사 뒤에만 결과를 저장한다. 단일 모터축 이름/유한 위치·각속도, RGB/BGR·행 패딩, CameraInfo 미교정 표시, IMU의 미제공/미지 공분산·비유한값, 스캔 메타데이터·무응답 개수를 분리한다. `--expect-frame`은 이름만 비교하며 실제 장착 축을 입증하지 않는다. IMU 등 나머지 frame은 실제 구성에 맞춰 명시해야 하며 생략한 항목은 미확인으로 표시한다.

이 보고서는 `findings`가 없더라도 동적 제어기 입력 적합성·시간 보정·노면/차량 관측·실물 운용 승인을 주지 않는다. 값·순서·시각·교정 계수와 운영 설정을 바꾸지 않으며 `+Inf`를 자유 공간이나 최대 거리로 채우지 않는다. CameraInfo 교정이나 채널 이름이 맞다는 이유만으로 실제 렌즈 교정/색상 정확성이 확인되는 것도 아니다.

## 8. 기록 오류와 종료의 한계

쓰기 오류와 종료 오류는 별도로 보존하며 Python 기록 경로는 `close()`를 한 번만 호출한다. 종료 오류가 있으면 `complete=false`와 실패 종료 코드를 유지한다. 실제 디스크가 멎거나 native writer가 반환하지 않는 경우는 별개다. 현재 60초 join 제한이 모든 스레드를 강제 종료하는 것은 아니므로, 정상 종료가 확인되지 않은 자료를 감사 통과/완료로 쓰지 않는다. 원본 보존 후 담당자가 프로세스 상태를 확인해야 하며 파일 강제 수정이나 자동 복구로 숨기지 않는다.

## 9. 계획 시간 뒤 종료를 요청하는 선택형 세션

Linux/WSL에서만 사용하며, §4의 직접 실행과 **택일**한다. 센서 생산자는 따로 준비되어 있어야 한다.

```bash
python3 -B scripts/sensor_probe_session.py \
  --output artifacts/first_session --clock-mode system \
  --sensor-topic /imu/data --sensor-topic /scan --duration 20 --max-raw-gib 0.1
```

자식 기록기의 PID가 일치하는 ready 뒤 20초를 재고 SIGINT로 정상 종료를 요청한다. 정상 종료 유예 75초 뒤에도 반환하지 않으면 **자기가 만든 자식 프로세스 그룹에만** TERM→5초→KILL→5초를 적용한다. 무관한 센서 생산자/ROS 프로세스는 종료하지 않는다. 기본 기록기나 주행 launch에 자동 연결되는 기능이 아니다.

- 원시 자료는 `first_session/sensor_recording`, 감독 결과는 `first_session/session.json`, 자식 출력은 `first_session/recorder.log`에 보존한다. 기존 경로는 재사용하지 않는다.
- `complete=true`는 계획된 종료·자식 exit 0·전체 파일/CDR 감사가 모두 성립한 경우뿐이다. 준비 실패, 실행 자체 실패, 빠른 종료(0 포함), 강제 종료, 취소, 감사 실패는 성공이 아니다.
- `spawn_failed`는 자식을 만들지 못한 상태로 PID/exit가 없다. `process_started=false`이며 `process_exited=null`을 정상 종료로 읽지 않는다.
- OS가 프로세스를 끝내지 못하면 `process_exited=false`와 PID가 남는다. 원인을 확인하지 않고 같은 경로로 재실행하거나 파일을 고쳐 통과시키지 않는다.
- 세션 감독 자체의 전원 상실·native 디스크 멎음은 보장 범위 밖이다. 정상 기록 여부를 차량 안전·센서 교정·제어기 재생 허가로 확대하지 않는다.
