# r269 — 추정 시각의 명시 사용과 검증 상태 분리

실제 센서의 추정 시각을 `validated_sensor_acquisition` 또는 `timing_verified=true`로 바꾸지 않고 사용할 준비를 했다. **기본값은 그대로 `verified_only`**다. ROS의 `motion_scan_time_policy=host_estimated_v1`은 현재 `hardware_shadow_v1`·실제 호스트 시계·native 추정 입력·both 보정·검증 false 조합에서만 허용한다. `/hardware_preview/drive`는 진단 출력이며 `/drive`나 `/drive/safe`에 연결하지 않는다.

## 구현과 저장 대조

선택한 계약은 운동 이력·점군 투영·추종기 내부 재투영·계획 snapshot·현재 재검사·기록/재생까지 전달한다. 원 `time_basis=unverified_estimate`와 false 검증값을 유지한다. 기본 snapshot에는 새 필드를 넣지 않아 기존 기록 형식을 유지했다. frame·미래/오래된 시각·운동 이력 공백·unknown·STOP 판정은 바꾸지 않았다.

[저장 대조](replay_result.json)는 기존 r225의 **632계획·592현재 재검사에서 전체 결과 동일**이다. 같은 센서/운동 숫자에 시간 출처와 선택만 바꾼 시험이다. 실물의 상대 시각 오차나 주행 성능 시험은 아니다. 실제 입력이 없어도 가짜0 엔코더를 넣어 계산을 진행하는 기능은 없다.

[관련208검사](tests.log), [추가 계약/의존성32검사](tests_contract.log), [변경분과 직전 출처2검사](provenance.log)를 통과했다. 직전 r268 전체 출처215검사는 별도로 보존한다. 두 패키지 colcon 빌드는11.9초에 완료했다.

첫 검사에서 계측 도구가 r247의 선택형 C++ 모듈을 누락 Python 파일로 보며 실패했다. `.so`를 빌드해도 Python 전용 탐색이므로 해결되지 않았다. 명시 등록한 C++ 모듈의 세 소스와 `setup.py`를 보존하도록 import 폐포 v2를 수정했다. 모르는 내부 모듈이나 빠진 C++ 소스는 계속 거절한다. 실제 바이너리가 빌드/설치됐다고 표시하지 않는다. 초기 두 실패 로그를 보존했다. 별도 WSL 직접 호출의 ROS 경로 수집 실패는 검증한 namespace와 LF 셸 파일로 재실행했다. 배포판 강제 재시작은 하지 않았다.

## 실제 Jetson에서 확인한 범위

`probe_g1_hardware_shadow.py --estimated-time`과 IMU `--imu-clock fifo_tick`, 기존 C1 preview를 약20초 연결했다. [독립 감사](audit.json)·[노드](node_summary.json)·[IMU](imu_summary.json)·[C1](c1_summary.json)·[종료](supervisor.json)를 남겼다.

- C1 203배열/102,548점의 원 메타데이터·수신 일치, IMU3,798개 공통 수락·거부0.
- 추정 정책 선택과 false 검증 표시가 실제 상태/기록에 남았다. 전용0명령1,298회, 실제 구동 없음.
- **미장착 frame `c1_unmounted_preview`·엔코더0·카메라 없음**이다. 따라서 실제 제어 계산/출발·운동 보정 완료의 증거가 아니다.
- C1의 기존 nominal200µs는 배치 중첩199회/최대4.002ms, 수신보다 미래인 끝점127회/최대2.382ms가 남았다. r260 추적 시계 후보는 이번에 사용하지 않았다.
- 계산 자식과 노드/센서 세 프로세스 정상 종료, IMU 비활성/C1 정지 확인, 두 포트 해제. 펌웨어·전력·구동 설정은 미변경이다.

첫 실행은 C1 포트 q 접근권이 없는 상태를 preflight에서 발견했지만 실행기에서 중단하지 않아 SDK가−11로 종료했고 C1 입력0으로 전체 실패했다. 접근권 부재와 종료 코드를 구분해 보존했다. C1의 특정 USB 장치에만 q ACL을 복구하고, 실행기에 접근권 검사 및 센서 실패 시 조기 정리를 추가한 별도 실행이 통과했다. 원 소스/manifest·실패 ZIP을 덮어쓰지 않았다.

원시 데이터는 Git 제외 `build/r269_recovered/`와 Jetson `/home/q/it_arena_estimated_time_r269_20261112/`에 있다. [회수 해시](recovery_receipt.json), [첫 묶음](bundle_receipt.json)·[수정 묶음](bundle2_receipt.json), 실행 소스를 보존했다. 다음은 C1 추적 시계의 명시 선택 연결과 실제 장착/엔코더 준비이며, 이번 허용을 실물 운영 출력 허가로 확대하지 않는다.
