# r500 Jetson 현장 설치 준비

2026-11-14. **소스·ROS overlay·ARM 의존성 준비 완료, 실물 운용 설정은 미선택**이다. 센서·ESP32 포트, ARM/flash/출력, 저장 입력 benchmark는 실행하지 않았다. 기존 네트워크·시스템 설정과 이전 설치본은 변경하지 않았다.

## 준비 위치와 확인

- Jetson: `isaac-jetson`, `aarch64`, Python3.10.12, `/opt/ros/humble`. 시작 시 저장공간27GB·가용메모리5,824MiB, 관련 실행PID0.
- 새 독립 루트: `/home/q/it-arena-field-r500-20261114-a`.
- **최종 환경 진입점:** [activate_r500_v2.bash](activate_r500_v2.bash). 실제 ROS domain을 명시한 깨끗한 Bash에서 source한다. ROS 두 패키지의 `install/local_setup.bash`, 동결 scripts와 전용 `deps`를 연결한다. 기본 `ROS_LOCALHOST_ONLY=1`이며 네트워크 설정을 변경하지 않는다. 노드·운용 YAML·장치·펌웨어를 시작하거나 선택하지 않는다. 초기 v1은 pyserial 경로가 없는 보관본이다.
- r491 ZIP SHA `73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965`, manifest SHA `ee450fef0d971d2b94e75544420de5e13d6e9e694b4d2a12bdf6f848581098a5`. **721파일/CRC·ZIP SHA·검증 당시 공통 원문 일치, 원격 추출 후와 설치 후 소스 변경0**. [로컬 검증](local_source_verification.json) · [추출 영수증](remote_receipts/extraction_receipt.json).
- `arena_autonomy`·`arena_vehicle_interface` overlay 빌드 exit0. 설치 Python231개 SHA 일치, ROS executable17개의 import 경로가 모두 새 `install` 아래다. 운영/host/profile/limits/영점/jog/preview CLI9개의 `--help` exit0. [최종 검증](remote_receipts/install_verification.json).
- native는 r470/r493의 `setup.py`와 C++4개가 현재 소스와 동일하여 검증본을 복사했다. r404는 구형이라 제외했다. 설치 ARM `.so` SHA **`5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff`**, ABI1·5함수 확인. 빌드 캐시도 같은 바이너리를 사용해 C++ 재컴파일0.
- `deps/serial`은 기존 r404 환경의 pyserial3.5를36파일 SHA 대조해 복사했다. 첫 검증의 `serial` 누락은 [원 실패](remote_receipts/install_verification_attempt1.json)에 보존했다. 시스템 pip/apt 설치는 없었다.
- C1 실행 파일: `bin/stream_c1_native_preview`, SHA `d8f7d2a02ef47786022206e2a0278c6e7edc2d3ddb40bed2254706b590748e2d`. r204/r166b의 동일 검증 binary와 r166b의 현재 C++ 원문 일치를 확인했다. 최초 r166 소스는 startup 처리 차이로 제외했다. ELF aarch64·동적 의존성 확인만 했으며 실행하지 않았다. [재사용 내역](remote_receipts/reused_dependencies.json).
- B0390의 Gst/GstVideo import와 `nvarguscamerasrc`·`nvvidconv`·`videoconvert`·`appsink` factory 존재를 확인했다. 영상 취득은 하지 않았다.

## 실제 설정 전에 남은 것

실제 assembly ID·보드/PI/duty·서보 영점/방향/장착 한계·encoder 방향·runtime port·clock policy·motor-feedback JSON은 **미확정/null**이다. r401 `handover/runtime_profile.template.yaml`은 `legacy`, max1.5/min.2와 비장착 frame 등을 가진 참고본이므로 **실물 활성화 금지**다. 과거 합성 profile이나 시각 정책을 채택하지 않았다.

실제 운용에는 검토된 YAML에서 b3·manual/autonomous 선택·지상 초기.05/상한.1·실장 frame/가림·`use_sim_time=false`를 명시하고, 설치 profile/limits를 검증해 planner·독립 guard·host·패널에 같은 선택을 연결해야 한다. host의 `--clock-policy`·포트와 drive-wire 허용은 별도 명시이며 자동 ARM은 없다. 이번 결과는 이 진입점들의 설치/import 가능 범위까지만 확인한다. [기존 r491 실장/시작/STOP 명령](../runtime_handover_r491_v1/README.md)을 따른다.

펌웨어 wrapper의 ESP-IDF는 `/home/jinhyeong/.cache/it-arena/idf-6.1-smoke/{esp-idf,tools,python-v3}`와 별도 Xtensa 경로를 사용하는 **WSL 구성**이다. Jetson 통상 경로에는 IDF가 없으며, 이번에 WSL을 조회하거나 합성 firmware를 반복 빌드하지 않았다. 실제 장착값이 정해진 뒤 기존 PC 빌드 담당에게 넘긴다.

## 자원 반환

빌드PID5366·최종 import검증PID6330 종료, 관련 ROS/runtime/benchmark/Gazebo/colcon 잔존0. 정상 종료이며 강제 종료는 없다. 센서 포트 소유권을 조회하거나 포트를 열지 않았다. [반환 영수증](remote_receipts/resource_return.json). 회수 ZIP SHA `f83e7bfe121d20260564ba9b2ee8d4658b4cc5cb2c63c56395c1fc81fc94cdc3`도 양쪽 일치한다. 이후 Jetson 실행 소유권은 주 담당에게 반환했다.
