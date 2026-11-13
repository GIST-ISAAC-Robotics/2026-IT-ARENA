# r495 기존 encoder 속도 경계의 선택형 실장 프로필 연결

**공통 이관 후속:** 주 담당이 변경2파일을 검토해 [공통에 이관](integration_v1/applied.json)했고, 실제 공통 header를 사용한 [PC C++ compile/run 계약](integration_v1/native_receipt.json)이 exit0으로 통과했다. 준비된 C++ fixture의 미지정 조향 상한0 때문에 검사 목적과 관계없이 invalid가 되는 부분은 실행 전에 합성 `.2rad`를 명시했다. 수정된 CPP SHA는 native 영수증에 보존했다. 앞선 receipt/Windows 결과는 원본으로 유지한다. 실제 board/영점/encoder 속도 상한은 아직 선택하지 않았으며 firmware build·설치·실기 출력은 하지 않았다.

2026-11-14. 독립 Windows stage 준비 당시의 기록이며 실물 값은 미설정이다. 새로운 속도제어/보호 상태기계를 만들지 않고, 기존 `encoder_implausible` 경계로 `max_wheel_mps`를 전달하는 2파일 변경만 준비했다. [내용 diff](candidate_content.diff), [원본/stage SHA](changes.json), [검사 영수증](receipt.json).

## 변경

- `scripts/runtime_installed_profile.py`: `runtime_installed_board_v1`에 선택형 `max_wheel_mps` 하나를 허용한다. 기존 필수 필드/오탈자 거절은 유지한다. 명시했을 때만 유한 양수이고 `maximum_speed_mps` 이상인지 확인한 뒤 `p.max_wheel_mps=...`를 내보낸다. 입력 JSON 원문을 보존하며 값을 정규화하거나 13km/h로 자동 선택하지 않는다.
- `src/arena_vehicle_interface/firmware_runtime_drive/main/board_profile.hpp`: `max_wheel_mps` 기본값은 기존 receiver와 같은 **5.0 m/s**다. 유효 실장 profile의 `receiver_config()`에서 그 값을 기존 `ReceiverConfig.max_wheel_mps`에 전달한다. 명령 상한 `maximum_speed_mps`, PI, duty, feedback 주기, STOP/FAULT, 미교정 비활성 분기는 변경하지 않는다.

필드를 생략한 기존 board JSON은 이전과 같은 header bytes를 생성한다. 생성기의 source SHA는 바뀌므로 이전에 생성한 profile bundle의 원 source manifest를 덮어쓰지 않는다. 새 코드로 빌드하려면 보존된 동일 실장 입력으로 새 bundle을 생성/검증하는 기존 흐름을 따른다. 이전 profile을 새 실행으로 소급하지 않는다.

## 검사

`python -B artifacts/validation/2026-11-14/installed_feedback_limit_r495_v1/test_contract.py` → **4개 통과, exit0** ([로그](tests_v1.log)).

1. 필드 생략 시 공통 기존 generator와 stage 생성 header 전체가 동일하고, C++ 기본5.0/전달문이 존재함을 확인했다.
2. 합성 계약값 `13/3.6`의 정확한 header 전달·입력 보존·manifest 왕복·출력/flash/CPP검증 flag false를 확인했다. 이는 운용값 채택이 아니다.
3. 0·음수·bool·null·문자열·NaN·Inf·명령 상한보다 작은 값을 파일 생성 전에 거절했다.
4. 필드 오탈자와 생성 후 값 변경을 계속 거절했다.

fixture는 기존 `tests/test_runtime_installed_profile.py`의 명시 합성 입력 함수만 재사용했다. 합성 파일은 임시 폴더에서 정리됐고 실제 영점 파일로 배포하지 않는다. stage의 변경2파일과 import용 무변경3파일은 모두 원 공통 SHA가 유지됨을 확인했다.

현재 Windows PATH에서 g++/clang++/cl은 발견하지 못했다. WSL 사용 금지 범위를 유지했으므로 **C++ 컴파일·firmware build·receiver 실행은 이번에 하지 않았다.** [receiver_profile_contract.cpp](receiver_profile_contract.cpp)는 주 담당의 후속 PC 계약 검사에 사용할 수 있게 준비만 했다. 이 파일은 기본5.0·명시값 전달·명령 상한 유지·불일치/NaN 거절을 검사하며 장치 I/O가 없다. Python의 header 검사와 C++ 실행 증거를 혼동하지 않는다. 필요한 경우 native 계약 검사는 r491 bundle 생성/실기와 별도로 배정하면 된다.

## 기존 차단의 의미와 남은 실물 한계

변경하지 않은 [actuation_receiver.hpp](../../../../src/arena_vehicle_interface/portable/actuation_receiver.hpp) 370–374행은 모터 count 차분으로 계산한 절대 속도가 `max_wheel_mps`를 **초과**하면 feedback를 무효화하고, ARMED 중이면 `encoder_implausible` FAULT로 전이한다. 같은 파일 260–273행의 목표0/세대·lease 폐기 뒤 [main.cpp](../../../../src/arena_vehicle_interface/firmware_runtime_drive/main/main.cpp) 117–119행의 unarmed 분기에서 PWM0이 된다. 비ARM 상태에서는 invalid feedback 반환이다. 기존 지상/자율 기본값을 이번에 낮추지 않았다.

이 경계는 count 양자화와 feedback 샘플 구간으로 얻은 속도에 대한 반응이다. 검출 전 가속, 검출/출력 지연, PWM0 이후 관성 때문에 **실측 과도까지 포함한 13km/h 절대 상한을 소프트웨어만으로 보증하지 못한다.** `13/3.6`을 경계로 넣는 것 자체가 사용자 요구의 물리 상한을 충족했다는 증거가 아니다. 실제 공중 시험의 명령 단계/여유/설치 선택/차단 및 overshoot 확인이 아직 필요하며, 이번 준비는 고속시험 개시를 뜻하지 않는다. 새 motor 기능·감속 방식·상수 임의 교체를 덧붙이지 않았다.

검사는 Windows 전경 프로세스로 종료했고 남은 소유 PID/장치 자원은 없다. r491 bundle은 주 담당의 r492 회수·공통 이관·생성 지시를 기다리는 상태로 돌아간다. r495를 채택할 경우 r491 prepare의 재귀 수집은 이 generator/header를 자동 포함하지만, 채택/실장값 설정 여부는 주 담당이 별도로 기록한다.
