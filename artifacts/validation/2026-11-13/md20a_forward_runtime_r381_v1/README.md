# MD20A 전진 duty 연결 준비 r381

2026-11-13, GPT. **staged C++/Python 검사29개를 통과한 뒤 5개 파일을 공통 소스로 이관했다.** 기본 출력2,015입력 정확 대조·기존25주기·전진 duty·역회전·정상 과속 감속·STOP/만료를 포함한다([실행 로그](verify_v1/pytest.log)). 이관본과 staged의 바이트 일치, 원4개 파일이 보존 ZIP의 해시와 같음을 [이관 영수증](integration.json)에 남겼다. 기존 `firmware_runtime_preview`는 여전히 출력 비활성이다. ESP32 빌드/flash·실물 출력은 이번 결과에 포함하지 않는다.

Windows 정적 확인: Python AST 구문 검사, 원본 4개 해시 불변, `git apply --check proposed.patch`, 공백 오류 검사를 통과했다. 후보 코드를 import하거나 제어 계산/합성 테스트를 실행한 결과는 아니다.

## 변경과 출력 의미

- [`speed_pi.hpp`](staged/src/arena_vehicle_interface/portable/speed_pi.hpp): 생성자 끝에 `output_min=-1.0, output_max=1.0`을 추가했다. 유한한 `-1 <= min <= 0 <= max <= 1`, `min < max`만 허용한다. 출력과 적분을 같은 범위로 제한하고 기존 조건부 적분의 포화 비교에 해당 경계를 사용한다. 기본 범위에서는 기존 산술 순서·상태·`brake_requested`가 동일하다. 게인은 변경하지 않았다.
- [`motor_control_candidate.hpp`](staged/src/arena_vehicle_interface/portable/motor_control_candidate.hpp)와 [`actuation_motor_endpoint.hpp`](staged/src/arena_vehicle_interface/portable/actuation_motor_endpoint.hpp): 기존 위치 인자를 유지하고 끝의 선택 한계를 PI까지 전달한다. receiver의 ARM/DRIVE/STOP/CLEAR·만료·피드백 검증·PI 초기화는 바꾸지 않았다.
- [`md20a_forward_output.hpp`](staged/src/arena_vehicle_interface/portable/md20a_forward_output.hpp): 같은 소유 작업에서 `control_tick` 직후 쓰는 순수 매핑이다. **DIR는 명시 장착 프로필의 전진 논리값으로 고정**하고, 수신기가 검증한 ACTIVE/owner/양수 목표 및 같은 세대·주기의 `PiStatus::kNormal`에서만 duty를 반환한다. 음수/비유한/프로필 상한 초과·모순된 intent는 duty 0과 별도 매핑 상태로 반환하며 DIR를 뒤집거나 receiver를 변경하지 않는다. `output_ready=true` 또는 `brake_requested` 하나만으로 구동을 허가하지 않는다. 준비값 기본은 false다. GPIO/PWM 해상도 변환·UART 송신·HAL은 포함하지 않는다.
- [`mcu_speed_loop.py`](staged/src/arena_vehicle_interface/arena_vehicle_interface/mcu_speed_loop.py): 같은 선택 한계와 수식을 Python 참조에 추가했다. 기존 dataclass/API와 기본 토크 모델을 유지한다.

실차 선택은 `SpeedPI(..., 0.0, profile.maximum_duty)`이다. 음수 바퀴 속도는 부호가 교정된 실제 역회전 피드백일 수 있으므로 양수 전진 duty 응답을 허용한다. 양수 목표에서 overspeed이면 정상 `kNormal`/duty 0을 반환하고 수신기 ACTIVE를 유지한다. 매핑 결과 `kActive`의 duty 0과 `kInhibited`의 duty 0은 물리 PWM 값은 같지만 원인/재개 계약이 다르다. 전자는 다음 정상 PI 주기에 양수 출력으로 돌아갈 수 있고, STOP/FAULT는 기존 CLEAR→ARM→새 DRIVE 없이 재출발하지 않는다. 목표 0은 기존 ARMED와 적분 초기화를 유지한다.

**MD20A의 PWM 0은 단락 제동이다.** signed torque 모델의 음수 effort와 동치가 아니며, coast·제어된 역토크·일정 감속률을 뜻하지 않는다. `brake_requested`의 기존 음수 effort 의미를 DIR 반전이나 STOP으로 옮기지 않는다. 이 변경은 전진 duty 제어의 계산 경로를 준비할 뿐 기존 가상 토크 게인이 실차 duty 게인으로 적절하다는 증명이 아니다. [기존 단독 모터 실측](../../../../docs/sensors/MD20A_MOTOR_HARDWARE_2026_11_12.md)은 DIR LOW·약 5% 구간의 제한 회전/정지이며 장착 전진 방향·운영 상한·장착 감속 성능 교정값으로 승격하지 않는다.

## 통합 시 명시할 입력

장착 프로필에서 확인한 `maximum_duty`와 `forward_dir_high`를 제공해야 한다. 운영 기본값은 추가하지 않았다. 시험의 `.04/.4/1.0` 상한, 기존 테스트의 15:1 환산·고정 boot ID는 **합성 검사값**이다. 복사해 운영 프로필로 사용하지 않는다. 엔코더 카운트/회전·전진 부호·실제 감속·정지 시간/거리, 부하에서의 허용 duty는 실측 없이 확정하지 않는다.

```cpp
// 아래 변수들은 장착 프로필과 기존 런타임에서 전달한다. 이 후보에는 수치 기본값이 없다.
const arena_actuation::Md20aForwardProfile profile{mounted_max_duty, mounted_forward_dir_high};
arena_actuation::ActuationMotorEndpoint endpoint(
    receiver_config, boot_id, boot_length, assembly_timeout_us, layout, tx_config,
    configured_kp, configured_ki, max_dt_s, 0.0, profile.maximum_duty);
const auto intent = endpoint.control_tick(now_us, actual_output_ready);
const auto command = arena_actuation::md20a_forward_command(
    intent, endpoint.receiver(), profile, actual_output_ready);
// 실제 출력 적용·준비 증거·GPIO 초기화/실패 처리·flash는 별도 HAL 통합 범위다.
```

`command`는 계산 결과이며 출력 적용 영수증이 아니다. preview/ready를 보관해 다음 주기에 재사용하지 않는다. 매핑은 새 watchdog이나 상태기계가 아니며 기존 receiver의 판정을 소비한다. 현재 `output_ready=false`를 단순히 true로 바꾸는 작업은 이 패치에 없다.

## 고정 검사 묶음

[`tests/test_forward_runtime.py`](tests/test_forward_runtime.py)는 공통 파일을 수정하지 않고 staged include 경로로만 후보를 빌드한다. `g++` 부재는 skip 대신 실패한다. 검사는 아래 범위다.

1. 기존 C++ 드라이버를 원본/후보 기본값으로 각각 빌드하고 같은 2,015개 입력의 effort·적분·brake·valid·status 출력 원문을 정확 비교한다. 명시 `[-1,1]` 출력 및 원본/후보 Python도 binary64 값으로 대조한다.
2. 네 범위와 네 dt에서 정방향·역회전·overspeed·0 목표·비허가를 대조한다. 상한 포화의 적분 누적 방지와 해제, 하한 포화 뒤 양수 응답, 비유한/잘못된 입력 및 범위의 초기화를 검사한다.
3. 기존 25개 receiver/주기 사례를 staged 기본값으로 실행한다. 전진 전용 주기에서는 고정 DIR 양쪽 값, 정상 duty 0의 ACTIVE 유지와 회복, 목표 0, STOP/물리 STOP/출력 비허가/명령 만료 뒤 자동 재출발 금지, CLEAR만으로 출발하지 않음, 기존 ARM+DRIVE 복구를 검사한다. 매핑에 bool만 전달하거나 상태/세대/주기가 다른 결과·음수 effort·모순된 intent를 넣어 구동할 수 없는지 확인한다. endpoint의 선택 인자 전달도 검사한다.

원본4개 파일의 사전 해시는 [`tests/source_manifest.json`](tests/source_manifest.json)에 있으며 staged 실행 전후 동일성을 확인했다. [`proposed.patch`](proposed.patch)는 그 원본에 적용한 패치다. 원문은 `integration.json`이 가리키는 r377 `sources.zip`에 보존되어 있다. 준비 파일 해시는 [`tests/prepared_manifest.json`](tests/prepared_manifest.json)에 별도 보존한다. 공통 이관 뒤 원 비교 스크립트를 그대로 실행하면 사전 해시 검사에서 거절되는 것이 정상이며, 재현에는 보존된 원본 소스를 사용한다.

실행 자원을 배정받은 담당자가 저장소 루트에서 **한 번** 실행한다. 새 결과 경로만 허용하며 실패 시 로그를 남긴다.

```sh
set -eu
A=artifacts/validation/2026-11-13/md20a_forward_runtime_r381_v1
test ! -e "$A/verify_v1"
mkdir "$A/verify_v1"
result=0
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q "$A/tests/test_forward_runtime.py" \
  --basetemp "$A/verify_v1/tmp" -o "cache_dir=$A/verify_v1/cache" \
  >"$A/verify_v1/pytest.log" 2>&1 || result=$?
cat "$A/verify_v1/pytest.log"
exit "$result"
```

기본 출력 차이, 신규 범위의 수치 불일치, 정상 감속의 FAULT/STOP 전이, DIR 반전, 허가 없는 양수 duty 또는 원본 해시 변경이 한 건이라도 있으면 통합을 중단한다. 통과 후 공통 패치/HAL 빌드 여부는 주 작업 담당자가 검토한다. 실차 게인·제동 성능이나 차량 전체 안전을 합성 검사 통과로 판단하지 않는다.
