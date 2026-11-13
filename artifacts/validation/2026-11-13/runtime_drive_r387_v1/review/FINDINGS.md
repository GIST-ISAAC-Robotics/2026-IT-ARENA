# r387 운영 연결 독립 검토

2026-11-13, GPT. 공통 소스·장치·WSL을 수정/실행하지 않고 관련 호출 경로를 읽었다. 첫 검토와 수정 후 검토를 구분한다.

첫 검토 SHA256: main `3c8534d08d5ffed0a60c6b17fb4a1a768055a3c7161fea63404218eba1011a87`, board profile `b494c2aff18dd8ec4bd8da552e01bb12ea0b1039dbe645fb6bae4652c9599776`.

## 발견과 반영 상태

1. **P1 — 만료 검사보다 서보 ACK 처리가 앞설 수 있음.** 첫 main은 `endpoint.poll(now,ready)`가 만료를 검사한다고 전제했지만 `ActuationIngress.poll`은 프레임 조립만, `sync→tx.observe`는 상태 조회만 했다. 새 수신/offer가 없는 제어 tick 사이에 deadline이 지나면 옛 armed snapshot과 늦은 hold/torque ACK가 다음 쓰기를 시작할 수 있었다. 주 작업 담당자가 heartbeat를 갱신하지 않는 `receiver/endpoint.check_deadlines`를 추가하고 host poll 앞·servo ACK 앞에 연결했다. **수정 코드 확인, 실행 검증 대기.** 합성 수용 조건은 tick 사이 만료+torque ACK에서 `command_expired`와 신규 쓰기 0, 반복 check_deadlines가 control gap 고장을 숨기지 않는 것이다.
2. **P1 — host UART 읽기 실패 뒤 같은 블록에서 서보 송신 가능.** 첫 main은 host `uart_read_bytes<0`으로 `failed=true`를 설정한 뒤에도 servo 읽기/step으로 진행했다. 주 작업 담당자가 추가 `!failed` 관문으로 분리했다. **수정 코드 확인.** HAL 오류 반환 자체는 portable 결합 검사가 검증하지 않는다.
3. **P1 — 프로필의 ki=0 승인과 PI 구성 불일치.** 첫 `RuntimeBoardProfile.valid()`는 `ki>=0`, 실제 `SpeedPI`는 `ki>0`이었다. configured=true인데 첫 제어 tick이 `control_invalid_config`가 될 수 있었다. 주 작업 담당자가 프로필도 `ki>0`으로 맞췄다. **수정 코드 확인, 합성 검증 대기.** 새 P-only 모드/게인 변경은 없다.
4. **P1 — CAD 범위를 넘는 command_limit을 프로필이 승인.** 첫 `valid()`가 호출한 `ResolveCadServoGoal(..., steering=0, command_limit, ...)`은 내부 양끝 `map.central(±limit)`의 `valid`만 보고 `clamped`를 확인하지 않았다. 예컨대 넓은 ZeroProfile과 command_limit=.8 rad는 CAD 범위를 넘는데도 승인될 수 있고 receiver도 ±.8을 수락했다. 실제 목표는 CAD 끝점으로 조용히 제한되어 명령 계약보다 작게 실행될 수 있었다. 주 작업 담당자가 운영 프로필 양끝 central 결과에 `valid && !clamped`를 추가했다. **수정 코드 확인, 합성 검증 대기.** 기존 CAD 함수 전체 정책은 바꾸지 않았다.

수정 후 검토 SHA256: main `346240eacdd9d9e37574b8a94ad0ffa76789d4dcd59f4b5aaeceaae074c5d060`, board profile `b75384fe3607ce41b867d77cd797c622fbb4aa18c97e36ca39072e28c72b3406`. 첫 보드 build_v1과 현재 프로필 수정본의 빌드 증거는 주 작업 담당자가 별도로 구분한다.

## 유지할 연결과 실측 경계

- ready/engaged 분리, 준비 중 PI reset, receiver.drive_seen을 통한 DRIVE 0 조향은 연결돼 있다. ARM만으로 중앙 목표가 발생하지 않음, 음수 속도 피드백의 전진 duty, 동작 중 서보 실패/복구 뒤 기존 CLEAR→ARM 계약을 대표 결합 검사로 확인한다.
- PWM은 `floor(request*1023)` 정수다. 10 bit 하드웨어의 이상적인 duty 비율은 해당 정수/1024이며 요청보다 작게 양자화된다. 작은 양수 요청은 정수 0이 될 수 있다. `duty_request` 진단은 실측 PWM·실제 모터 토크가 아니다.
- 현재 코드 환산의 28 count/rev·11.9118:1·65 mm 바퀴·20 ms에서 1 count는 약 0.0306 m/s로 정지 기준 0.03 m/s와 가깝다. 실물 count scale·장착·잡음은 미확정이므로 이 계산만으로 정지 기준/피드백 주기/게인을 바꾸지 않는다.
- 독립 watchdog, 실제 LEDC/PWM·DIR, 장착 서보 hold/속도/응답·전류, 실차 감속은 검토·합성 검사의 증명 범위 밖이다. 출력 비활성 기본 프로필의 빌드와 flash/구동 허가는 별개다.

## 검사 범위

`combined_cases.cpp`와 `test_combined.py`는 실제 board profile/Endpoint/RuntimeServo/RuntimeTx/MD20A portable 코드를 직접 사용하고 IA/UART 요청·응답을 합성한다. main 자체나 ESP-IDF HAL은 실행하지 않는다. 최종 입력 해시·실행 명령은 README와 manifest에 남긴다.
