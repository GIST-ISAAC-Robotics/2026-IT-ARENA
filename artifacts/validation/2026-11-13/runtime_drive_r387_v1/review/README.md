# r387 portable 결합 검토·검사 인계

2026-11-13, GPT. [독립 발견과 반영 상태](FINDINGS.md). **Windows 정적 준비만 수행했으며 아래 합성 C++ 검사/컴파일은 아직 실행하지 않았다.** 공통 소스는 수정하지 않았다.

`combined_cases.cpp`는 실제 `RuntimeBoardProfile`, `ActuationMotorEndpoint`, `RuntimeServo`, `RuntimeTx`, MD20A 매핑을 직접 include한다. 호스트 명령은 실제 IA frame+command envelope로 입력하고, 실제 Tx의 offer 프레임에서 lease를 받아 사용한다. 서보 sink는 송신 바이트를 모으고 ST 응답 형식의 합성 ACK를 주입한다. receiver 상태를 테스트 전용 setter로 만들거나 `/drive/safe`를 우회하지 않는다. 시간·count·servo 위치·교정 flag는 합성 입력이며 운영 프로필이 아니다.

11개 대표 사례:

1. `default`: 기본 무효 프로필에서 서보 송신 0·모터 duty 0·비허가.
2. `engagement`: ready offer→ARM→실제 DRIVE→hold/torque ACK 순서, 준비 중 PI 출력 0, 결합 직후 첫 PI 응답 .303으로 적분 누적 없음, DRIVE 속도 0에서도 조향.
3. `arm_no_center`: ARM과 torque 결합만으로 CAD 중앙 goal을 보내지 않음.
4. `stop_late_ack`: STOP과 torque ACK를 같은 주기에 처리해도 재결합/새 쓰기 없음.
5. `expired_late_ack`: 제어 tick·새 offer 없이 명령 만료 1 µs 뒤 torque ACK 앞에 check_deadlines가 FAULT를 반영하고 새 CAD goal을 막음.
6. `control_gap`: 새로운 encoder 입력과 반복 check_deadlines가 있어도 실제 control tick의 50 ms 공백을 숨기지 않음.
7. `negative_feedback`: 프로필 encoder 방향 환산 뒤 음수 속도에도 고정 전진 DIR·양수 duty.
8. `servo_fault_recovery`: 동작 중 서보 오류→ready false·duty 0·기존 receiver FAULT, 외부 drain 가정의 read-only 복구 후 자동 DRIVE 재개 없음, 명시 CLEAR/ARM 경로 유지.
9. `profile_gains`: ki=0/kp=0·미확인 encoder 방향의 프로필 거절.
10. `profile_geometry`: CAD 범위보다 큰 명령 한계·내륜 제한과 맞지 않는 명령 한계의 프로필 거절.
11. `quantization`: 현재 count 환산과 PWM 정수화의 수치 범위. 실측 PWM·제동·속도 리플 검증이 아님.

main의 host UART 오류 후 분기, ESP-IDF 드라이버, 독립 watchdog, 실제 LEDC 재시작/파형, GPIO/장착 서보 응답은 실행 대상이 아니다. 합성 step은 주요 호출 순서와 기존 portable 계약을 확인할 뿐 5 ms 실시간 성능을 입증하지 않는다. Python harness는 실행 전후 원본/검사 파일 해시를 확인한다. 검사에 사용하는 최종 main/profile/portable dependency 해시는 `manifest.json`에 보존한다.

Windows에서는 Python AST 구문·관련 파일/해시를 정적으로 확인했다. C++ 검사는 지정 실행 담당자가 저장소 루트에서 새 출력 경로로 한 번 수행한다.

```sh
set -eu
A=artifacts/validation/2026-11-13/runtime_drive_r387_v1/review
test ! -e "$A/verify_v1"
mkdir "$A/verify_v1"
result=0
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q "$A/test_combined.py" \
  --basetemp "$A/verify_v1/tmp" -o "cache_dir=$A/verify_v1/cache" \
  >"$A/verify_v1/pytest.log" 2>&1 || result=$?
cat "$A/verify_v1/pytest.log"
exit "$result"
```

실패·원본 해시 차이가 있으면 해당 결과를 보존하고 통합 판정을 보류한다. r384의 27개 개별 순서기 검사는 여기에서 다시 복제하지 않았다.
