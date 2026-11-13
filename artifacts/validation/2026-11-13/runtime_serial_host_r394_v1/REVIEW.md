# r394 core 독립 검토

2026-11-13. `staged/scripts/runtime_actuation_link.py`와 공통 r391 session, r392 adapter, `serial_ingress`, MCU receiver의 관련 경로를 읽었다. 코드 수정·새 테스트 실행·ROS/serial 접속은 하지 않았다.

**P1 실제 결함은 확인하지 못했다.** 다음 판단은 session/link의 의사결정을 한 소유자가 순서대로 수행한다는 현재 계약을 전제로 한다. I/O thread와 공유하는 mailbox만 lock으로 보호된다.

- **폐기된 seq:** `prepare_drive`에서 번호/원천 watermark를 소비한 뒤 mailbox가 해당 DRIVE를 교체해도 문제가 되지 않는다. MCU `actuation_receiver.hpp:185`는 `seq > accepted_seq`만 요구하며 연속 번호를 요구하지 않는다. 원천 재사용은 계속 금지되고, 폐기된 프레임의 시각이나 lease를 새 프레임으로 바꿔 넣지 않는다.
- **STOP 선점:** session `begin_control('STOP')`은 기존 ARM 근거와 pending을 즉시 폐기한다. mailbox는 대기 control/DRIVE를 제거하고 STOP을 먼저 꺼낸다. STOP pending 중 DRIVE 삽입도 거절한다. STOP을 꺼낸 뒤 mailbox의 표식이 사라져도 정상 session은 `stop_requested` 및 ARM barrier 부재로 DRIVE를 만들지 않는다. MCU의 STOP 세대 변경으로 이전 세대 프레임도 다시 허가되지 않는다. CLEAR/ARM 자동 복구 경로는 없다.
- **부분 wire/송신 중 프레임:** mailbox는 완전한 canonical 프레임 1개만 받고, 기존 I/O는 short write에서 오류로 종료하며 잔여 바이트를 재송신하지 않는다. I/O가 이미 꺼내 송신하는 프레임 한 개는 STOP으로 취소할 수 없다. `write_timeout=0.2s` 및 MCU의 기존 만료 처리는 유지되며, 이 구조를 즉시 정지 보장으로 해석하지 않는다.
- **disabled/출력 허가:** `drive_enabled=False`는 ARM과 DRIVE 생성을 막고 명시 STOP은 허용한다. CLEAR/DISARM은 출력 시작이 아니다. `command_created`/`wire`는 준비 결과이며, 상속 결과의 `ready_to_drive=False`와 ACK/offer를 물리 출력 허가나 정지 완료로 승격시키지 않는다.
- **queue 지연/r391 결합:** wire는 age를 계산한 같은 lease로 고정된다. 대기열 지연 후 송신되어도 MCU deadline은 늘어나지 않는다. 늦은 도착의 거절 가능성은 남지만, 최신 DRIVE 교체 때문에 오래된 입력의 수명이 연장되는 경로는 확인하지 못했다.

## 후속 runner 연결에서 지켜야 할 기존 경계

`SerialIngress.send(STOP)`는 **enqueue**이며, `SerialIngress.close()`는 `stopping`을 켜고 drain 없이 종료한다. 따라서 `send(); close()`를 연속 호출하면 아직 쓰지 않은 STOP이 남을 수 있다. 예정된 “clock fault/session closed → STOP 전송 후 종료”는 **그 STOP의 tx receipt 또는 명시한 제한시간 종료**를 구분해야 한다. tx receipt도 MCU ACK/물리 정지 증거는 아니다. 현재 core에는 runner의 종료 순서가 아직 없어 이를 확인된 P1로 분류하지 않았다.

또한 `drive()`에서 session이 닫히는 경우 `link.closed` 반영은 이후 `poll()`에서 이루어진다. 부모가 명시한 대로 runner는 `session.closed`도 확인해야 한다. 이미 예정된 조건이므로 새 보호층이나 별도 정책 변경을 제안하지 않는다. 최초 STOP·자동 ARM 없음, `/drive/safe` 구독, 원 receipt 시각 보존은 후속 runner의 검토 대상이다.

## 검토한 판본

- staged `runtime_actuation_link.py`: `c84e6d77338f0d61b8c18ef9e32579cddb2b88d9f63d2357e9af7e7975336611`
- common `receiver_owned_actuation_host.py`: `005244d8918d4741c449a8f2f717a674c39cea643b360e52b751897777a6023f`
- common `actuation_host_session.py`: `47c89b846d8b0dbcc50eb39ec207fa9253804a2abe0749474edff89ff9602a4c`
- common `serial_ingress.py`: `f7837cc58877c1f0f456df5a9f6d0d1ea38490f3777e657273d51bebf61108de`

기존 `verify_core_v1/pytest.log`의 88 passed / 1 skipped는 저장된 실행 결과이며 이번 검토의 신규 실행 결과가 아니다. 실물 구동/정지·시계 속도오차·진행 중 write 취소를 검증한 것으로 확대하지 않는다.
