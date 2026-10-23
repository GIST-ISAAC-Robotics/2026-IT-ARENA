# MCU 이관용 보드 독립 구동 계약 코어

`actuation_wire.hpp`는 기존 `IA + uint16 length + payload + CRC32`의 **바이트 경계만** C++로 옮긴 준비물이다. ROS 런타임·실제 MCU에는 연결하지 않았다.

- payload 1~1536 B, CRC/헤더 포함 최대 1544 B. 코어 자체는 고정 배열을 사용하며 동적 메모리를 할당하지 않는다.
- `feed` 한 번에 최대 256 B. 상위 수신 작업이 작은 청크로 나눠 공급해야 한다. 초과/잘못된 포인터는 남은 부분을 폐기하고 `false`를 반환한다. 이 제한은 기존 Python의 64 KiB 입력 제한과 다르며 명시적으로 선택한 코어 API 한도다.
- callback은 동기 호출이고 포인터 수명은 그 호출 동안뿐이다. 포인터 보관·decoder 재진입을 금지하며, callback의 작업량/예외/큐 한도는 별도로 관리해야 한다. 코어의 고정 메모리가 전체 펌웨어의 처리 시간 보장을 뜻하지 않는다.
- 반환 payload는 **CRC만 통과한 원문**이다. 엄격 JSON과 명령 필드 표현은 아래 `actuation_command.hpp`가 맡고, 세대/임대/소유권/STOP 상태 판정은 `actuation_receiver.hpp` 후보가 맡는다. 어느 단계도 직접 모터 출력에 연결하면 안 된다.
- `reset`은 부분 바이트만 폐기하며 구동 상태를 변경하지 않는다. `reject_head`는 상위 정책이 머리 후보를 손상으로 판정할 때 CRC 실패와 같은 규칙으로 2 B를 버리고 다시 검사한다.
- CRC는 인증이 아니다. 송신 payload와 encode 출력 버퍼는 겹치지 않아야 한다.

## 10/22 링크 후보 — 절대 조립 기한과 연결/오류 정책

`actuation_link.hpp`의 `FrameLink`는 위 decoder를 감싸는 **PC 검증용 후보**다. 운영 기본값·ROS 경로·Python 런타임에는 연결하지 않았다.

- 시각: 호출자가 주는 단조 `uint64` tick. 같은 값은 허용하고, 직전 값보다 작으면(32비트 wrap 포함) `kClockRegressed`로 고정한다. 모듈러 산술은 하지 않는다.
- 기한: 부분 버퍼가 생긴 시각을 anchor로 두고 `now - anchor >= timeout`이면 만료한다. `anchor + timeout` 시각에 도착한 바이트는 이전 조각에 붙지 않는다. 바이트를 조금씩 보내거나 빈 입력을 넣어도 anchor는 늦춰지지 않는다. 이전 호출의 바이트가 모두 소비된 경우에만 anchor를 그 호출 시각으로 갱신한다. 그 밖의 재동기화는 이전 anchor를 유지하므로 실제 시작보다 이른(보수적) 기한이 될 수 있다.
- 만료 처리: 기한이 지난 머리를 손상으로 보고 남은 바이트를 CRC 실패 규칙으로 끝까지 재검사한다. **이미 완성돼 있던 CRC-valid 프레임만** `FrameOrigin::kExpiredDrain`으로 전달하고 버퍼를 비운다. 따라서 합법 길이 손상 머리 뒤에 숨은 STOP도 기한에 드러난다. 늦게 전달된 프레임의 수명은 수신 측이 원래 임대/원천 시각으로 판단해야 한다.
- `poll(now)`: 무수신 상태에서 독립 작업이 기한을 확인한다. 실제 폐기 지연은 호출 주기에 의존한다.
- 연결/오류: `disconnect`·연결 중 `connect`(놓친 단절)·고장 고정은 부분 바이트를 **전달 없이** 폐기하고 `session_breaks`를 증가시킨다. 단절 중 입력은 거절한다. 호출당 256 B 초과·null 포인터는 `kInputBudget`, 시계 역행은 `kClockRegressed`로 고정되며 그 호출의 바이트는 하나도 처리하지 않는다. `clear_fault(now)`는 시계 기준을 다시 잡고 `kDisconnected`로만 돌아간다. timeout 0은 영구 구성 오류다.
- 이 객체는 구동 허가를 만들거나 해제하지 않는다. 참조 정책([`actuation_link_reference.py`](../../../scripts/actuation_link_reference.py))은 `session_breaks` 증가를 세션 경계로 보고 로컬 STOP 고정 → 새 offer 기반 CLEAR → 별도 ARM만 재구동으로 인정한다. 조립 만료·CRC/길이 오류는 구동 상태를 바꾸지 않으며, 명령 만료·제어 tick watchdog·물리 STOP은 링크와 독립이다.
- 시험에 쓴 timeout(링크 단위 1~1000 tick·`UINT64_MAX`, 정책 사례 30 ms)은 **시험값**이다. 실제 baud·최대 프레임 직렬화 시간·USB/스케줄 지연 실측 전 운영값을 정하지 않는다.
- 만료 drain의 작업량은 기존 CRC 실패 재동기화와 같은 차수로 버퍼 크기에 묶이지만 측정한 최악 실행시간이 아니다. callback 실행 비용은 제한하지 않는다.

## 10/22 명령 표현 후보 — 엄격 JSON과 명령 필드

`actuation_command.hpp`의 `CommandParser`는 CRC 다음 단계의 **PC 검증용 후보**다. 기존 Python `Decoder`의 JSON 판정과 `ActuationReceiver.receive`가 읽는 필드의 타입·값을 고정 메모리 `Command`로 표현한다. 운영 Python·ROS·MCU에는 연결하지 않았다.

- 배치: `kBareCommand`(포팅 벡터의 명령 객체)와 `kEnvelope`(현행 ROS 가상 MCU의 `{"type":"command","request":…,"command":{…}}`).
- 거절(`ParseStatus`): 크기, 엄격 UTF-8(overlong·surrogate·U+10FFFF 초과·잘림·BOM), JSON 문법(제어문자·잘못된 escape·선행 0·잔여 데이터·후행 쉼표 등), `NaN/Infinity/-Infinity`, 중복 키(escape 해석·surrogate 결합 뒤 비교, 중첩 객체 포함), 최상위 비객체. 오류 종류는 Python 판정 순서를 따른다: UTF-8 먼저, 상수는 만나는 즉시, 중복 키는 그 객체가 닫힐 때.
- 표현: 필드마다 `absent/null/bool/int(int64 정확)/bigint(부호만)/float(IEEE binary64)/string(코드포인트 수)/array/object`. bool은 int가 아니다. 정수는 double을 거치지 않는다. float은 `std::from_chars`, 범위 밖은 Python처럼 ±inf/±0.0. `boot`·`client`·envelope `request`는 64 코드포인트까지 정확히 저장(짝 없는 surrogate 포함)하고, 더 길면 길이만 둔다. 알 수 없는 키와 kind 이름을 따로 둔다.
- 내용 분기 함수: `classify`(invalid_message/STOP/schema/ARM/CLEAR/DISARM/DRIVE, envelope 무시), `session_shape_ok`, `drive_values_ok(limits)`. **잘못된 DRIVE 값은 걸러내지 않고 `kDrive` + `drive_values_ok=false`로 넘긴다.** 상태기계는 세션·임대·소유권·순번을 기존 순서로 판정한 뒤에야 `invalid_drive` 고장을 고정해야 한다. 한도는 유한하고 2^63 미만이어야 한다(아니면 `limits_supported=false`).
- 메모리: x86-64 호스트 빌드에서 `sizeof(CommandParser)` 3,848 B(키 스택 769개, 깊이 1536 비트스택 2개), `sizeof(Command)` 1,144 B. MCU 크기는 툴체인·정렬에 따라 다르다. 매 호출마다 `Command`를 초기화하며 실패 결과에는 상태값만 남는다. 결과는 payload 포인터를 갖지 않아 프레임 callback 밖에서도 유효하다.
- 작업량: 1차 선형 스캔 + 같은 객체 안 키 쌍 비교 + 관심 필드 변환으로 payload 1536 B에 묶이지만 측정한 최악 실행시간이 아니다. `from_chars`는 PC libstdc++ 13.3에서 Python과 비트 일치를 확인했을 뿐이다. MCU 툴체인의 정확 반올림과, PC·MCU 모두에서 라이브러리 내부 할당 여부(allocator 계측 안 함)는 미확인이다.

## 10/22 수신 상태기계 후보

`actuation_receiver.hpp`의 `ActuationReceiverCore`는 `ActuationReceiver`의 판정을 C++로 옮긴 **PC 검증용 후보**다. 운영 Python·ROS·MCU에는 연결하지 않았다.

- 순서: 시계 → 제어 tick 공백 → 피드백 신선도 → 명령 만료 → 모터 무진행(`_check`), receive는 내용 분기 → 세션 → 임대 → 준비/소유권/순번 → DRIVE 값(`invalid_drive` 고장) → 원천 만료. 제어 tick만 watchdog 시각을 갱신한다. 물리 STOP 해제·재연결·parser reset은 CLEAR/ARM을 만들지 않는다.
- 피드백: 모터축(28카운트·15:1·65 mm 기준 시험)과 legacy 좌우 모드. 모터축에서 좌우 속도는 유효한 측정값이 아니며 PC status 직렬화가 null로 표시한다. 숫자 getter를 좌우 측정값으로 사용하지 않는다. capture 시각 차분, 첫 표본·중복·역순·gap·implausible·settle·무진행을 옮겼다. 카운트 차분은 int64 양끝에서도 wrap 없이 크기+부호로 계산해 double로 한 번 반올림한다. 단일 축 평균 0은 차체 정지나 좌우 정지 보장이 아니다.
- 구성: `ReceiverConfig{}`는 과거 Python 참조의 legacy 기본값을 유지한다. 현행 차량용 어댑터는 모터축 모드·28카운트·기어비15·바퀴 반지름0.0325 m를 명시해야 한다.
- 고정 크기: 임대 32개(가장 오래된 것부터 퇴출, 만료분은 시계 갱신 때 정리), owner/boot 64 코드포인트. 단일 제어 소유자 계약: 한 작업(또는 잠금 아래)에서만 호출한다.
- Python과 다른 점: `now_us`는 0..`INT64_MAX-2×10^7`만 유효(밖이면 `invalid_clock`, Python은 2^63-1까지 허용). generation/임대 순번이 int64 끝에 닿으면 재사용하지 않고 `counter_exhausted` 고장으로 고정(Python은 임의 정밀로 계속). 진단 dict(`deadline_diagnostic`)는 옮기지 않았고 횟수만 둔다.
- 공개 API 경계: 생성자가 구성과 boot 포인터/길이/코드포인트를 검사하며 잘못된 경우 영구 `invalid_config` 고장으로 둔다. 공개 신선도 조회도 유효 시각 검사 뒤에만 차분한다. 세대 소진 뒤 CLEAR/offer는 실패하며 시험용 카운터 주입 함수는 `ARENA_ACTUATION_TESTING`에서만 공개한다. 실제 통합 빌드에는 이 매크로를 정의하지 않는다.

PC 대조 결과: [10/21 프레임 코어](../../../artifacts/validation/2026-10-21/actuation_wire_cpp_v1/README.md), [10/22 링크 후보](../../../artifacts/validation/2026-10-22/actuation_link_cpp_v1/README.md), [10/22 명령 표현 후보](../../../artifacts/validation/2026-10-22/actuation_command_cpp_v1/README.md), [10/22 수신 상태기계 후보](../../../artifacts/validation/2026-10-22/actuation_receiver_cpp_v1/README.md). 모든 결과는 `scripts/*_driver.cpp`의 string/vector/표준 I/O·JSON 출력을 PC 검사기에서만 쓴다. 실제 ESP32 크로스 컴파일·실행·최악 실행시간·전기 출력 검증은 아니다.

독립 검토에서 발견한 [수정 전 API 실패](../../../artifacts/validation/2026-10-22/actuation_receiver_api_v1/README.md)를 보존했다. 수정본의 [최종 v4](../../../artifacts/validation/2026-10-22/actuation_receiver_cpp_v4/README.md)는 전체 3,635회귀·API 10개·변이체 13/13을 통과했다. 실물 이관 순서와 미검증 경계는 아래 보고서 §15.4~15.5를 따른다.

계약·링크 예산·재개 순서는 [구동 계약 보고서](../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#15-1022-후속--수신-상태기계의-c-이관)를 따른다.

## 10/22 수신 연결부와 목표 빌드 후속

`actuation_ingress.hpp`의 `ActuationIngress`는 `FrameLink → CommandParser → ActuationReceiverCore`를 단일 소유자에서 연결한다. `receiver`는 외부가 소유하며 연결부보다 오래 살아야 한다. 같은 작업/잠금 밖에서 동시에 호출하거나 observer에서 재진입하지 않는다. 형식과 조립 기한은 명시 인수이며 운영 기본값을 새로 정하지 않는다.

- 링크 세션 경계/고장 뒤 기존 receiver가 고정 상태가 아니면 로컬 STOP을 넣는다. 부분 바이트 폐기·재연결은 CLEAR/ARM을 대신하지 않는다.
- frame의 live/expired-drain 출처와 JSON 판정·receive 결과·request를 동기 observer에 전달한다. 비객체 JSON/무관 envelope는 receiver로 보내지 않고, 잘못된 명령 객체는 receiver의 원래 판정 순서를 따른다.
- 음수·수신기 시각 상한 밖의 시각은 unsigned로 변환하지 않고 영구 연결부 고장으로 둔다. `clear_link_fault`는 링크만 해제하며 연결부 구성/유효 시각 오류나 receiver 정지를 해제하지 않는다.
- 수신/poll은 제어 tick을 갱신하지 않는다. 진단 observer/숫자 parser의 최악 실행시간·실제 UART 큐 동시성·출력 준비 감시는 별도다. 연결부만으로 구동을 허가하지 않는다.
- `feed`/`poll`의 마지막 `output_ready` 인수는 호출 시점의 독립 감시값이다. 생략하면 false이며 이전 호출의 true를 저장하지 않는다. raw kind가 `ARM`이고 준비되지 않으면 버전/세션 검사 전에 `output_not_ready`로 거절한다(실제 Python `VirtualMCU.on_tx` 순서). `IngressDelivery.output_gate_rejected=true`, `receiver_called=false`로 구분하며 요청 ID가 있으면 송신 후보가 거절 ACK를 보낸다. STOP/CLEAR는 막지 않고 준비값이 true로 바뀌어도 자동 ARM은 없다. 실행 중 출력 상실을 감지/차단하는 독립 감시기는 별도로 필요하다.

[52개 관련 호스트 회귀](../../../artifacts/validation/2026-10-22/actuation_ingress_v2/README.md)와 [ESP32-S3 SDK 링크](../../../artifacts/validation/2026-10-22/actuation_idf_smoke_v4/README.md)를 확인했다. 앞 절의 PC 단계 이후 **목표 컴파일/링크는 확인됐지만 대상 실행은 미검증**이며 실제 펌웨어/전기 STOP 완료가 아니다.

## 10/22 송신 정책 후보 — 구동 출력 없는 MCU→호스트 송신

`actuation_tx.hpp`의 `ActuationTxScheduler`는 receiver 상태를 **유한 크기 송신 프레임**으로 바꾸는 PC 검증용 후보다. 운영 Python/ROS(`actuation_ros.py`)와 기본값에는 연결하지 않았고, UART 드라이버·출력 허가·제어 task가 아니다.

- 역할: `offer`(호스트 명령 생성에 필요한 `type,v,boot,generation,state,owner,lease,lease_issued_us,lease_expires_us,output_ready`만), `ack`(요청 ID가 있는 명령의 판정, 기존 ack와 같은 바이트 형식), `state`(세대/고정 상태 변화, 최신 하나로 병합·`changes` 횟수), `diag`(상세 진단, 실수는 정수 micro 단위). 명령마다 offer를 보내지 않고 세대가 바뀔 때만 앞당긴다.
- 소유권: receiver와 같은 작업(또는 잠금)에서만 호출한다. offer 임대를 발행하므로 receiver를 비상수 참조로 받는다. `observe`는 조회만, `pump`만 `receiver.offer(now)`를 부른다.
- 저장: ack 8개(요청 64 코드포인트), 전송 중 프레임 1개(최대 1,544 B), 상태/offer/diag는 플래그와 시각만. x86-64 `sizeof` 4,296 B. 동적 할당 없음(코어 코드). pump 호출당 materialize 2개 이하.
- 시각: offer 임대는 프레임 내용을 고정하는 순간에만 발행한다. 고정한 프레임은 부분/0 B 쓰기에도 끝까지 그대로 보내며 새 상태로 바꾸지 않는다. 큐에 오래 있던 내용을 새 시각으로 다시 포장하지 않는다. `lease_issued_us/lease_expires_us`는 MCU 시계이며, 호스트 시계와 빼서 신선도를 판단하는 것은 PC fixture에서만 허용된 가정이다.
- 우선순위: ack(STOP 결과 먼저) > 세대 변화 offer > state > 주기 offer > diag. ack/state가 `max_consecutive_events`에 닿고 offer가 기다리면 offer를 먼저 보낸다. `line_ns_per_byte`를 주면 diag는 다음 offer 기한 전에 직렬화가 끝날 때만 시작한다.
- 포화: ack 큐가 차면 STOP 결과는 가장 오래된 비STOP 결과를 축출하고, 그 밖의 새 결과는 버린 뒤 계수한다(호스트 쪽은 timeout으로 "미확인"). STOP 자체는 수신 단계에서 이미 적용되며 ack 유실이 정지를 취소하지 않는다. 모든 사건 전달을 보장하지 않는다.
- 세션/고장: 링크 세션 경계에서 전송 중 꼬리와 ack 큐를 버리고, 링크가 끊긴 동안 쓰지 않는다. CLEAR/ARM을 만들지 않는다. escape가 필요한 boot·잘못된 정책값·시계 역행·sink가 요청보다 큰 값을 반환하면 영구 고장으로 침묵한다(호스트 임대가 만료되어 fail-closed).
- 최악 payload(같은 인코더 경로 계산): ack 856 B, state 247 B, offer 1,081 B, diag 1,062 B. 모두 1,536 B 이하.

검증과 한계는 [구동 계약 §19](../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#19-1022-후속--구동-출력-없는-송신-정책-후보)를 따른다. 실제 UART FIFO/드라이버 버퍼·pump 주기·대상 실행시간은 측정하지 않았다.

## 10/23 출력 비활성 연결부

`actuation_bench.hpp`는 receiver/ingress/TX를 한 소유자로 묶고 준비값을 항상 false로 둔다. mutable receiver/TX·피드백 주입 API를 노출하지 않으며 ARM/DRIVE를 허용하지 않는다. `feed/poll/pump`와 독립 `control_tick`을 같은 소유 작업에서 호출한다. ISR 재진입·실제 UART·출력 감시 드라이버는 구현하지 않았다. [공유 검사·전체 회귀·ESP32-S3 링크](../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#25-1023-후속--출력-비활성-통신-연결부)는 실제 보드 실행/물리 STOP과 구별한다.

## 10/23 단일 모터 PI 후보

`speed_pi.hpp`는 단일 모터축에서 환산한 평균 바퀴 속도와 목표 속도로 정규화 노력값을 계산한다. 고정 크기·불변 설정의 객체이며 설정 오류는 무효 객체, 입력/중간 산술 오류는 출력 0·제동 요청·적분 초기화로 반환한다. 정상 수식과 포화 쪽 적분 억제는 Python 참조와 같다. `reset()`은 적분만 지우며 ARM 허가를 만들지 않는다. gain 0.6/1.2·max dt 0.05 s는 임시 참조값이다.

IEEE binary64·`-ffp-contract=off`를 사용하고 fast-math/finite-math-only는 허용하지 않는다. `MotorIntent.brake_requested`와 음수 `effort`는 실제 PWM/DIR·역회전·전기 제동 신호가 아니다. 이 계산기는 명령 만료·독립 출력 감시·물리 STOP을 대신하지 않는다. [수정 전 반례·대조·한계](../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#27-1023-후속--단일-모터-속도-pi의-c-이관)를 따른다.

## 10/23 출력 없는 제어 주기 결합 후보

`motor_control_candidate.hpp`의 `MotorControlCandidate`는 receiver와 같은 소유 작업에서만 쓴다. `tick(now, output_ready=false)`는 receiver 만료/주기 감시 후 자신의 dt와 허가 세대를 확인하고 모터축 평균 속도로 PI를 계산한다. 반환 `MotorCyclePreview`는 해당 시각의 계산 의도이며 전기 출력 패킷이 아니다. 출력 준비가 실제로 확인됐다는 보장은 없다.

허가 세대가 바뀌면 적분을 초기화한 뒤 현재 목표를 계산한다. 같은 소유 작업의 신뢰된 receiver가 ARM·DRIVE를 이미 검증했으므로 두 명령이 제어 주기 사이에 연속 도착해도 중간 ARMED를 직접 관측하라고 요구하지 않는다(v1/v2의 관측 강제 조건은 최종 검토에서 제거). 비신뢰 상태 메시지를 받는 Gazebo `AppliedGuard`의 관문은 이와 별개로 유지한다. 출력 준비 상실·주기/설정 오류·산술 고장은 `report_control_fault`로 고정하며 기존 ESTOP/FAULT를 덮지 않는다. 이 API는 고장 고정만 가능하고 CLEAR/ARM·구동 허가 기능은 없다. 고장 뒤에는 정상 피드백과 명시 CLEAR→ARM이 필요하다. 통신/수신기 tick만 돌려 실제 PI 공백을 숨길 수 없다. 부팅부터 생성해야 하며 ACTIVE 중 교체한 객체는 재허가 전 거절한다.

UART/타이머/ISR/출력 감시·하드웨어 watchdog·PWM/DIR는 구현하지 않는다. 같은 작업이 멎으면 이 후보도 실행되지 않으므로 전기 STOP 보장이 아니다. 기존 출력 비활성 `ActuationBenchEndpoint`는 계속 구동 불가이며 이 후보와 자동 결합하지 않는다. [검사·미검증 범위](../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#28-1023-후속--출력-없는-제어-주기-결합)를 따른다.

## 10/23 출력 비활성 UART 주기 어댑터

`actuation_uart_bench.hpp`의 `UartBenchAdapter<Io, Clock>`는 `ActuationBenchEndpoint` 하나를 같은 소유 작업에서 `run_cycle()`로 돌린다. SDK 헤더를 포함하지 않으며 실제 드라이버 연결은 [`firmware_bench`](../firmware_bench/README.md)에 둔다. 준비값은 항상 false이고 endpoint는 const로만 노출한다.

- 순서: 시각 → 기한이면 `control_tick` 1회 → (전 주기에서 미룬 세션 경계) → [사건 확인 → 읽기 → 사건 확인 → `feed`] 반복 → `poll` → `pump` 1회 → 종료 시각. 늦은 tick은 한 번만 호출하고 실제 간격·놓친 기한 수를 기록하며 지난 기한을 재생하지 않는다. 수신/송신은 제어 tick을 호출하지 않는다.
- 수신 시각(`last_rx_buffered_us`)은 바이트가 고정 256 B 버퍼에 들어온 뒤 읽은 단조 시각이다. 선로 도착·원천 취득 시각으로 쓰지 않는다. 주기 실행시간(`start_us→end_us`, `max_cycle_us`)은 송신·입력 폐기·단절 뒤 시각을 **다시 읽어** 구하며, 이 종료 읽기의 역행도 시계 고장이다. `cycle_budget_us`는 새 읽기를 시작하지 않는 상한일 뿐 주기 전체 마감 보장이 아니다. 넘은 주기는 `cycles_over_budget/over_period`로 센다.
- 사건 예산 `max_events_per_cycle`은 **한 주기 전체** 상한이다. 바이트를 꺼내기 전에 그때까지의 사건을 모두 확인해야 하고(예산 안에서 큐가 비지 않으면 읽지 않음), 읽기 뒤 확인용 `post_read_event_reserve`가 남아 있어야 새 읽기를 시작한다. 읽은 뒤 오류 사건이 보이거나 예산 안에서 사건을 다 확인하지 못하면 그 조각은 연결부에 넣지 않고 세션을 끊는다. `next_event`가 비었다고 답한 직후 도착해 마지막 `pending_events()` 조회에 보인 사건도 내용 미확인으로 보며, 마지막 조회가 0일 때만 Clean이다. 바이트는 그 밖의 경우 드라이버 버퍼에 남는다.
- 사건 큐 포화: 소비자가 이 작업 하나뿐이므로 직전 확인 이후 가득 찼다면 다음 확인 진입 때도 가득 차 있다(진입 검사). 꺼내는 동안 다시 가득 차 있으면 종료 검사가 본다. **두 검사 사이 가득 찼다가 우리가 꺼내 내려간 순간 포화는 볼 수 없으며** 모든 사건 유실을 검출하지 않는다.
- 넘침·버퍼 가득 참·프레임/패리티 오류·break·기타 사건·포화는 세션 경계다. 연결부 단절(부분 프레임·송신 꼬리·ack 폐기, 필요 시 로컬 STOP) → 사건 큐 비움 → RX 폐기 → 새 세션 순서이며 CLEAR/ARM을 만들지 않는다. 한 주기 경계는 한 번이고 두 번째 오류는 다음 주기 시작으로 미룬다. 이미 전달한 완전 프레임은 되돌릴 수 없고 늦은 오류는 남은 부분 프레임만 버린다.
- 이미 TX FIFO에 들어간 바이트는 회수할 수 없다. 잘린 프레임은 호스트 CRC/재동기화가 버리지만, 남은 **완전한 이전 프레임은 CRC를 통과**하므로 호스트의 요청 ID·세션·세대 계약으로 배제해야 한다.
- 무수신을 USB 단절로 추정하지 않는다. 송신 0 B/부분 수락은 다음 주기로 넘기고, 오류(-1)·요청보다 큰 수락 보고·수신 오류·입력 폐기 실패·시각 역행/범위 밖은 영구 어댑터 고장이다. 고장 뒤에는 I/O·tick을 하지 않으며 재개는 새 객체(재부팅)뿐이다.
- `check_uart_plan`은 기본 비활성이며, 배선 확인 표시·포트/핀/baud 명시·콘솔 분리·UART0 부팅 출력 확인·ESP32-S3 예약 핀/배정 핀 충돌을 검사한다. VND018 실제 배선을 정하지 않는다.

SDK 의미(첫 바이트가 ring에 들어오는 시점, 사건과 바이트의 비동기 순서, 큐 포화 동작)는 로컬 소스를 읽은 해석이며 하드웨어에서 확인하지 않았다. 실제 UART 타이밍·ISR/드라이버 동시성·전기 신호·대상 실행도 미검증이다. [결과·한계](../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#29-1023-후속--출력-비활성-uart-주기-어댑터)를 따른다.
