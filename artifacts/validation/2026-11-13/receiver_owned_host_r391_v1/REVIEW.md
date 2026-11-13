# Receiver 기준 host 시간 정책 검토 r391

2026-11-13. **조건부로 가능한 작은 운용 후보다.** 현재 receiver가 이미 lease의 **발급 시각**에 source 잔여 수명을 붙이므로, 수락한 DRIVE의 source deadline을 연장하지 않는 데 MCU↔host 절대 offset/왕복 `ClockFit`과 forward-delay 상한은 논리 필수가 아니다. 다만 정확한 원천 시각/시계 비율 상한·lease와 age의 인과관계가 필요하며, 도착 성공이나 물리 정지를 보장하는 결론은 아니다. 기존 strictfit 경로는 유지하고 의미가 다른 별도 선택 정책으로만 연결하는 것이 타당하다.

코드 읽기와 수식 검토만 수행했다. 구현·pytest·시뮬레이션·장치/WSL 실행은 없으며, 이 문서만 추가했다. Fable 자문의 최종 결론이 아니라 GPT의 독립 검토다.

## 실제 receiver 식과 증명

[`ActuationReceiverCore::offer`](../../../../src/arena_vehicle_interface/portable/actuation_receiver.hpp) 132–139행은 실제 MCU `now`를 `issued_us`로 저장하고, `receive` 191–212행은 다음을 수행한다. Python [`ActuationReceiver.receive`](../../../../src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py) 322–341행도 같은 식이다.

```text
0 <= age_us < ttl_us <= receiver.lease_us
D = min(lease.expires_us, lease.issued_us + ttl_us - age_us)
receive_now >= D 이면 거절
```

정수/범위 조건은 [`drive_values_ok`](../../../../src/arena_vehicle_interface/portable/actuation_command.hpp) 647–665행에서 확인한다. C++의 `kMaxNowUs`, `kMaxConfigUs` 경계로 덧셈 overflow도 막는다. 식은 **수락한 DRIVE**에 대한 것이며 ARM의 대기 deadline은 기존 lease 만료다.

host 단조 시계의 원천 구간을 `[S−,S+]`, age를 계산한 시각 구간을 `[N−,N+]`, 원천 TTL을 `T ns`라 하자. 그 명령에 실제 넣을 lease의 발급 MCU tick을 `i`, 해당 tick의 host 시각을 `H(i)`라 둔다. 필요한 인과관계는 `발급 → 그 offer의 수신 완료 → age 계산`이므로 `H(i) <= N+`이다. offset 값을 알아야 이 순서를 알 수 있는 것은 아니다.

MCU 지속 시간1µs에 대응하는 host 지속 시간이 `1000r_min..1000r_max ns`이고 `0 < r_min <= r_max`라는 선언 범위가 유효하다고 가정한다. 기존 [`review_source_time`](../../../../scripts/actuation_source_time_review.py) 37–47행의 보수적 환산을 그대로 사용하면 다음과 같다.

```text
A = N+ - S− >= 0
a = ceil(A / (1000 r_min))
q = min(max_lease_us, offer_duration_us, floor(T / (1000 r_max)))
0 <= a < q 인 경우만 명령 생성

H(D) <= H(i) + 1000 r_max (q-a)
     <= H(i) + T - (r_max/r_min) A
     <= H(i) + T - A
     <= S− + T
```

양의 지속 시간의 비율 상한만 사용하므로 알려진 절대 offset이나 왕복 적합의 중앙값은 필요 없다. 범위가 해당 구간에서 유지되면 전 구간에 고정 affine offset을 적합할 필요도 없다. source가 미래가 아님을 `N− >= S+`로 확인하고, source/now/ARM 구간은 같은 유효 host epoch에 속해야 한다. source 구간 자체가 실제 `/drive/safe` 생성 시각을 포함한다는 전제는 별도다.

MCU 정수 tick은 실제 단조 타이머의 보수적 tick 경계로 해석해야 한다. 단순 µs 절삭은 발급 tick을 실제 발급보다 늦게 만들지 않아 위 인과관계에 맞는다. 반면 알려지지 않은 양의 timestamp 오차나 무제한 clock rate를 묵시적으로0으로 둘 수는 없다. host 읽기/ROS→단조 변환 오차는 source/now 구간에 포함하고, MCU tick의 추가 오차가 있다면 명시된 시간 예산에 반영해야 한다. 실측 없이 ppm이나 오차를 새로운 운영 기본값으로 확정하지 않는다.

## 큐·재사용·세대별 판단

| 쟁점 | 코드와 논리상 결론 |
|---|---|
| host 송신 큐 | **lease/age/TTL을 한 명령으로 고정한 뒤** 늦어지는 것은 `D`를 바꾸지 않는다. 송신 직전 원 source로 age를 다시 계산하면 더 보수적이다. 사실 같은 lease를 이미 받은 뒤 age를 계산했다면, 이후 고정 envelope의 지연을 age에 다시 더하지 않아도 위 비연장 증명은 유지된다. 늦은 명령의 도착 성공은 보장하지 못한다. |
| age 계산 후 lease 교체 | 성립하지 않는다. 예: source0ms, 계산20ms, TTL100ms, age20ms를 만든 뒤 발급60ms의 새 lease로 갈아끼우면 `D=140ms`가 되어 원100ms를 넘는다. 송신 시 새 offer를 고르는 경우 반드시 같은 원 source로 age도 함께 다시 계산해야 한다. |
| MCU 입력 큐 | receiver는 메시지 안의 송신/수신 시각을 믿지 않고 호출자가 준 현재 `now`로 만료를 검사한다. 처리 시각을 넘기면 오래 큐에 있던 프레임도 수명이 늘지 않는다. 과거 RX 시각을 `now`로 넘기면 늦은 명령 수락 여부가 잘못될 수 있으므로 금지해야 한다. 숫자 `D` 자체의 비연장과, 실제 출력 전에 만료를 검사하는 일은 구분한다. |
| 현재 MCU 연결점 | 범위상 필요한 호출자만 추가 확인했다. [`ActuationIngress`](../../../../src/arena_vehicle_interface/portable/actuation_ingress.hpp) 60–75/120행은 `feed/poll`의 현재 `now`를 receiver에 전달한다. [`runtime_drive/main.cpp`](../../../../src/arena_vehicle_interface/firmware_runtime_drive/main/main.cpp) 90–110행은 UART 처리 및 서보/모터 적용 전에 `esp_timer_get_time()`과 `check_deadlines/control_tick`을 다시 사용한다. 이는 RX 취득 시각으로 수명을 재시작하는 경로가 아님을 보여준다. 한 함수/배치 동안의 경과시간·태스크 정지·실제 PWM 유지 시간까지 증명한 것은 아니다. |
| 동일 lease 재사용 | lease의 `issued/expires`는 고정이다. 같은 age면 `D`도 같고, 원 source를 유지하며 age를 늘리면 `D`는 더 이르다. lease 재사용 자체는 source 수명 연장 사유가 아니다. 만료 lease는 `clock()`에서 제거한다. |
| 같은 source에 새 lease/seq | 원 source로 age를 다시 계산하면 매번 원 source deadline 이하라는 성질은 유지된다. 다만 receiver는 source ID/stamp를 받지 않으므로 같은 source인지 알지 못한다. stale 값을 새 생성 시각으로 찍거나 `age=0`을 반복하면 fresh lease로 계속 연장할 수 있다. 기존 host watermark를 없애지 않는다. receiver의 증가 seq는 동일 envelope 재생을 막을 뿐 source 중복 검사가 아니다. |
| ARM/STOP 세대 | receiver의 전환은 lease를 지우고 generation을 증가시킨다. 기존 boot/generation/owner/seq 검사를 유지하면 늦은 옛 명령은 새 ARM에 적용되지 않는다. host에서는 request에 맞는 ACK와 `base_generation+1` offer가 모두 필요하며, 둘을 처리한 시각의 `arm_barrier` 이후 source만 허용해야 한다. source deadline 비연장만으로 자동 재출발 방지를 대신할 수 없다. |
| ROS timejump/재연결 | 현재 `HostTimeWindow`는 선언된 host 단조 구간일 뿐 ROS stamp 변환기가 아니다. 원 ROS stamp를 수신/재발행 `now`로 바꾸면 증명의 `S`가 바뀐다. ROS 역행뿐 아니라 전진 점프·`use_sim_time`/clock 전환도 mapping epoch 무효화 대상이다. 현재 Session의 host 역행/epoch 검사만으로 ROS 점프가 검출된다고 볼 수 없다. mapping을 잃으면 pending source/송신 후보를 폐기하고 새 세션·명시 ARM 증거를 확보해야 한다. |

`receive()`는 첫머리 `check(now)`에서 기존 명령 만료/제어 공백/피드백 고장도 처리한다. 따라서 오래 지연된 다음 명령이 도착해도 이미 발생한 FAULT를 덮고 자동 ACTIVE가 되지 않는다. 반대로 새 명령만 `source_expired`로 거절되면 아직 유효한 기존 명령은 그 기존 deadline까지 유지될 수 있다. 이를 새 source의 수명 연장으로 해석해서는 안 된다.

## 현재 strictfit 경로가 증명하는 더 강한 조건

[`assess_offer`](../../../../scripts/audit_actuation_offer_clock.py) 32–96행은 `ClockFit`으로 발급/만료를 host 절대 구간으로 옮기고, 발급 나이와 `now.upper + forward_delay_upper < earliest_expiry`를 검사한다. [`review_source_time`](../../../../scripts/actuation_source_time_review.py) 53–62행은 도착이 source/receiver deadline보다 앞선지도 요구한다. 즉 단순한 비연장보다 강한 **선언된 지연 가정 아래의 도착 가능 조건**이다.

현재 [`CompactHostSession.begin_control`](../../../../scripts/actuation_host_session.py) 149–155행과 `review_source` 294–303행은 이를 직접 호출하므로, `fit=None`을 주거나 delay만0으로 두는 것으로 새 정책이 생기지 않는다. 전자는 거절되며 후자는 근거 없는 도착 상한 선언이 된다. 기존 `timestamp_policy_satisfied=True`를 무조건 반환하는 우회도 부적절하다.

최소 연결 후보는 **기존 세션/observer의 공통 상태 처리는 유지하고 시간 검토 부분만 명시 선택**하는 것이다.

1. strictfit은 그대로 기본/기존 경로로 보존한다. 별도 receiver-owned 정책은 offer schema/boot/generation/receipt 순서와 명시된 rate/TTL 조건만 검토한다. control의 실제 lease 유효성은 receiver가 판정하며, host가 만료 전 도착을 증명했다고 표시하지 않는다.
2. DRIVE에는 기존 올림 age·내림 TTL 산술과 원 source/ARM barrier/watermark를 유지한다. 명령을 만드는 곳에서 lease·source·age·TTL을 함께 고정하고, 큐에서 fresh lease를 자동 덮어씌우지 않는다. 검토 후 송신부가 `latest('offer')`를 다시 읽어 새 lease만 붙이는 순서도 같은 반례다. 현재 `review_source`는 검토만 하고 DRIVE/serial을 생성하지 않으므로 이 연결은 아직 구현되어 있지 않다.
3. 결과의 의미는 `receiver deadline nonextension under declared rate/source assumptions`으로 제한하고 도착 성공/`ready_to_drive`/장착 검증과 합치지 않는다. 실제 운영 주기에서 offer가 오래되어 거절이 잦을 수 있으며, fit 제거가 처리율 확보를 뜻하지 않는다.

추가 프로토콜·독립 보호층·새 TTL은 이 논리에 필요하지 않다. 기존 [`CompactHostObserver.observe`](../../../../scripts/actuation_host_observer.py)의 boot/연결/세대와 lease 단조성, Session의 ACK 상관·ARM barrier·STOP/CLEAR·source watermark를 재사용해야 한다. `CommandProducer`의 단순 host µs age는 이 방향의 기존 참조지만, 시계 비율 보수 환산과 ARM ACK/세대 barrier가 없는 만큼 그대로 운영 어댑터로 승격하지 않는다.

## 후속 반례 검사 권고 — 아직 실행하지 않음

한 묶음이면 충분하다. 동작 동일성 확인은 기존 strictfit 회귀를 유지하고, 새 정책에는 아래 핵심 사례만 추가하는 것이 적절하다.

- 비율 범위 양끝·ns/µs 경계에서 age는 올림/TTL은 내림을 유지하고 `H(D) <= source.lower + T`를 확인한다. age 내림·TTL 올림·실제 rate가 선언 상한을 넘는 반례도 함께 고정한다.
- 동일 envelope에 큰 host/MCU 큐 지연을 넣어 deadline 불변과 늦은 거절을 확인한다. 큐에서 lease만 새 것으로 교체하는 위20/60/100ms 반례는 거절해야 한다.
- 원 source를 유지한 같은 lease/새 lease의 반복과, source를 재발행 시각으로 잘못 찍은 경우를 구분한다. 기존 watermark는 동일/역행 원 timestamp를 막지만 새로 찍힌 timestamp의 출처를 알아낼 수는 없다. 따라서 어댑터가 원 timestamp를 바꾸지 않는지를 별도로 확인한다.
- STOP/만료 뒤 늦은 DRIVE·ACK, 옛 ARM 세대 offer, ARM 확인 전 생성 source, ROS mapping epoch 변경을 넣어 새 명시 ARM 이전에 복구되지 않음을 확인한다.

이 검토는 코드의 조건부 deadline 성질에 대한 결론이다. 원천 ROS→단조 구간 변환·rate/timestamp 오차의 실물 유효성, 스케줄 정지 중 출력 차단, USB 도착률, 모터/서보 장착 응답은 확인하지 않았다.

## 후속 staged 구현 독립 리뷰

2026-11-13. [`staged/scripts/actuation_host_session.py`](staged/scripts/actuation_host_session.py), [`receiver_owned_actuation_host.py`](staged/scripts/receiver_owned_actuation_host.py), [`test_receiver_owned_host.py`](test_receiver_owned_host.py)를 읽고 기존 공통 Session과 diff를 대조했다. **현재 순수 준비 범위에서 P1 결함, strictfit 행동 변경, 위 deadline 증명의 필수 조건 누락을 발견하지 못했다.** 정적 리뷰 결론이며 pytest 통과를 뜻하지 않는다. 코드는 변경하지 않았다.

- 기본 Session 변경은 두 timing 호출을 `_review_offer_time`/`_review_source_time` hook으로 옮긴 것이다. 기본 hook은 기존 observer/function에 같은 receipt·fit·now·source·ARM/boot/generation·한도 인자를 전달한다. ACK 상관, 세대 변경, STOP 선점, timeout, 원천 watermark의 기존 분기와 순서를 변경하지 않았다.
- 새 `source_time` 58–68행은 원천 나이를 `ceil(.../(1−δ))`, TTL을 `floor(.../(1+δ))`와 실제 offer 길이의 최솟값으로 만든다. 미래/ARM 이전 입력, 다른 epoch, 표현 불가능한 잔여시간을 거절한다. `offer_time`의 수신 완료→판단 순서와 결합하면 위 인과 증명이 적용된다. `max_receipt_age_ns`는 명시 입력인 추가 host 큐 제한이며 절대 발급 나이나 도착 성공 증명으로 표시하지 않는다.
- `source_time`이 `bound_offer=dict(offer)`를 검토 결과에 고정하고, `prepare_drive` 115–123행은 최신 offer를 다시 조회하지 않는다. 그 offer·age·TTL로 즉시 불변 `wire bytes`를 생성한다. 반환된 가변 envelope를 나중에 수정해도 이미 만든 wire에는 반영되지 않는다. 준비된 wire의 lease를 바꾸어 재인코딩하면 이 계약 밖이며 원 반례가 다시 생긴다.
- `prepare_drive`는 유효 수치 검사를 먼저 하되 유효 DRIVE는 공통 `review_source`를 거쳐야 하므로 확인되지 않은 ARM, STOP 이후, 다른 세대/owner/준비 상태, 동일/역행 source가 명령 생성으로 통과하지 않는다. 명령 seq와 제어 요청 seq를 공유하고 소진 시 세션을 닫는다. 빈 DRIVE `request`는 기존 portable TX가 명시적으로 ACK 생성을 생략하는 계약과 일치하며 envelope를 무시하는 값이 아니다.
- 테스트는 rate 범위/소수 µs, source 불확실 구간·미래/만료/ARM 이전, ACK/세대/watermark, 기존 receiver에 대한 큐 지연, age→새 lease 반례, 검토 직후 새 offer 수신과 wire 불변을 포함한다. 기존 strictfit 회귀를 staged 기본 Session에 연결하여 실행하면 hook 이동의 행동 보존을 확인할 수 있다. 이번 검토에서는 실행하지 않았다.

남은 전제는 이미 구분된 외부 연결 책임이다. `source_host_mapping_verified=False`, `hardware_rate_assumption_verified=False`, `arrival_before_expiry_proven=False`가 유지되며 ROS/serial이나 extended→compact offer 변환을 구현했다고 표시하지 않는다. 원 source를 `now`로 다시 찍는 오류는 이 클래스의 watermark가 자동 발견할 수 없다. 실제 송신부는 단일 소유자 순서를 유지하고 준비된 wire를 보내야 하며, STOP/연결 변경 후 큐에 남은 옛 명령의 송신 처리도 그 연결 단계에서 기존 STOP 우선순위에 맞춰야 한다. 지금 별도 보호층이나 새 프로토콜을 추가할 이유는 아니다.

검토한 파일 SHA256:

```text
staged/scripts/actuation_host_session.py
47c89b846d8b0dbcc50eb39ec207fa9253804a2abe0749474edff89ff9602a4c
staged/scripts/receiver_owned_actuation_host.py
005244d8918d4741c449a8f2f717a674c39cea643b360e52b751897777a6023f
test_receiver_owned_host.py
9353f3aa051b29fcc7d70e1cc6f7a5783000fee602f79a7f45ec190b5b15cdc0
```
