# 실물 모터축·IMU 입력 기록의 수동 검사 준비

2026-10-13 작성, 10/23 수동 바이트 기록 준비 추가. C1/C2 실물 이관 준비 도구다. **현재는 PC 입력만 검사했으며 실물 센서·ESP32 보드·드라이버·통신을 연결한 것이 아니다.** 제어/보호층의 시간·속도 한도를 바꾸지 않고, UART 설정·계수 적용·모터 구동도 하지 않는다. 아래 정규화 센서 감사와 뒤의 원시 바이트 기록은 서로 다른 층이다.

## 1. 무엇을 준비했는가

[`audit_hardware_capture.py`](../../scripts/audit_hardware_capture.py)는 향후 드라이버에서 내보낼 **정규화 기록 파일**을 읽는다. 이 형식은 MCU wire protocol의 변경안이나 실제 바이너리 패킷 형식이 아니다. 실제 드라이버가 정해지면 원본 기록을 보존한 채 이 검사 형식으로 내보내는 어댑터를 별도로 둔다.

검사 대상은 다음과 같다.

- 단일 모터축의 확장 signed 카운트·28카운트/모터 회전·15:1·반지름 0.0325 m 환산. 평균 바퀴 420카운트/회전, 약 0.4862 mm/카운트다. 좌우 속도나 차체 속도로 꾸미지 않는다.
- 취득 시각/수신 시각의 단조성, 센서 스트림별 순번 중복·역전·누락, 재부팅 식별자 변경. 새 부팅은 이전 카운트/시각과 연결해 속도를 계산하지 않는다.
- 실제 취득 간격별 1카운트 속도 단계와 관측된 signed 평균 바퀴 속도 범위. 거대한 카운트 점프를 자동으로 래핑 보정하거나 정상 물리값으로 승인하지 않는다.
- 사람이 외부에서 고정했다고 명시한 구간에 한해서 IMU 평균·표준편차를 **미적용 교정 후보**로 출력한다. 자료량·관측 시간·최대 간격 요구조건을 명시해야 하며 구간의 카운트 변화·순번 누락·끝부분 미관측/큰 간격·자료 부족/오류가 있으면 후보를 내지 않는다. 0카운트 자체로 정지를 추론하지 않는다.

`record_integrity_valid=true`는 파일 구조와 순서 검사 결과다. **주행 준비 완료가 아니므로 `ready_to_drive=false`는 항상 유지한다.** 누락 순번은 별도 계수이며 그것만으로 센서 자체 손실인지 전송/저장 손실인지 결정하지 않는다.

`recorded_sequence_complete`는 파일 안의 스트림별 연속 순번 검사다. 시작/끝에서 누락된 자료는 순번 연속성만으로 알 수 없으므로 **`complete_capture_verified=false`**도 유지한다. 각 부팅에서 엔코더/IMU가 둘 다 존재해야 파일 무결성이 참이며, 재부팅 전 자료로 새 부팅의 누락을 가리지 않는다.

## 2. 파일 계약

첫 줄은 `kind=header`, 이후 각 줄은 `kind=encoder` 또는 `kind=imu`인 JSON 객체다. [합성 예시](../../config/tests/hardware_capture_nominal_v1.jsonl)는 실제 측정치가 아니며 `origin=synthetic`을 유지한다.

| 항목 | 의미 |
|---|---|
| `device_clock=mcu_monotonic_us` | 같은 MCU의 확장 단조 취득 시각. 타이머 래핑은 생산자에서 먼저 처리 |
| `receive_clock=host_monotonic_ns` | 호스트가 받았을 때의 별도 단조 시각. ROS 시각과 몰래 혼용 금지 |
| `boot` / `seq` | 부팅 세대 / 각 센서 스트림의 증가 순번. 재부팅 뒤 예전 세대가 다시 오면 오류 |
| `motor_counts` | signed 64비트 범위의 누적 **모터축** 카운트. raw PCNT 또는 바퀴 카운트는 그대로 넣지 않음 |
| `gyro_rad_s` | 3축 각속도, rad/s. deg/s의 묵시 변환 없음 |
| `gyro_frame` / `mounting_note` | 기록된 축 좌표와 장착 설명. 설명만으로 실제 축 검증이 완료되지는 않음 |
| `externally_stationary_windows` | 부팅 식별자·취득 시각 구간·외부 고정 근거. 생략하면 교정 후보를 만들지 않음 |
| 구간의 `review_requirements` | `minimum_span_us`, `max_sample_gap_us`, `minimum_imu_samples`, `minimum_encoder_samples`. 사용자가 명시한 자료 충분성 조건이며 실물 교정 합격선은 아님. 없으면 후보 거절 |

현재 형식은 엔코더와 IMU가 같은 MCU 시계를 사용한다는 전제를 명시한다. 실물 IMU가 별도 장치 시계를 쓰게 되면, 그 시계를 억지로 MCU 시각이라고 이름 붙이지 말고 별도 계약/어댑터가 필요하다. 배선·최종 MCU 취득 구조를 이 파일로 확정한 것이 아니다.

## 3. 지연과 교정에서 하지 않는 추론

서로 다른 시계의 `수신 시각 − 취득 시각`은 전송 지연이 아니다. 두 시계의 시작점이 다르기 때문이다. 도구는 각 시계의 **구간 경과시간 차이**만 보고하며, 이 역시 전송 지터와 시계 드리프트가 섞인 수치다. 드리프트 추정값·절대 지연·시계 보정 계수로 자동 채택하지 않는다.

엔코더 0카운트는 모터 평균 관측값일 뿐 차체 정지 증명이 아니다. IMU 평균값도 장착 축·온도·진동·외부 고정 확인이 있어야 실제 bias 후보가 된다. 출력값은 어느 설정에도 자동 적용하지 않는다. 저수준 PCNT 입력률/펄스 누락·전기 신호 레벨·카운터 확장·센서 data-ready·전송 시간 매핑은 별도의 실물 의무다.

## 4. 실행과 다음 이관 단계

WSL에서 저장소 Python 패키지 경로를 설정한 뒤 다음과 같이 실행한다. 출력은 새 파일이어야 하며 입력/기존 보고서를 덮어쓰지 않는다.

```bash
PYTHONPATH=src/arena_vehicle_interface:scripts python3 scripts/audit_hardware_capture.py \
  config/tests/hardware_capture_nominal_v1.jsonl --output /tmp/capture-audit-new.json
```

실제 기록을 받으면 `origin=hardware_capture`와 실제 축·단위·사양·부팅 식별자를 명시한다. 추정 취득 시각을 실제 취득 시각으로 표시하지 않는다. 첫 이관 순서는 **구동 비허가 상태 입력 기록 → 전원/리셋/순서/단위 검사 → 외부 고정/수동 회전으로 축·부호·펄스 환산 대조 → 시간 매핑 설계 → 독립 정지 경계와 실물 낮은 속도 시험**이다. 외부 고정/수동 조작도 실제 장치 준비와 담당자 확인 후 수행한다.

[구동 계약](ACTUATION_CONTRACT_2026_09_22.md), [모터축 선정](DRIVETRAIN_GEARING_2026_10_02.md), [센서 입력 보호](../autonomy/SENSOR_INPUT_GUARDS_2026_10_04.md).

## 5. 검증 이력과 제한 묶음 마감

[v1](../../artifacts/validation/2026-10-13/hardware_capture_audit_v1/README.md)은 관련 89회귀·합성 파일 CLI를 확인했다. 이후 검토에서 순번 누락/부분 구간/재부팅별 스트림 누락에도 교정 후보가 나오는 허점을 발견했고 **옛 묶음으로 세 경우를 재현**했다. [v2](../../artifacts/validation/2026-10-13/hardware_capture_audit_v2/README.md)는 관련 **121회귀·7조건**에서 자료 품질과 교정 후보를 분리했다. 이전 결과·예시·패키지는 그대로 보존했다.

v2의 40 ms·5표본은 빠른 합성 회귀용 값이다. 실제로 충분한 측정 시간/주기는 데이터시트·취득 경로·온도 안정·현장 기록으로 정해야 한다. 표준편차도 출력 통계일 뿐 자동 통과 기준이 없으며, `calibration_ready=false`·`applied=false`다. 이 파일 감사 준비 묶음은 마감하고 실제 기록이 생기면 원본→정규화 어댑터와 시간 매핑 검증으로 이어간다. 없는 실측 계수나 MCU 프로토콜을 더 만들어 확정하지 않는다.

## 6. 왕복 시각으로 구하는 조건부 시간 구간

센서 기록 감사와 별도로 [`clock_bounds`](../../src/arena_vehicle_interface/arena_vehicle_interface/clock_bounds.py)와 [`audit_clock_exchanges.py`](../../scripts/audit_clock_exchanges.py)를 준비했다. **실제 동기화 통신이나 시계를 바꾸는 프로그램이 아니라, 향후 수집할 왕복 시각 증거의 오프라인 검사기**다. 왕복 메시지 형식·주기·MCU UART 프로토콜은 확정하지 않는다.

왕복 1회에서 호스트 송신 `h1`, MCU 수신 `m2`, MCU 송신 `m3`, 호스트 수신 `h4`를 각각 원래 시계로 기록한다. 같은 부팅의 짧은 구간에서 `host_ns = a × mcu_ns + b`가 성립한다고 명시적으로 가정하고 `h1 ≤ map(m2)`, `map(m3) ≤ h4`를 만족하는 모든 `(a,b)`를 남긴다. 방향별 지연이 같다고 가정하지 않으며 왕복 시간을 반으로 나누지 않는다. 타임스탬프 위치의 오차 상한은 부등식을 바깥쪽으로 확장한다.

범위 안의 모든 변환을 질의 시각에 적용해 호스트 시각 **하한/상한**을 출력한다. 중앙값을 교정 계수로 자동 채택하지 않는다. 정수 ns·유리수 계산으로 큰 uptime에서 유효 자릿수를 보존하고 결과는 바깥 방향 정수 반올림을 한다. 상한은 사용자가 입력한 주파수비 범위·timestamp 오차·affine 구간 가정이 실제 장치에서 성립할 때만 유효하다. 긴 시간의 온도 드리프트나 중간 clock reset까지 보증하지 않는다.

파일은 `arena_clock_exchange_audit_v1` header, 직렬화한 exchange 1~64개, query 1~64개다. `origin`, 별도 호스트/MCU 시계 단위, 오차·적합 시간·외삽 상한을 모두 명시해야 한다. 모호한 단위·JSON 중복 키·NaN·부팅 혼합·순서 역전·근거 모순·구간/자료량 한도 초과는 거절한다. 순번 공백은 보고하되 채우지 않는다. 한 파일은 같은 **호스트 단조 시계 세대**와 MCU boot에 한정한다. 호스트 재부팅/시계 변경도 새 기록·적합이 필요하며, 단조 수치만으로 물리적 재부팅을 전부 판별할 수는 없다.

[v1 검증](../../artifacts/validation/2026-10-13/clock_bounds_v1/README.md)은 42회귀와 합성 11조건에서 포함/거절을 확인했다. 예를 들어 1 ms/9 ms 비대칭 통신은 약 10 ms 폭으로 남았고, 반대 비대칭 왕복도 함께 주면 약 2.1 ms로 줄었다. 이는 장치 지연 측정이나 주행에 충분한 정확도라는 판정이 아니다. ±150 ppm 합성 skew, timestamp 오차, 순번 공백, 재부팅/역전/시계 점프/외삽 초과와 큰 시각 정밀도를 분리했다.

남은 실물 의무는 **타임스탬프를 실제로 찍는 위치와 지연 상한 확인 → 부팅/타이머 확장·왕복 원본 수집 → 구간 불확실성 확인 → host monotonic과 ROS 시각의 별도 연결 → 원래 입력 신선도/만료 정책과 대조**다. MCU 취득 시각을 호스트 수신 시각으로 바꿔 붙이지 않는다. `ready_to_drive=false`, `ros_stamp_verified=false`, `clock_mapping_applied=false`이며 기존 제어기/보호 시간 한도도 바꾸지 않았다.

### 6.1 10/15 신선도 문턱과 구간 불확실성의 대조

[`clock_freshness`](../../src/arena_vehicle_interface/arena_vehicle_interface/clock_freshness.py)는 실제 timestamp를 하나로 확정하지 않고 취득 구간 `[c_lo,c_hi]`와 호스트 결정 시각 `[n_lo,n_hi]`에서 나이 구간 `[n_lo-c_hi,n_hi-c_lo]`를 계산한다. 모든 가능한 나이가 0 이상이고 명시한 만료 한도 안일 때에만 `fresh_under_assumptions`다. 전부 만료/전부 미래/문턱에 걸친 불확실 상태를 구별하며, 상관 정보를 활용해 범위를 낙관적으로 줄이지 않는다.

[`audit_clock_freshness.py`](../../scripts/audit_clock_freshness.py)는 기존 exchange 원문과 별도 검토 JSON을 읽는다. 검토서는 원문의 SHA-256·합성/실측 구분·host 시계 세대·원래 query 인덱스·MCU boot·결정 시각 구간·한도/문턱 포함 여부·그 한도의 출처 설명을 명시해야 한다. 이는 MCU wire protocol이나 ROS timestamp 형식이 아니다. 시계 세대는 기록자가 확인한 주장이지 숫자만으로 증명되는 값이 아니다.

[79회귀·합성 12조건](../../artifacts/validation/2026-10-15/clock_freshness_v1/README.md)에서 분류와 거절을 확인했다. 예시의 21.95~31.95 ms 나이는 중간값만 쓰면 30 ms 아래로 보이지만 불확실 상태로 남았다. 정확히 문턱에 닿는 경우에는 명시한 `<`/`≤` 차이도 보존했다. 도구 종료 코드 0은 **진단 실행 성공**이며, 별도 `all_timestamp_policies_satisfied`와 언제나 거짓인 `ready_to_drive`를 혼동하지 않는다.

이 단계는 오프라인 판정 준비를 마감한다. 기존 timeout·평활·제어/보호 입력·기본 launch를 변경하지 않았으며 실물에서 한도를 완화해도 된다는 근거가 아니다. 다음은 실제 기록에 대한 원문→정규화 어댑터와 host monotonic↔ROS 연결이다. 장치/시계 정보가 없는데 자동 보정 계수나 통신 프로토콜을 더 만들어 확정하지 않는다.

### 6.2 10/15 호스트 단조 시각과 ROS 시각의 동시 읽기 준비

[`capture_clock_pairs.py`](../../scripts/capture_clock_pairs.py)는 `monotonic → ROS now → monotonic` 순서로 읽고
파일 쓰기는 그 구간 밖에서 수행한다. [`clock_pair_audit`](../../src/arena_vehicle_interface/arena_vehicle_interface/clock_pair_audit.py)의 읽기 함수를 나중에 실제 드라이버의 **그 노드가 사용하는 시계**에 연결할 수 있으나,
현재 CLI는 `TimeSource`가 없는 독립 `rclpy.ROSClock`만 읽는다. 다른 노드의 `use_sim_time`·센서 취득 시각·Jetson의 상태를 확인한 것이 아니다.

읽기마다 모드 전후·시각 구간·순번·명시한 시계 세대를 남긴다. CLI의 세대 식별자는 실행별 UUID이며 OS boot ID 검증을 대체하지 않는다.
완료 footer·자료 수·순서·세대를 검사하고 ROS 시각 0·정지·역행·모드 변경은 별도 사건 목록으로 보존한다.
미완료 기록은 무결성 부적합으로 남긴다. 유효하게 기록된 시계 역행과 파일 파손은 서로 다른 문제다.

물리 시각 모드가 유지된 경우에만 각 읽기에서 가능한 일정 offset `[ROS−host_after, ROS−host_before]`의 교집합을 진단한다.
이는 **읽은 표본들과 일정 offset 가설이 양립한다는 뜻**이며 표본 사이 연속성·장기 drift·절대시각 정확도·자동 변환 허가가 아니다.
교집합이 비어도 NTP·점프·drift 중 원인을 바로 확정하지 않는다.
ROS 시간원은 정지·역행할 수 있으므로 활성화된 모의 시간원을 실물 시각 변환에 사용하지 않는다.
[ROS 시간 설계](https://design.ros2.org/articles/clock_and_time.html)와 실행 환경에 설치된 rclpy `clock.py`/`time_source.py`를 대조했다.

| 검증 | 결과 | 범위 |
|---|---|---|
| [PC 500표본](../../artifacts/validation/2026-10-15/clock_pair_capture_v1/README.md) | 약 10.135초, 완료·무결성 확인. 일정 offset 교집합 폭 13.125 μs, 최대 읽기 구간 247.119 μs. 관련 회귀 111개 통과 | PC 독립 시계 1회. 센서 지연/동기화 오차/실차 허용치가 아님 |
| [설치 rclpy 7조건](../../artifacts/validation/2026-10-15/clock_pair_ros_modes_v1/README.md) | 조건당 3표본, 총 21표본의 기대 사건 판정 통과. 0 시각 목록 추가 후 관련 회귀 112개 통과 | 활성 전/미초기화/증가/정지/역행/시간원 전환/읽기 중 전환의 합성 fixture |

두 묶음은 서로 다른 소스 시점의 별도 검증이며 개수를 합산하지 않는다. ROS 노드·DDS·`/clock` 발행/구독·시스템 시계·Jetson을 건드리지 않았다.
500표본 원문은 로컬 제외 정책으로 보존하며 요약·해시·실행 당시 소스 ZIP은 별도로 남긴다.
`between_samples_continuity_verified=false`, `ros_stamp_verified=false`, `clock_mapping_applied=false`, `ready_to_drive=false`는 유지한다.

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=src/arena_vehicle_interface:scripts python3 scripts/capture_clock_pairs.py \
  --samples 500 --period-ms 20 --output /tmp/clock-pairs-new.jsonl
PYTHONPATH=src/arena_vehicle_interface:scripts python3 scripts/audit_clock_pairs.py \
  /tmp/clock-pairs-new.jsonl --output /tmp/clock-pairs-audit-new.json
```

**이 준비 묶음은 마감한다.** 다음 실물 단계는 실제 드라이버 시계·timestamp 위치·reset 식별자를 정하고,
그 시계에서 왕복 원문/센서 기록/동시 읽기를 함께 수집해 신선도 계약과 대조하는 것이다.
독립 CLI의 작은 읽기 구간을 근거로 MCU→ROS timestamp 변환을 자동 적용하지 않는다.

## 7. 10/23 — 명령 없는 PC 원시 바이트 기록

[`actuation_raw_capture.py`](../../scripts/actuation_raw_capture.py)는 §1의 정규화 센서 감사보다 앞단에 놓이는 **PC 파일/파이프 기록 도구**다. 실제 serial backend는 아직 없으며 장치 탐색·포트 설정·DTR/RTS 변경·ROS 연결·ARM/DRIVE/STOP 송신을 하지 않는다. CLI는 Linux/WSL의 일반 파일 또는 이미 연결된 stdin pipe만 받는다. 파일은 `O_PATH`로 대상을 고정해 일반 파일임을 확인한 뒤 같은 inode를 읽으며, symlink·FIFO·장치를 read-open하지 않는다. stdin도 터미널/장치는 거절한다.

### 7.1 기록과 완료의 의미

새 폴더에 아래 세 파일을 쓰며 기존 폴더를 덮어쓰지 않는다. 자동 생성하는 `.gitignore`는 원시 세 파일을 공개 저장소에서 제외한다.

| 파일 | 의미 |
|---|---|
| `raw.bin` | 읽은 바이트 그대로. 손상 프레임·콘솔 잡음·중복도 삭제하지 않음 |
| `chunks.jsonl` | header, chunk별 순번·offset·길이·read 호출 전후 host 단조 시각·SHA-256, footer의 종료 이유·전체 길이/해시 |
| `closed.json` | raw와 footer까지 포함한 metadata의 해시. 저장을 끝냈다는 표식이며 인증 서명/원자적 전원 장애 복구 장치가 아님 |

`host_epoch`/`connection_id`는 이 수집 프로세스가 만든 식별자다. OS/MCU의 실제 boot나 통신 상대 인증값이 아니다. read 구간에는 입력 대기와 userspace 복사가 포함될 수 있으며, 선로 도착·센서 취득 시각이 아니다. 파일 재생을 기록하면 **지금 파일을 읽은 시각**이지 원래 촬영/수신 시각이 아니다. frame이 여러 chunk에 걸치면 원본과 chunk 순서를 보존하여 재현하지만 정확한 frame 시작/끝 offset이나 원천 시각을 새로 추론하지 않는다.

기본 제한은 1 MiB·10,000 chunk·60초이고 최대 허용은 16 MiB·100,000 chunk·600초다. 제한값은 저장 도구용이며 MCU 보호/명령 TTL이 아니다. 한 번 읽는 양은 최대 65,536 B다. byte/chunk 상한에 정확히 닿아도 EOF 확인을 위한 추가 바이트를 읽지 않으므로 `byte_limit`/`chunk_limit`로 남긴다. 조용한 pipe도 대기 기한으로 끝난다. 파일/디스크 syscall 자체의 강제 취소·최악 실행시간은 보장하지 않는다.

EOF일 때만 `completed=true`다. 시간/크기/개수 제한, Ctrl-C, 읽기 오류, 시계 역행은 정상 저장된 **불완전 수집**으로 구별한다. 저장 중 오류·강제 종료로 footer나 완료 표식이 없거나 잘리면 감사에서 파일 무결성을 거절한다. 정상 표식이 있어도 실제 UART/USB 또는 상위 생산자에서 이미 버린 바이트는 알 수 없다. 유한 메모리로 동기 기록하며 디스크가 느리면 source read도 늦어진다. 이를 센서 자체의 지연/누락으로 단정하지 않는다.

### 7.2 오프라인 감사의 층

- `record_integrity_valid`: 제한·순번·offset·길이·각 chunk/전체 해시·footer·완료 표식·추가 꼬리 대조 결과. 해시는 우연한 손상/파일 혼합을 잡는 수단이며 함께 고쳐 쓴 공격을 막는 인증 수단은 아니다.
- `timing_valid`: 기록된 host 시각의 순서만 검사한다. MCU↔host 시계 변환이나 전송 지연의 증거가 아니다.
- `decoded_messages`/`decoder_errors`/`decoder_pending_bytes`: 기존 `actuation_wire.Decoder`로 재생한 결과. 오류 수는 decoder의 원래 정의이며 모든 콘솔 잡음 바이트를 세지 않는다. EOF에서 불완전 frame 꼬리가 남아도 원시 파일이 손상됐다는 뜻은 아니다.
- `schema_accepted`/`schema_rejected`: 기존 strict `validate_payload`로 형식을 검사한 수. 형식상 유효한 메시지를 운영 호스트가 수락했다는 뜻은 아니다. 인접한 동일 메시지와 관측 boot 전환은 진단 수치로만 세고 삭제하지 않는다.
- `CompactHostObserver`/요청 상관/시간 적합/구동 허가는 적용하지 않는다. `host_observer_applied=false`, `complete_capture_verified=false`, `firmware_verified=false`, `ready_to_drive=false`다.

```bash
# 예시: 이미 보존한 일반 바이트 파일. 실제 장치 경로를 넣지 않는다.
PYTHONPATH=src/arena_vehicle_interface:scripts python3 scripts/actuation_raw_capture.py capture \
  --input /tmp/synthetic-wire.bin --output /tmp/new-raw-capture --duration-ms 1000
PYTHONPATH=src/arena_vehicle_interface:scripts python3 scripts/actuation_raw_capture.py audit /tmp/new-raw-capture
```

capture의 exit 0은 EOF 기록 완료, 2는 종료 이유가 남은 불완전 수집, 1은 설정/파일 오류다. audit의 exit 0도 저장/host 시각 계약일 뿐 protocol·부품 합격이 아니다. 실제 포트를 열어 얻는 원시 기록은 별도 장치 backend와 reset/콘솔/배선 검토 후에 연결한다.

### 7.3 검증

[v1](../../artifacts/validation/2026-10-23/actuation_raw_capture_v1/README.md)은 실행 스크립트의 import 경로 누락으로 수집 오류 2건·exit 2였으며 본문 미실행이다. 경로만 명시한 [v2](../../artifacts/validation/2026-10-23/actuation_raw_capture_v2/README.md)는 기존 호스트 보관기와 합쳐 **82통과·skip 0·exit 0·소스 불변**이다. 분할/손상/중복/부분 꼬리와 완료 표식 손상·저장 실패, 실제 PC pipe 대기 종료, C++ host 송신 프레임을 대조했다. 보드·실제 포트는 열지 않았다. 원본 시각을 추론하지 않는 기록 도구이며 다음 의무는 실제 포트 연결 시 reset/드라이버 버퍼/읽기 위치를 함께 확인하는 것이다.

## 8. 10/23 — 독립 가속도 표본을 보존하는 v2

[`audit_hardware_capture.py`](../../scripts/audit_hardware_capture.py)에 `arena_hardware_capture_v2`를 추가했다. 기존 v1 파일/출력 계약은 유지하며, v2에서만 부팅별 `encoder`·`imu`(자이로)·`accel` 세 스트림을 요구한다. 자이로 표본에 가속도를 억지로 맞춰 붙이지 않고 **각자의 순번·취득 시각·수신 시각**을 기록한다. 같은 숫자의 시각이어도 물리적 동시 취득이 입증되는 것은 아니다.

header에는 `accel_unit=m/s^2`, `accel_frame=gyro_frame`, `accel_semantics=specific_force_including_gravity`, `accel_timing=independent_acquisition_stream`, 축별 `accel_full_scale_mps2`를 명시한다. 이는 센서 좌표계의 비력(specific force)이다. 정지 상태에서 센서 z축이 위를 향하면 약 +g가 나오는 [ROS REP-145 관례](https://github.com/ros-infrastructure/rep/blob/master/rep-0145.rst)를 따른다. 필드 이름만으로 실제 축/부호/보정 상태를 검증하지 않는다. 중력을 이미 제거한 값을 같은 의미로 기록해서는 안 된다.

| `status` | `specific_force_mps2` | 감사에서의 처리 |
|---|---|---|
| `available` | 유한한 3축 벡터 | 선언한 축별 full-scale 이상이면 별도 오류. 유효한 표본만 통계에 사용 |
| `unavailable` | `null` 필수 | 누락 사실은 보존하며 0으로 채우지 않음 |
| `saturated` | 유한한 벡터 또는 `null` | 포화 관측을 보존하고 정상 표본 통계에서 제외 |

정상적으로 기록한 누락/포화는 **파일 무결성 통과**와 양립한다. `acceleration_review.all_recorded_samples_available` 등 이용 가능성은 별도로 본다. 같은 부팅의 스트림 누락·시각 역행·순번 공백·부적절한 단위/상태/NaN·선언 범위 위반은 기존 감사와 함께 검사한다. 통계는 축별/벡터 크기 요약일 뿐 자세 추정·방지턱 검출·가속도 교정·센서 동기화가 아니다. 정지 자이로 편향 후보도 기존 제한을 유지하며 가속도 평균으로 중력/바이어스를 자동 분리하지 않는다.

[합성 예시](../../config/tests/hardware_capture_accel_v2.jsonl)는 8g에 해당하는 78.4532 m/s²를 **파일 선언값 예시로만** 쓴다. 실제 LSM6DSOX 설정/검증값으로 채택하지 않았다. available/unavailable/saturated를 하나씩 넣은 9행의 작은 감사 예이며 실물 수신기·wire protocol·ROS 입력은 변경하지 않는다.

[v1 증거](../../artifacts/validation/2026-10-23/hardware_accel_v1/README.md): 기존 v1과 신규 v2 관련 **77통과·감사 CLI exit 0·소스 불변**. 다음은 실제 장치 기록에서 full-scale·축 방향·timestamp 위치·드라이버의 포화 표식을 확인하고 정규화 어댑터에 연결하는 것이다. 가속도가 없던 옛 로그에 값을 합성하여 추가하지 않는다.

## 9. 10/23 — LSM6DSOX 입력을 연결하기 전

배선 후보는 [ESP32의 SPI + INT1](ESP32_BUDGET_REVIEW_2026_10_02.md#현재-구성의-핀-예산)다. 이번에는 장치·보드를 연결하거나 드라이버/핀/ODR/필터를 채택하지 않았다. 아래는 **공개 원문 감사와 이관 조건**이며 기존 시뮬레이션 100 Hz·보호 한도를 유지한다.

### 9.1 읽는 함수의 단위와 시각부터 구별한다

- [ST 기본 드라이버](https://github.com/STMicroelectronics/lsm6dsox-pid/blob/2e86793125f7f46b6195fce93a2b3cb76a51e564/lsm6dsox_reg.c)는 각속도 변환을 **mdps**, 가속도 변환을 **mg**로 반환한다. 이를 ROS의 `rad/s`, `m/s²`로 이름만 바꾸면 안 된다. 예를 들어 ±500 dps 설정의 raw 1,000은 17.5 dps ≈ 0.305433 rad/s다. 설정 범위를 모르면 변환 계수도 정할 수 없다.
- 확인한 [Adafruit Arduino 구현](https://github.com/adafruit/Adafruit_LSM6DS/blob/379a5204c0bad71264c3d635de84d0f9679ab784/Adafruit_LSM6DS.cpp)은 이미 rad/s·m/s²로 변환한다. `getEvent(accel, gyro, temp)`의 timestamp는 `_read()` 직전의 `millis()`이며 센서 내부 취득 시각이 아니다. 개별 센서 `getEvent`는 읽은 뒤 시각을 붙인다. 같은 함수 이름만으로 같은 시각 의미라고 볼 수 없다.
- 같은 Arduino 구현은 `_read()`에서 버스 읽기 반환값을 소비하지 않고 `getEvent`가 true를 반환한다. 따라서 **이 반환값만으로 새 정상 표본을 받았다고 승인하지 않는다.** 실제 버스 고장을 재현한 결과는 아니며 선택할 버스 backend와 드라이버 판본을 다시 확인해야 한다.
- [CircuitPython 구현](https://github.com/adafruit/Adafruit_CircuitPython_LSM6DS/blob/cdfc14a687a138aa0f2c6abab061bfc1bfafa561/adafruit_lsm6ds/__init__.py)의 `gyro`도 rad/s다. [안내 페이지](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/python-circuitpython)의 일부 degrees/s 표현과 예제의 radians/s 표현은 맞지 않는다. 이번 판본은 실제 변환 소스를 기준으로 확인했으며 CircuitPython 사용을 선택한 것은 아니다.

### 9.2 센서 시계·data-ready·전송 시계

[ST DS12814 Rev 4](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf)의 §9.17·§9.44와 ODR/FIFO 항목을 대조했다. 센서 timestamp는 **32비트·25 μs 단위**이고 MCU 단조 시계와 별개다. BDU는 읽는 중 상하위 바이트 갱신을 제어하는 기능이지 표본 무손실·여섯 축 동시 취득 보장이 아니다. 104/208 Hz 등 실제 ODR 선택과 FIFO 배치율·timestamp 배치는 서로 구분한다. FIFO의 연속 모드는 가득 차면 오래된 항목을 덮어쓴다.

따라서 다음을 이관 조건으로 둔다.

1. **첫 단계는 구동 없는 raw 기록:** 장치 식별, FS/ODR·필터·BDU/자동 증가·FIFO/인터럽트 설정의 읽기 되돌림, 원시 정수/상태를 보존한다. 초기 설정 성공과 표본 읽기 성공을 별도로 검사한다.
2. **새 표본인지 검사:** DRDY/상태와 실제 읽기 결과를 확인하고 data-ready 관측 시각, SPI 읽기 시작/끝, 상위 전달 시각을 나눈다. 폴링할 때마다 새 순번/취득 시각을 붙여 동일 레지스터 값을 새 측정으로 만들지 않는다. 반대로 값이 같다는 이유만으로 정상 정지 표본을 중복이라고 버리지 않는다.
3. **FIFO를 쓰면 원래 tag/순서/overrun을 보존:** 읽은 묶음 전체에 같은 MCU 수신 시각을 취득 시각으로 붙이지 않는다. 내부 timestamp와 각 accel/gyro 항목의 관계부터 검증한다. 센서 reset/wrap·설정 변경·FIFO 유실은 세대/불연속으로 남기고 빈 표본을 보간해 숨기지 않는다.
4. **시계 변환과 지연은 별도 검증:** 센서 시계를 사용하는 경우 §1의 `mcu_monotonic_us`에 그대로 넣을 수 없다. 원래 센서 값과 대응 MCU 시간 구간을 먼저 보존한 뒤 명시적 변환/불확실성 계약이 필요하다. 인터럽트 시각도 내부 필터 지연을 포함한 물리 측정 시각의 정답으로 단정하지 않는다.

ST 원문의 `timestamp_raw_get` 출력은 `uint32_t`지만 `from_lsb_to_nsec` 보조 함수 인수는 `int16_t`다. **32비트 전체 카운터를 그 보조 함수로 축소 전달하지 않는다.** 충분한 정수 폭에서 단위를 변환하고 wrap/reset은 별도로 처리해야 한다. 함수 이름만 보고 조합하면 긴 기록에서 시간이 망가질 수 있다.

### 9.3 현재 소비자와 남은 경계

[`MotionInput.on_imu`](../../src/arena_autonomy/arena_autonomy/lidar_motion_ros.py)는 각속도 세 축을 장착 roll/pitch에 따라 차체 yaw로 투영한다. quaternion/가속도는 이 보정에 사용하지 않는다. 이번 감사에서 IMU frame 일치 검사가 없는 것을 확인하여 **명시형 `motion_imu_frame_id` 옵션**을 추가했다. 설정하면 다른/빈 `header.frame_id`를 `gyro_frame_mismatch`로 거절하며 이력 시각을 갱신하거나 값을 대신 만들지 않는다. 기존 finite·범위·시간·급변 검사는 유지한다.

빈 문자열 기본값은 옛 bag/시뮬레이션 호환 동작을 유지하기 위한 것이며 실물 좌표 검증 완료를 뜻하지 않는다. 현재 launch에는 실제 장치 frame을 가정해 넣지 않았다. 실물 연결 시 기대 이름과 장착 변환을 함께 확정해야 한다. 이름이 일치해도 값의 축/부호가 실제로 맞는지는 별도로 검사해야 하며 TF 변환을 자동 적용하는 기능도 아니다.

장착 시 x 전방·y 좌측·z 위 기준과 우수 회전 부호, 정지 시 중력 방향, 천천히 각 축을 돌린 결과를 확인한다. 적용된 단위/축 변환은 한 곳에서만 수행하며 원시 축도 남긴다. 이 과정은 영점/온도 교정·중력 제거·자세 추정·미세 필터 채택과 별개다. ±500 dps 입력 한도나 slew 한도를 바꿔 잘못된 단위/축을 통과시키지 않는다.

[제조사 원문 PC 대조](../../artifacts/validation/2026-10-23/imu_source_contract_v1/README.md)는 exit 0·ASAN/UBSAN 오류 없음·소스 불변이다. 각 FS의 int16 전체 변환·부호·timestamp·버스 오류 반환을 가짜 레지스터로 대조했으며 실제 센서/MCU 입력·시각 정확성·실물 frame 검증은 아니다.

[명시 frame 옵션 회귀](../../artifacts/validation/2026-10-23/imu_frame_gate_v1/README.md)는 추가 20사례를 포함한 전체 **4,301통과·실패/skip 0·소스 불변**이다. 이름 불일치/비어 있음·기본 호환·장착 투영·거절 후 정상 복귀·계속 거절할 때 이력 만료를 확인했다. ROS 메시지 객체의 PC 검사이며 실제 DDS/장치·새 Gazebo 주행 검증은 아니다.

후속 [설치본 실제 ROS 25조건](../../artifacts/validation/2026-10-23/imu_frame_ros_v1/README.md)은 격리 DDS·합성 시계/IMU·단일 모터 입력으로 위 거절/기본 호환·이력 만료/복귀·시계 되감김을 확인했다. 오류 0·정상 종료·설치 소스 불변이며 제어 명령은 발행하지 않았다. 입력 어댑터의 ROS 전달 보충이지 실제 축/교정·제어기/차량 정지 경로의 검증이 아니다.

## 10. 10/23 — 제어기를 켜지 않는 ROS 센서 점검 기록

실물 단품을 연결할 때 기존 [`record_sensor_bag.py`](../../scripts/record_sensor_bag.py)를 그대로 사용하면 두 전제가 맞지 않는다. 기존 `autonomy_sim`은 `/clock` 기반 시뮬레이션 시계와 두 제어기 매개변수·구동/보호 명령·계측 토픽을 요구한다. **기존 기본 계약은 유지**하고, 별도 `sensor_probe` 선택 모드를 추가했다. 실제 장치를 연결하거나 새 드라이버를 구현한 것은 아니다.

| 구분 | 기존 `autonomy_sim` | 선택형 `sensor_probe` |
|---|---|---|
| 목적 | 당시 제어기와 입력의 격리 재생 | 구동 없이 센서 단품/조합의 원래 ROS CDR 보존 |
| 시계 | 기존 simulation 유지 | `--clock-mode system` 또는 `simulation` 명시 필수 |
| 기록 대상 | 기존 센서·비교용 명령/상태/계측 | `--sensor-topic`으로 선택한 기존 센서 허용 목록만 |
| 제어기/명령 | 제어기 매개변수 2곳과 관측 토픽 필수 | 제어기 매개변수 요청·명령 토픽 구독 없음, 제어 출력 없음 |
| manifest | 기존 schema 2/모터축 기준 | 별도 `arena_sensor_probe_v1`, 기존 제어기 재생기가 거절 |
| 완료 의미 | 기존 기록기 내부 완료 | 선택한 모든 센서 수신·내부 큐 손실/오류/수신 시계 역전 없음·정상 파일 종료. 센서 시각/값/실물 승인 아님 |

### 10.1 사용과 기록의 경계

이미 검토한 생산자가 선택 토픽을 발행하는 상태에서만 다음과 같이 쓴다. 아래는 **첫 실물 실행 승인이나 포트 자동 탐색 명령이 아니라 사용 예시**다. 모터축 입력도 기록하려면 현행 계약의 `/drive_motor/encoder`를 명시한다. 실제 발행 이름·형식은 `drive_feedback.TOPIC_BY_MODE`와 대조한다.

```bash
python3 scripts/record_sensor_bag.py \
  --profile sensor_probe --clock-mode system \
  --sensor-topic /imu/data --sensor-topic /scan \
  --output artifacts/hardware_capture/session_name/sensor_recording \
  --max-wall-seconds 60 --max-raw-gib 0.1
```

필요한 구간을 받은 뒤 Ctrl+C로 정상 종료한다. 시간/용량 한도 도달은 성공 종료 예약이 아니라 **불완전 기록 보존용 안전 한도**다. 선택 센서 하나가 오지 않으면 `complete=false`다. 기존 폴더는 덮어쓰지 않으며, 원시 bag·상세 receipt·ready는 산출물 내부 `.gitignore`로도 제외한다. 이전 `sensor_recording/` 경로 기반 제외 정책을 유지한다.

```bash
python3 scripts/audit_sensor_probe.py \
  artifacts/hardware_capture/session_name/sensor_recording \
  --output artifacts/hardware_capture/session_name/sensor_probe_audit.json
```

[`audit_sensor_probe.py`](../../scripts/audit_sensor_probe.py)는 파일 해시·토픽/형식·원시 CDR 개수/누적 해시·receipt 한 행과 같은 메시지의 수신 시각/바이트수/원본 header 대응·수신/쓰기 단조 순서를 검사한다. 파일 해시만 다시 계산해서 내용 불일치를 숨긴 반례도 거절한다. JSON 중복 키/비유한 값·잘린/초과 행·명령/정답 토픽·미완료 자료는 거절하며 기존 제어기 재생기로 자동 변환하지 않는다.

**원래 header 0·역전·잘못된 clock은 고쳐 쓰지 않는다.** 감사 결과의 `source_stamp_zero/regressions`는 진단이고, 원본 내용을 정확히 기록했다는 결과와 양립한다. `sensor_acquisition_time_verified=false`, `controller_replay_eligible=false`, `hardware_verified=false`, `ready_to_drive=false`를 유지한다. 센서 header와 recorder ROS 시각의 차이를 검증된 지연으로 바로 보고하지 않는다. 현재 신호/값의 물리 유효성 검사는 센서별 소비자와 원시 감사의 별도 책임이다.

이 도구가 저장하는 것은 **ROS 생산자 이후 자료**다. C1 정렬 전 패킷/SDK callback 시각이나 B0183 노출/PTS 메타데이터가 상류에서 빠졌다면 복구하지 못한다. 그런 자료는 앞 절과 센서별 이관 문서에 따라 별도로 보존해야 한다. 실물/대회용 주행 노드에 연결하거나 unknown을 자유 공간으로 바꾸지 않는다.

### 10.2 PC 검증과 실패 보존

- 관련 단위/기록 계약 회귀 **138통과**, 저장 CDR 재감사 통과·소스 불변: [checks v3](../../artifacts/validation/2026-10-23/sensor_probe_checks_v3/README.md). 기존 4,301 전체 회귀와 합산하지 않는다.
- 격리 localhost ROS에서 **제어기 없이** IMU/scan 각 80개를 system 시계로 기록했다. 각각 header 0은 20개·역전은 39회였으며 원래 CDR 바이트가 그대로 보존됐다. 기록기 구독은 해당 센서 두 개뿐이었다. [ROS v3](../../artifacts/validation/2026-10-23/sensor_probe_ros_v3/README.md).
- simulation 선택에서 `/clock`이 없으면 샘플을 만들어 넣지 않고 `complete=false`·exit 1이었다. 정상 `/clock`을 주면 두 센서 기록/감사가 통과했다. [ROS v4](../../artifacts/validation/2026-10-23/sensor_probe_ros_v4/README.md).
- 수신 시계 역전 후 외부 신호 없이 자체 종료하는 시험은 `clock_regressions=1`·이전 표본 보존·`complete=false`·exit 1이었다. [ROS v5](../../artifacts/validation/2026-10-23/sensor_probe_ros_v5/README.md). 원래 header 역전과 기록기 수신 시계 역전의 처리가 다른 것을 확인했다.
- 기존 옵션 없는 기록기의 시간 한도/미완료 파일 보존·기존 재생 거절은 7조건을 그대로 통과했다. [기존 모드 대조](../../artifacts/validation/2026-10-23/sensor_probe_old_default_v1/report.json).

첫 inline 환경의 `rclpy` import 실패, 다음 시험기의 DDS node graph 준비 경쟁, 시계 역전 시험기의 중복 SIGINT 종료 실패를 각각 v1/v2/v4에 보존했다. 환경/시험기만 수정했고 기록기/보호 수치를 늘려 성공시키지 않았다. 실제 SDK/센서/Jetson·고속 영상 처리량·장시간 쓰기·새 폐루프는 이번 검증이 아니다. 소스/실패 기록은 보존하고 새 장치 정보가 없으면 같은 연결 검사를 반복하지 않는다.

### 10.3 Humble 메타데이터 호환과 휴대 묶음

기록기의 기존 `TopicMetadata(id=...)` 생성은 Jazzy에 맞지만 [공개 Humble binding 원문](https://github.com/ros2/rosbag2/blob/0aa2e05fe9cb3983812fec6c078c9d0bb4590353/rosbag2_py/src/rosbag2_py/_storage.cpp)의 생성자는 `id`를 받지 않는다. [`rosbag_writer_compat.py`](../../scripts/rosbag_writer_compat.py)에서 설치 API에 `id` 속성이 있는 경우에만 그 인수를 사용하도록 분리했다. 다른 TypeError를 잡아 재시도하거나 저장 형식을 바꾸지는 않는다. `SequentialWriter.open/write/create_topic/close` 지원도 먼저 확인하며, 미지원 시 암묵적 소멸자 종료로 대체하지 않는다.

Humble 형식은 원문과 같은 생성자 stub으로 검사했고, 실제 설치된 Jazzy에서는 변경 후에도 [system 센서 기록/감사](../../artifacts/validation/2026-10-23/sensor_probe_ros_v6/report.json)가 통과했다. **실제 Jetson의 Humble 판본·ARM 실행은 아직 확인하지 않았다.** [고정 원문·라이선스·해시](../../artifacts/validation/2026-10-23/rosbag_humble_contract_v1/README.md)를 보존했다.

[`prepare_jetson_bundle.py --sensor-probe-tools-only`](../../scripts/prepare_jetson_bundle.py)는 제어기·재생 실행기·드라이버·자격 증명·원시 bag 없이 15파일의 센서 전용 소스 묶음을 만든다. [`verify_sensor_probe_bundle.py`](../../scripts/verify_sensor_probe_bundle.py)는 파일 목록/해시·경로/링크/중복·용량·Python 3.10 문법을 검사하며 압축 해제·실행·전달은 하지 않는다. 사용 절차는 [짧은 휴대 묶음 안내](SENSOR_PROBE_RUNBOOK.md)에 있다.

- 관련 회귀 **174통과**, archive 검증 통과. 원래 repo 경로를 제외하고 새 폴더에 푼 PC Python 3.12 검사도 **100통과**였다. [v2 근거](../../artifacts/validation/2026-10-23/sensor_probe_bundle_v2/README.md).
- 첫 묶음 v1은 불필요한 전체 A2 시험 파일이 제외된 재생기 소스를 읽어 **142통과·1실패**였다. 재생기를 추가하지 않고 센서 전용 시험 목록을 고쳤으며 원래 저장소에서는 전체 A2 시험을 계속 실행한다. [실패 보존](../../artifacts/validation/2026-10-23/sensor_probe_bundle_v1/README.md).
- 별도 전달·설치·Jetson 접속을 하지 않았다. 문법 통과와 PC 독립 실행을 Python 3.10/ARM/Humble 실행 성공이나 실물 승인으로 확대하지 않는다.

### 10.4 저장 입력에서 시계별 간격과 기록 완료 지연 보기

[`analyze_sensor_probe_timing.py`](../../scripts/analyze_sensor_probe_timing.py)는 원래 CDR과 receipt의 전체 무결성 감사가 끝난 경우에만 진단을 저장한다. 이 분석기는 **PC 저장 자료 분석용**으로, 위 v2 최소 기록 묶음에는 포함하지 않았다. 입력·제어·교정값은 바꾸지 않는다.

```bash
python3 scripts/analyze_sensor_probe_timing.py \
  artifacts/hardware_capture/session_name/sensor_recording \
  --output artifacts/hardware_capture/session_name/timing.json
```

- callback 단조 시계, recorder ROS 시계, 원래 header 시계의 간격을 각각 집계한다. 평균 메시지율만 보지 않고 최소/최대/표준편차·같은 시각·역전도 표시한다. header에 0/음수/중복/역전이나 서로 다른 publisher가 있으면 header 메시지율을 제시하지 않는다. 정상적인 간격도 실제 취득 clock의 증거는 아니다.
- `callback_to_writer_receipt`는 **큐·역직렬화·write 호출·해시/통계 갱신 뒤**의 기록 시각까지다. 순수한 큐 대기, 물리 디스크 flush, 센서 취득→수신 지연과 다르다. 서로 다른 clock의 epoch를 직접 빼지 않는다.
- publisher ID가 제공되지 않는 경우를 따로 표시한다. 같은 header가 같은 데이터라는 뜻도 아니며, 수신 기록만으로 생산자/전달 과정의 전체 손실을 확인하지 못한다. 통과/실패용 지연 문턱이나 교정값을 새로 적용하지 않는다.

[관련 161회귀·저장 CDR 2묶음](../../artifacts/validation/2026-10-23/sensor_probe_timing_v1/README.md)이 통과했다. 합성 simulation 기록의 ROS 100 Hz와 실제 callback 약 91 Hz를 구분했고, 의도적으로 잘못된 header에서는 메시지율을 `null`로 남겼다. 새 ROS 실행·Jetson/장치 검증이 아니다. 첫 단품 기록에서는 이 진단과 원천 메타데이터를 함께 보고 필요한 실제 측정을 결정한다.

### 10.5 큰 RGB와 단일 모터축까지 연결한 결과

허용 목록에 있다는 사실만으로 실제 직렬화/기록 성공을 간주하지 않고, 합성 1280×720 RGB/BGR(일부 행 padding 16 B)·CameraInfo·IMU·scan·단일 모터축을 localhost에서 함께 기록했다. 모터축은 28카운트·15:1·65 mm 계약만 사용했고 CameraInfo의 `k[0]=0`은 교정 미완료를 유지한다. 실물/제어기를 실행한 것은 아니다.

| 단계 | 확인한 결과 | 해석 |
|---|---|---|
| 사전 형식 v1/v2 | 각각 115통과·5실패 | 시험기의 재직렬화 padding·Python double 대 wire float32 기대값 오류. wire 타입에 맞춰 수정, 원래 실패 보존 |
| 기본 전송 v7, 회차 대기 0.05 s·drain 0.25 s | Image 17/20, 나머지 각 20/20 | 내부 `complete=true`·무결성과 상류 무손실이 다름. 전체 실패 |
| 낮은 빈도 v8, 0.2 s·2 s | Image 10/20, 나머지 각 20/20 | 빈도/종료 대기만으로 해결되지 않음. 실패 유지 |
| 기록기 없는 SHM 단독 크기 대조 | 기본 크기 선택 4/10, 16 MiB 10/10 | 파일 쓰기 없는 경로에서도 큰 영상 누락. 실제 descriptor는 확인했지만 내부 손실 경로를 완전히 특정한 것은 아님 |
| 명시 SHM 16 MiB v9, 원래 0.05 s·0.25 s | 5토픽 모두 20/20, 관련 138회귀·정상 종료·소스 불변 | 제한 PC 통과. 운영 기본값/전역 설정·QoS·Jetson은 미변경 |

CameraInfo는 별도 프로세스 사이에서 333 B→336 B가 됐다. 기록기를 제외한 한 표본 대조에서도 원래 prefix/모든 필드는 같고 끝에 0 세 바이트만 추가됐다. 따라서 시험용 [`sensor_probe_delivery.py`](../../scripts/sensor_probe_delivery.py)는 송신 prefix 해시/크기 일치와 최대 3 B 끝 정렬 0을 **별도로 표시**한다(18회귀). 임의 padding 삭제·필드 오차 허용을 하지 않고, **recorder callback→MCAP 무결성 검사는 모든 bytes를 그대로 검사**한다. v9의 `full_wire_bytes_equal=false`와 수신/저장 무결성 통과는 서로 다른 경계다.

Jazzy의 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`는 [설치 버전 대응 원문](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/participant.cpp#L185-L214)에서 기존 XML에 SHM/UDP transport를 추가한다. 첫 두 transport 시도는 실제 설정이 기대와 달라 **입력 전 차단**했다. 후속은 해당 자식 환경만 SYSTEM_DEFAULT로 두고, 두 participant가 builtin false·SHM 하나인지 읽어 검사했다. SYSTEM_DEFAULT 자체가 격리라는 뜻은 아니다. 예전 Humble의 검사를 느슨하게 하거나 PC 결과를 Jetson에 소급하지 않았다.

근거: [형식 v1](../../artifacts/validation/2026-10-23/sensor_probe_formats_checks_v1/README.md), [v2](../../artifacts/validation/2026-10-23/sensor_probe_formats_checks_v2/README.md), [ROS v7](../../artifacts/validation/2026-10-23/sensor_probe_ros_v7/README.md), [v8/끝 정렬 진단](../../artifacts/validation/2026-10-23/sensor_probe_ros_v8/README.md), [전송 설정 차단 v1](../../artifacts/validation/2026-10-23/sensor_transport_pc_v1/README.md)·[v2](../../artifacts/validation/2026-10-23/sensor_transport_pc_v2/README.md), [SHM 크기 대조](../../artifacts/validation/2026-10-23/sensor_transport_pc_v3/README.md), [최종 v9](../../artifacts/validation/2026-10-23/sensor_probe_ros_v9/README.md).

이 묶음은 **고속 기록 성능/근본 손실 원인 미확정**을 남기고 마감한다. SHM 단독은 같은 호스트 전용이고 참가하는 생산자 설정도 필요하다. 실제 첫 카메라 기록에서 생산자 순번·촬영 시각·전송 설정을 함께 확인해야 한다. 60 fps·동시 제어·장시간·실물/Humble 성공이 아니며 최소 기록 묶음/일반 실행에 이 설정을 자동 채택하지 않는다.

### 10.6 장치를 열기 전의 환경 사전점검

[`sensor_probe_preflight.py`](../../scripts/sensor_probe_preflight.py)는 ROS 노드/포트/장치/writer 생성 없이 Python·설치 모듈·TopicMetadata API·MCAP reader/writer 등록·선택 센서 메시지의 빈 표본 직렬화·기록 경로/디스크 여유만 확인한다. 부족한 항목을 자동 설치하지 않는다. 선택한 `--output` 보고서 외에는 파일/디렉터리를 만들지 않는다. 기존/외부 capture 경로는 거절하며 실제 기록기도 다시 자체 경로/디스크 검사를 수행한다.

```bash
python3 scripts/sensor_probe_preflight.py \
  --capture artifacts/hardware_capture/session_name/sensor_recording \
  --clock-mode system --sensor-topic /imu/data --sensor-topic /scan \
  --output artifacts/hardware_capture/preflight.json
```

보고서의 `checks_passed`는 설치 API/경로 확인에 한정된다. 실제 MCAP 쓰기·DDS 전달·생산자 QoS/시각·실물 교정·주행 준비는 별도 false다. [관련 74회귀·실제 PC 사전점검](../../artifacts/validation/2026-10-23/sensor_probe_preflight_v1/README.md)은 통과했고 지정 capture 경로는 생성되지 않았다. 휴대 묶음 v2에는 이 후속 진단을 소급 추가하지 않았다. [새 v3 묶음](../../artifacts/validation/2026-10-23/sensor_probe_bundle_v3/README.md)은 관련 249회귀·분리 147회귀와 설치 ROS를 통한 기존 MCAP 파일 감사/진단을 통과했다. 상세 [사용 순서](SENSOR_PROBE_RUNBOOK.md)를 함께 포함하며 전달/Jetson 실행은 하지 않았다.

### 10.7 실제 writer의 쓰기·종료 예외 주입

실제 MCAP writer를 감싼 시험 래퍼에 `OSError`를 주입했다. 첫 [v1](../../artifacts/validation/2026-10-23/sensor_probe_writer_faults_v1/README.md)은 시험 래퍼가 기록기의 반환값을 전달하지 않아 종료 코드 판정에 실패했다. 기록기 본체가 아니라 래퍼를 수정한 [v2](../../artifacts/validation/2026-10-23/sensor_probe_writer_faults_v2/README.md)는 두 조건 모두 통과했다. 세 표본 뒤 쓰기 오류는 3표본, close 완료 직후 오류는 160표본을 보존하고, 모두 `complete=false`·원래 오류·자식 exit 1·감사 거절을 확인했다.

파일이 실제로 닫힌 뒤 예외를 던지고 두 번째 close를 무시하는 시험 래퍼였으므로 실제 디스크 고장/close 멎음/강제 전원 차단을 검증한 것은 아니다. 원시 자료와 실패는 보존한다. 사용자 마감 요청 시점의 코드 검토에서는 정상 close 예외 뒤 `finally`의 재호출, 쓰기 오류 뒤 정리 오류가 추가로 발생할 때의 기록 누락 가능성이 남았다. 다음에는 이 두 경우만 먼저 재현하고 필요하면 예외 보존/한 번 종료를 수정한다. 이미 테스트한 고장 주입 성공을 이 추가 조건에 확대하지 않는다. 60초 join 뒤 writer가 계속 멎은 경우에는 non-daemon 스레드로 인해 프로세스 종료가 보장되지 않으며, 그때 작성한 파일 해시의 안정성도 검증하지 않았다. 이 제한을 숨기거나 파일을 정상 완료로 승인하지 않는다.

**19시 재개 후 확인:** [수정 전 재현](../../artifacts/validation/2026-10-23/writer_cleanup_baseline_v1/README.md)에서 두 문제가 실제로 나타났다. close 예외 뒤 재호출, write+close 예외의 종료 오류 누락이 있었으며 두 경우 모두 원래도 미완료/exit 1이었다. [수정본](../../artifacts/validation/2026-10-23/writer_cleanup_fix_v1/README.md)은 finally 한 곳에서 한 번 종료하고 모든 오류를 보존한다. 관련 262회귀·정상/close/write+close 3조건 통과·처리되지 않은 스레드 예외 0을 확인했다. 종료 멎음/실제 디스크 장애는 이 수정으로 해결됐다고 주장하지 않는다.

### 10.8 저장 센서 입력의 형식 관찰

[`inspect_sensor_probe_inputs.py`](../../scripts/inspect_sensor_probe_inputs.py)는 CDR/receipt 전체 무결성을 확인한 뒤 정적 값과 표시를 읽는다. 영상은 현재 `VisionPipeline`의 RGB/BGR·크기/step/본문 경계를, 모터는 기존 단일 모터축 parser를 대조한다. IMU 유효성/공분산·비유한값, CameraInfo 미교정, scan 메타데이터·무응답/범위 밖 수, 명시 frame 문자열 불일치를 따로 기록한다. 새로운 조향/보호 한도를 만들거나 값을 보정하지 않는다. frame 이름 비교는 장착/좌표축 확인이 아니다.

[관련 279회귀·기존 5토픽 파일 관찰](../../artifacts/validation/2026-10-23/sensor_probe_inputs_v1/README.md)은 각 20표본의 원문을 유지했다. 미교정 CameraInfo·라이다 `+Inf`·합성 frame 불일치를 진단했고 늦은 digest 오류가 있으면 부분 결과를 저장하지 않았다. 첫 실물 입력의 형식 실수를 좁히는 도구이며 동적 운동 이력·시간 계약·단위/교정·관측/주행 적합성을 통과시키지 않는다.

입력 관찰까지 포함한 [휴대 묶음 v4](../../artifacts/validation/2026-10-23/sensor_probe_bundle_v4/README.md)는 종료 수정·사전점검·간격·입력 관찰의 관련 313회귀·분리 206회귀와 설치 ROS 파일 CLI를 통과했다. 기존 15/19파일 목록도 검증하며 원본은 유지한다. 뒤의 세션 감독까지 포함한 최신 전달 후보는 아래 v6이며 실제 전달/Jetson 실행은 하지 않았다.

### 10.9 기록기가 반환하지 않을 때의 선택형 세션 실행

[`sensor_probe_session.py`](../../scripts/sensor_probe_session.py)는 **별도 명시 실행 도구**다. 기존 기록기/launch의 기본 동작이나 주행 보호를 바꾸지 않는다. 미리 발행 중인 허용 센서만 기록하는 자식을 새 프로세스 세션으로 띄우고, 자신의 PID가 적힌 ready 뒤 계획 시간을 재며 정상 SIGINT 종료를 요청한다. 기록 상한에 걸려 멈춘 경우와 계획된 정상 종료는 다르다.

```bash
python3 scripts/sensor_probe_session.py \
  --output artifacts/hardware_capture/new_session \
  --clock-mode system --sensor-topic /imu/data --sensor-topic /scan \
  --duration 20 --max-raw-gib 0.1
```

- 새 `new_session/sensor_recording`에 원시 자료, 상위 `session.json`에 실행 결과를 남긴다. 기존 경로/파일·기록기 manifest를 덮어쓰지 않는다.
- 준비 대기 최대 30 s, 정상 종료 대기 75 s 뒤에도 살아 있으면 자기 자식 그룹만 TERM→5 s→KILL→5 s로 확인한다. 강제 종료·예정보다 이른 종료(코드 0도 포함)·취소는 성공이 아니다.
- 정상 종료 코드만으로 성공하지 않고 완성된 CDR 감사까지 요구한다. 환경/디스크 오류로 파일 감사가 실패하면 세션도 실패이며 교정/주행 허가와는 별개다.
- OS가 프로세스를 종료하지 못하는 상태라면 `process_exited=false`·PID를 남긴다. 감독 프로세스 자체의 전원 상실·native I/O 멎음·전체 시스템 감시는 보장하지 않는다. 강제 종료된 자료를 복구했다고 주장하지 않는다.

[제한 시험](../../artifacts/validation/2026-10-23/sensor_probe_session_v1/README.md)은 합성 센서의 실제 ROS 정상 2초 기록과 close Event 멎음을 대조했다. 두 번째는 시험에서만 종료 대기를 짧게 주었고 실제 파일/디스크 장애가 아니다. 처음 묶음의 134회귀 뒤 CLI의 취소/감사 실패·명시 bool 조건을 추가하여 관련 26개 세션 회귀도 확인했다. 이어 [전체 4,656회귀](../../artifacts/validation/2026-10-23/sensor_probe_integration_v2/README.md)는 모두 통과했다. 별도 [소스·설치본 감사](../../artifacts/validation/2026-10-23/installed_source_v1/README.md)는 기존 161검사와 새 5패키지 빌드·166검사/CTest를 통과했고 기존 설치 누락은 발견하지 않았다. Gazebo/실물 주행 증거와는 다르다.

후속 실행 전 취소·자식 실행 자체 실패에서도 실패 세션 보고서를 남기도록 보완했다. [v5](../../artifacts/validation/2026-10-23/sensor_probe_bundle_v5/README.md)는 이 변경 뒤 기존 가짜 멎음 시험이 시작 전 취소되어 349통과·1실패였고 archive 생성 전 중단했다. fixture를 자식 생성 뒤 취소로 고친 [최신 휴대 v6](../../artifacts/validation/2026-10-23/sensor_probe_bundle_v6/README.md)는 **관련 350·분리 236회귀**, 파일 CLI와 추출본 실제 ROS 2초 세션을 통과했다. 고정 목록 판본은 4(23파일)이며 v4 원본/실패는 보존한다. 실물 드라이버/취득/교정, Humble/Jetson, 디스크 장애 복구를 입증하지 않는다.
