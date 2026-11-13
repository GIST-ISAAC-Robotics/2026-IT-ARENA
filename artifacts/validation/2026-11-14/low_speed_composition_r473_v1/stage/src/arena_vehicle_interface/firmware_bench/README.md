# 출력 비활성 UART bring-up 검사 앱

[`ActuationBenchEndpoint`](../portable/actuation_bench.hpp)를 한 소유 작업에서 유한 수신·송신·제어 tick으로 돌리는 [`UartBenchAdapter`](../portable/actuation_uart_bench.hpp)의 **ESP-IDF 연결과 목표 컴파일 확인용 앱**이다. 구동 펌웨어가 아니며 모터 PWM/DIR·서보·PCNT·IMU·독립 출력 감시가 없다. 업로드/모니터 스크립트를 제공하지 않는다.

## 기본 동작 — 외부 I/O 비활성

- `arena_bench_protocol_plan{}`의 기본값은 `enabled=false`다. `app_main`은 공유 가짜 I/O 사례만 실행하고 진단 JSON을 콘솔에 한 줄 쓴 뒤 반환한다. 이 경우 UART 드라이버 설치·핀 설정·소유 작업 생성이 없다.
- 합성 자기 검사(`self_check_ok`)가 하나라도 실패하면 계획이 준비돼 있어도 외부 I/O를 시작하지 않는다. 자기 검사 통과도 실물 동작 확인은 아니다.
- 계획을 켜려면 실물 판본·배선·상대 장치 확인(`wiring_confirmed`), 포트/TX/RX/baud 명시, 진단 콘솔과 다른 포트·핀, ESP32-S3 직접 USB(19/20)·flash/PSRAM(26~37)·strapping(0/3/45/46) 회피, 다른 기능 배정 핀과의 충돌 없음이 모두 필요하다. UART0는 ROM/부트로더 출력 침묵을 별도로 확인해야 한다. 이 조건을 통과해도 전기적 적합성 확인은 아니다.
- 기존 [CH343P→UART0 우선안](../../../docs/hardware/ESP32_BUDGET_REVIEW_2026_10_02.md#usb-연결-검토---102-후속)은 현재 기본 진단 콘솔과 같은 포트다. 프로토콜과 콘솔을 섞지 않으므로 이 앱은 그 경로를 자동 선택하지 않는다. 콘솔 이동/부팅 출력 처리·실제 납품 판본 대조는 남은 결정이다.

## 구성

- `main/uart_bench_fakes.hpp`: 가짜 I/O·합성 시각·사례 도구. 이전 어댑터 판본으로도 컴파일되도록 공통 API만 써서 [결함 재현기](../../../scripts/uart_bench_defect_repro.cpp)가 공유한다. `firmware_smoke/main`의 `Report`, 단일 모터축 28카운트·15:1·65 mm 구성, 송신 시험값을 재사용한다.
- `main/uart_bench_cases.hpp`: 호스트와 대상이 공유하는 33개 반례. 계획 거절·I/O 비활성·tick 독립/따라잡기 없음·ARM/DRIVE 거절·STOP 뒤 CLEAR 거절·수신 바이트/시간 상한·사건 범람/포화(진입 때 정적 포화, 꺼내는 중 재포화, 볼 수 없는 순간 포화의 한계 고정)·주기 전체 사건 예산·예산 뒤 숨은 오류·읽은 뒤 미확인 조각 폐기(빈 큐 응답 직후 도착해 마지막 조회에 보인 사건 포함)·늦은 오류·넘침/프레임 오류/break의 부분 프레임·송신 꼬리 폐기·송신 0/부분/오류/과대 보고·수신 오류·시각 역행(종료 읽기 포함)·마지막 작업까지 포함한 주기 실행시간·무수신을 단절로 보지 않음·조립 기한·수신 시각 위치.
- `main/esp_uart_io.hpp`: v6.1 `uart_driver_install`(TX ring 0)·`uart_read_bytes(..., 0)`·`uart_tx_chars`·이벤트 큐 `xQueueReceive(..., 0)`·`xQueueReset`+`uart_flush_input`. 계획이 `kReady`가 아니면 열지 않는다. 바이트가 ring에 들어오는 시점·사건과 바이트의 비동기 순서·큐 포화 때 사건 버림은 로컬 SDK 소스 해석이며 하드웨어에서 확인하지 않았다.
- `main/main.cpp`: 계획 확인 뒤에만 소유 작업을 만든다. 작업은 `xTaskDelayUntil` 1 tick 주기이며 늦으면 기한을 현재로 다시 잡고 한 tick 양보한다(연속 실행으로 따라잡지 않음). 고장 뒤에는 UART를 더 다루지 않고 콘솔에 한 줄만 남긴다. 부팅별 boot ID는 `esp_random` 16진 16자이며 인증 수단이 아니다.
- `sdkconfig.defaults`: ESP32-S3, `CONFIG_FREERTOS_HZ=1000`(1 ms 소유 주기 검사값), 예외/RTTI/PSRAM 비사용. 콘솔 설정은 바꾸지 않는다. 운영 tick률·flash/PSRAM 구성 확정이 아니다.

검사값: 제어 간격 5 ms(수신기 `max_control_gap_us` 50 ms 미만), 주기 수신 상한 1024 B·새 읽기 시작 시간 상한 0.5 ms(주기 전체 마감 아님), 사건 16개/주기·읽기 뒤 확인 예약 4·큐 32, RX ring 4096 B, offer 20 ms, 조립 기한 30 ms. 모두 실측 전 시험값이며 운영 baud·timeout을 정하지 않는다.

## 빌드·실행 범위

`scripts/build_actuation_bench.sh <새 증거 폴더> <공백 없는 새 빌드 폴더>`는 `idf.py build/size`만 수행하고 SDK UART 함수 6개가 링크됐는지 기호표로 확인한다. 링크 확인은 실행 증거가 아니다. 호스트 검사는 [`actuation_uart_bench_host.cpp`](../../../scripts/actuation_uart_bench_host.cpp)·[`uart_bench_defect_repro.cpp`](../../../scripts/uart_bench_defect_repro.cpp)와 [`test_actuation_uart_bench.py`](../../../tests/test_actuation_uart_bench.py), 변이 검사는 [`check_uart_bench_mutants.py`](../../../scripts/check_uart_bench_mutants.py)를 사용한다. 결과와 한계는 [구동 계약 §29](../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#29-1023-후속--출력-비활성-uart-주기-어댑터)를 따른다. `firmware_verified=false`, `ready_to_drive=false`.
