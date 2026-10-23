# UART bring-up 검사 앱 대상 빌드 v2 — 빌드만

[v1](../actuation_uart_bench_idf_v1/README.md)의 기호 기대값(`uart_set_pin` → v6.1 실제 기호 `_uart_set_pin6`)만 고친 뒤 새 빌드 폴더에서 다시 빌드했다. **build exit 0·빌드 전후 소스 해시 차이 0 B**다. 앞서 스크립트를 읽는 관련 검사 6개도 수정본에서 통과했다([로그](related_tests.log)).

- ESP-IDF v6.1·Xtensa 15.2.0, ESP32-S3. ELF 4,149,468 B, BIN 216,144 B. 정적 DIRAM 105,022 / 341,760 B, IRAM 16,384 / 16,384 B(SDK 구성 값).
- 기본 계획(`enabled=false`)에서 실행되지 않는 소유 작업 경로가 지워지지 않고 링크됐다: `uart_driver_install`, `uart_param_config`, `_uart_set_pin6`, `uart_read_bytes`, `uart_tx_chars`, `uart_flush_input`([기호](linked_uart_symbols.txt)). 링크는 실행·배선·타이밍 증거가 아니다.
- `-fstack-usage` 함수별 정적 프레임 최대 496 B(`emit_ack`), 대상 `run_cycle` 304 B, 공유 `uart_case` 384 B([`.su`](main_cpp_stack_usage.su)). 호출 사슬 합·SDK/ISR 사용량을 포함하지 않으므로 소유 작업 stack 6,144 B나 main stack 8,192 B의 충분성 증거가 아니다.
- `CONFIG_FREERTOS_HZ=1000`은 이 검사 앱의 1 ms 소유 주기용 값이다. 콘솔은 SDK 기본(UART0, 보조 USB-Serial-JTAG)으로 두었다.
- `ESP_ROM_ELF_DIR` 미정의 진단 2건과 `nm -u`의 `__cxx_fatal_exception`, `start_app`, `start_app_other_cores`는 기존 검사 앱과 같이 보존한다. 보드 부팅 검증이 아니다.
- flash/monitor·장치 조회/업로드·PWM/DIR·센서·모터·Jetson 작업은 하지 않았다. `hardware_executed=false`, `firmware_verified=false`, `ready_to_drive=false`.

명령: [`run.sh`](run.sh) → `pytest tests/test_actuation_uart_bench.py` → `bash scripts/build_actuation_bench.sh artifacts/validation/2026-10-23/actuation_uart_bench_idf_v2 /home/jinhyeong/.cache/it-arena/idf-6.1-smoke/bench-source-v2`. [빌드 로그](build.log), [크기](idf_size.txt), [미해결 기호](undefined_symbols.txt), [종료값](build_exit.txt), 소스/바이너리 해시와 스냅샷을 같은 폴더에 보관한다.

**후속 검토(같은 날):** 이 빌드의 소스는 이후 계약 불일치 3건이 발견된 판본이다. 링크 결과는 그 소스의 결과로만 보존하며, 수정본 대상 빌드는 [v4](../actuation_uart_bench_idf_v4/README.md)를 따른다.
