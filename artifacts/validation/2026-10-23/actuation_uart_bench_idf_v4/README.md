# UART bring-up 검사 앱 대상 빌드 v4 — 검토 수정 최종본, 빌드만

**build exit 0·빌드 전후 소스 해시 차이 0 B**이며, 빌드 입력은 [검토 수정 v2](../actuation_uart_bench_review_v2/README.md) 실행 뒤의 소스와 같다.

- ESP-IDF v6.1·Xtensa 15.2.0, ESP32-S3. ELF 4,176,568 B, BIN 219,904 B. 정적 DIRAM 105,182 / 341,760 B, IRAM 16,384 / 16,384 B(SDK 구성 값).
- 기본 계획(비활성)에서도 실행되지 않는 소유 작업 경로가 링크됐다: `uart_driver_install`, `uart_param_config`, `_uart_set_pin6`, `uart_read_bytes`, `uart_tx_chars`, `uart_flush_input`([기호](linked_uart_symbols.txt)). 링크는 실행·배선·타이밍 증거가 아니다.
- 함수별 정적 프레임 2 KiB 초과 0건([목록](large_stack_frames.txt) 비어 있음). 최대 496 B(`emit_ack`), `UartScratch::make` 208 B. 호출 사슬 합·SDK/ISR 사용량이 빠져 있어 실제 stack 충분성 증거가 아니다.
- 앱은 합성 자기 검사 실패 시 외부 I/O를 시작하지 않으며 기본 계획은 비활성이다. `CONFIG_FREERTOS_HZ=1000`은 검사값이다.
- `ESP_ROM_ELF_DIR` 미정의 진단 2건과 `nm -u`의 `__cxx_fatal_exception`, `start_app`, `start_app_other_cores`를 보존한다. 보드 부팅 검증이 아니다.
- flash/monitor·장치 조회/업로드·PWM/DIR·센서·모터·Jetson 작업은 하지 않았다. `hardware_executed=false`, `firmware_verified=false`, `ready_to_drive=false`.

명령: `bash scripts/build_actuation_bench.sh artifacts/validation/2026-10-23/actuation_uart_bench_idf_v4 /home/jinhyeong/.cache/it-arena/idf-6.1-smoke/bench-source-v4`. [빌드 로그](build.log), [크기](idf_size.txt), [미해결 기호](undefined_symbols.txt), [종료값](build_exit.txt), 소스/바이너리 해시와 스냅샷을 같은 폴더에 보관한다.

**2차 검토(같은 날):** 이 빌드의 소스에는 관측된 pending 경쟁이 남아 있었다. 수정본 대상 빌드는 [v5](../actuation_uart_bench_idf_v5/README.md)다.
