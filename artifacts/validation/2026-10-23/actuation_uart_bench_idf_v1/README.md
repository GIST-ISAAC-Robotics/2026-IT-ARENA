# UART bring-up 검사 앱 대상 빌드 v1 — 빌드 스크립트 기호 검사 실패

`build_exit.txt`는 **1**이다. ESP-IDF v6.1·Xtensa 15.2.0 링크와 ELF/BIN 생성(`Project build complete`)까지는 진행됐고 빌드 전후 소스 해시 차이는 0 B다. 실패는 마지막 확인 단계의 `linked_uart_symbols.txt`가 기대 6줄이 아닌 **5줄**이어서 발생했다.

- 원인: v6.1 `driver/uart.h`에서 `uart_set_pin`은 인수 개수로 고르는 매크로이며, 4핀 호출은 `_uart_set_pin6`으로 링크된다. 스크립트가 존재하지 않는 `uart_set_pin` 기호를 기대했다. `symbols.txt`에 `_uart_set_pin6`이 있다. 앱/어댑터 코드 결함이 아니라 검사 스크립트의 기대값 오류다.
- 실행 보조(Haiku 5.5 별칭, 자기 보고)의 래퍼는 종료값 0을 보고했으나 같은 `set -e`·EXIT trap 구조의 재현에서는 래퍼도 1을 반환했다. 래퍼 값은 확인할 수 없어 근거로 쓰지 않고 스크립트가 기록한 `build_exit.txt`=1을 판정으로 둔다.
- 참고 수치(실패 판정과 별개): BIN 216,144 B, 정적 DIRAM 105,022 / 341,760 B. `ESP_ROM_ELF_DIR` 미정의 진단 2건, `nm -u` SDK 기호 3개(`__cxx_fatal_exception`, `start_app`, `start_app_other_cores`).

수정은 기대 기호를 `_uart_set_pin6`으로 바꾼 스크립트 한 줄이며 v2에서 새 빌드 폴더로 다시 확인한다. 이 폴더의 산출물·로그는 실패 근거로 보존한다. flash/monitor·장치 조회는 하지 않았다.
