# MCU 부팅 로그 분리 r300

r298과 같은 비구동 코드를 INFO/UART 로그 빌드로 만들고 115200 baud에서 8초 수신했다. ROM의 소프트웨어 리셋 반복은 확인했지만 앱이 UART0을 921600으로 바꾸므로 이후 원인은 읽을 수 없었다. 같은 앱을 921600으로 수신한 [r301](../runtime_boot_uart_r301_v1/README.md)에서 원인을 확인했다.

시험 전 factory app 1 MiB를 백업하고 시험 후 바이트 해시가 일치하도록 복원했다. 원 IMU 앱의 inactive 응답·포트 비점유를 확인했다. host 송신/ARM/모터 구동은 없었다. [회수 근거](recovery.json)·[원 결과](actual/result.json). 실제 UART 원문·Flash 백업은 Git 제외 자료로 보존한다.
