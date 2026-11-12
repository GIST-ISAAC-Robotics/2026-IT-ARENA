# 실제 MCU main 스택 초과 확인 r301

r300 로그 앱을 UART 초기화 뒤의 921600 baud로 8초 수신했다. `A stack overflow in task main has been detected.`가 반복되어 r298의 프레임0/재부팅 원인을 확인했다. 센서 고장이나 CRC 문제로 분류하지 않는다. [진단](boot_diagnosis.json).

기존 main 스택은 3,584 B, app_main 컴파일 스택 프레임은 2,688 B였다. 중첩 호출 여유가 작았다. [r302](../runtime_stack_r302_v1/README.md)는 단일 task의 큰 임시 버퍼를 정적 저장소로 옮기고 8,192 B 스택·실행 중 최저 여유 계측으로 검증한다. 컴파일 성공만으로 실제 동시 실행을 통과 처리하지 않는다.

시험 후 원 factory app 1 MiB를 해시 일치로 복원하고 inactive 응답·포트 비점유를 확인했다. 출력은 항상 불허였고 호스트 UART 송신0이다. [회수](recovery.json)·[원 결과](actual/result.json). 로그 원문·Flash 백업은 Git 제외 보존이다.
