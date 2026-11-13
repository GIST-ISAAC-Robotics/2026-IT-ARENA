# r387 운영 출력 HAL 연결 후보

2026-11-13. **미교정 진단 오류 수정 후 결합 검사 11개와 ESP32-S3 재빌드가 통과했다. 장치 실행은 없다.** [새 앱](../../../../src/arena_vehicle_interface/firmware_runtime_drive/README.md)은 기존 수신기·전진 PI·CAD/영점·서보 순서기를 실제 GPIO/UART/PCNT/LEDC 호출에 연결한다. 기본 프로필은 미교정으로 출력 불가이고 장치에 설치하지 않는다.

원 비구동 preview의 serial/events/encoder/pwm_zero 함수 본문은 `runtime_peripherals.hpp`로 그대로 옮겨 두 앱이 공유한다. 기존 preview는 계속 준비값false·읽기 전용 서보이며 운영 앱으로 자동 전환되지 않는다. 변경 원문·검사·빌드와 독립 리뷰를 이 폴더에 보존한다.


## 최초 빌드와 결합 검사

`build_v1/`은 ESP-IDF6.1 빌드 exit0, bin299,472 bytes, app partition 여유71%와 입력 소스 불변을 확인했다. ROM ELF 경로가 없는 환경에서 gdbinit 생성 경고/traceback이 두 번 있었으며 바이너리 빌드 실패와 구분한다. 전력 설정·도구체인 설치·보드 flash는 하지 않았다. 이후 추가한 CAD 명령 한계 검증과 아래 진단 수정의 빌드는 별도다.

독립 검토에서 명령 만료의 ACK 처리 순서, host UART 실패 뒤 서보 송신, ki=0 설정 불일치, CAD 범위 초과의 조용한 제한을 찾아 수정했다. 최초 실행(`review/verify_v1/`)은 결합 사례1개 실패·나머지10개와 Python 계약58개 통과다. 실패는 미교정 운영 구성으로 receiver 자체도 초기화하지 못해 상태 송신이 중단되는 문제였다.

운영 구성이 무효일 때 진단에만 사용할 유효 wire 구성을 명시적으로 분리했다. 실제 운영 유효성 판정은 원 설정을 그대로 검사하며, 이 진단 구성으로 ready·ARM·속도 피드백·서보 목표를 허가하지 않는다. 원 실패 소스는 `before_diagnostic_fix.zip`, 원 manifest는 `review/manifest_v1.json`에 보존했다. 원 검사 실패를 통과로 고쳐 쓰지 않는다. 수정 후 결합 검사와 새 빌드의 결과는 후속 기록을 따른다.

## 수정 후 결과와 범위

`review/verify_v2/pytest.log`에서 결합 검사 11개 통과를 확인했다. 기본 프로필은 진단 송신 가능·준비 미완료·ARM 거절·서보 목표/PWM 출력0을 함께 확인한다. 앞선 Python 계약 58개는 변경이 없어 다시 실행하지 않았다. 가짜 UART 하네스는 실제 수신기·순서기·송신기·매핑을 결합하지만 IDF `app_main`과 물리 주변장치를 실행하지 않는다.

`build_v2/` 재빌드는 exit0, bin299,712 bytes, app partition 여유71%다. 바이너리 SHA256은 `de636d025da6ea55e867cc6357cdf344966b382f6d69dc1a0dc2343c2d0be2f9`이다. 코드 입력 해시와 자체 `source_diff.txt`는 변하지 않았지만 전체 목록에는 빌드 중 README 설명 수정 1건이 있다. 따라서 **전체 파일 동결 불변으로 판정하지 않는다.** `source_before.sha256`/`source_after.sha256`에 문서 변경을 보존했으며 이 차이를 감추기 위한 재빌드는 하지 않았다. ROM ELF/gdbinit 경고는 남아 있고 빌드 성공과 구분한다.

운영 프로필·실장 영점·방향·출력 상한·PI 값은 조립 후 확인해야 한다. 실제 UART/PCNT 동시 주기, 출력 파형, 조향 부하, 장착 STOP 및 독립 차단은 미검증이다. 이 단계는 무구동 연결 준비를 마친 것으로 보고 최종 차량 기하와 Jetson 계산 지연 작업으로 이어간다.
