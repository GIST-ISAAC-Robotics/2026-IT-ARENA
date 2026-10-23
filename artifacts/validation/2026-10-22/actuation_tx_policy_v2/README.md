# A1 유한 송신 정책 후보 — v2

구동 출력 없는 C++ 후보의 고정 입력·선로 시간 모델 검사다. 실제 UART/USB·보드·ROS 노드·모터를 실행하지 않았다. 운영 Python/ROS 경로는 바꾸지 않았다.

## 결과

- `checks_exit.txt`: 0. `source_hash_diff.txt`: 0 B.
- `related_tests.log`: 132 passed. `full_tests.log`: 3,736 passed, 19 skipped. 기존 portable 사전 빌드 바이너리 환경변수가 지정되지 않은 19개는 미실행이며 통과로 세지 않는다.
- `mutants.log`와 `mutants/`: 12개 변이 모두 검출. v1의 STOP ACK 혼합 순서 검사 누락을 보강했다. [v1 실패](../actuation_tx_policy_v1/README.md)는 보존했다.
- `audit/report.json`: 위반 0. 460800 baud·8N1 합성 선로에서 정상 DRIVE 50 Hz의 MCU→host 발생량은 65,818 → 17,662 B/s, offer 100 → 50회/s. 이전 명령 반복은 원래와 같이 `command_expired` 고정으로 끝난다.
- 목표 SDK 링크는 [별도 v5](../actuation_idf_smoke_v5/README.md)이며 이 호스트 실행 결과와 구분한다.

## 재현과 경계

명령·대상 파일 목록은 `run_checks.sh`, 시험 당시 내용은 `source_snapshot.tar.gz`, 전후 해시는 `sources_before.sha256`·`sources_after.sha256`다. 이미 완료된 증거 폴더를 덮어쓰지 않는다. 실행 후 GPT/GPT-6 Luna가 로그·해시·SDK 명령을 별도 대조했으며, 이후 ARM 관문 변경의 결과를 이 v2에 소급하지 않는다.

FIFO 128 B·pump 250 µs·공유 합성 시계·출력 준비 참은 가정이다. 115200에서는 첫 DRIVE 도착 전 만료 고장, 230400에서는 상세 진단 기아가 남는다. 최악 문자열·ACK 폭주·실제 FIFO/USB 지연에 대한 실차 운용 보장이 아니다. `firmware_verified=false`, `ready_to_drive=false`.

정책·수치·한계의 기준 문서는 [구동 계약 §19](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#19-1022-후속--구동-출력-없는-송신-정책-후보)다.
