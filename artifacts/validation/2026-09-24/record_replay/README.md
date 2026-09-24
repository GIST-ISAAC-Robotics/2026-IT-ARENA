# A2 기록·격리 재생 검증 산출물

해석과 다음 작업: [A2 보고서](../../../../docs/simulation/RECORD_REPLAY_2026_09_24.md).

## 사용 기준

- `capture_v2/report.json`: 12.11848 m 기록 주행·실제 모형 정지·정상 종료 통과. 전체 코스가 아니다.
- `capture_v2_audit.json`: 기록 파일과 CDR/개수/수신 로그 무결성 통과. 센서 송신부터의 완전 무손실 인증은 아니다.
- `replay_half_raw_v1/report.json`: 최종 원시 CDR 재생, 0.5배속 기능/스케줄 통과. 지역 제어기 IMU 15개·RGB 1개 부족.
- `replay_1x_raw_v1/report.json`: 최종 원시 CDR 재생, 1배속 스케줄 통과·**기능 실패**. 지역 제어기 IMU 1,563개·RGB 245개 부족, 안전 명령 최고 0.03 m/s.
- 두 재생의 `delivery_audit.json`: 기록/실제 콜백의 헤더 시각 대조. 노드 내부 완료 횟수와 진단 행 일치를 먼저 확인한다.
- `failure_final/report.json`: 입력 없는 미완료 기록의 제한 종료·보존·재생 거절. 파일을 임의로 완성 처리하지 않는다.
- `regression_verified.log`: 마지막 코드의 전체 Python/ROS 회귀. 초기 빌드는 `build.log`다.

재생은 별도 도메인 `/replay`에서 제어기 두 개만 시작한다. 모터/MCU/Gazebo/정답 입력 없음. 둘 다 EOF 0명령 및 정상 종료를 확인했지만 이를 실제 제동 시험으로 해석하지 않는다.

## 이력 보존

`capture_v1`은 사용자 중단·출발 전 0 m다. `failure_v1/v2` 및 초기 회귀 로그는 당시 결과다. `replay_half_v1`은 그래프 준비 전 접근 실패, `replay_half_v2`는 당시 기능 통과지만 최대 19.97초 지연, `replay_half_v3`는 개선 후 0.5배속 표본이다. `replay_1x_v2`는 회귀 시험과 부하가 겹친 실패이므로 단독 성능값으로 사용하지 않는다. 이후 `*_raw_v1` 두 실행은 재생만 단독 수행했다.

## 원시 자료 보존

`capture_v2/sensor_recording/bag/`·`receipt.jsonl`은 로컬에만 있으며 Git 제외다. MCAP 약 59.5 MB·압축 전 합계 약 3.61 GB다. 다른 PC로 가져갈 때 `sensor_recording` 전체를 별도로 복사하고 무결성 감사를 다시 수행한다. 코드 버전/매개변수는 capture 소스 스냅샷과 재생 보고서 해시를 함께 확인한다. 이전 bag의 계측에는 새 snapshot 서비스/순번이 없으므로 원래 기록과 재생 계측 버전을 구분한다.
