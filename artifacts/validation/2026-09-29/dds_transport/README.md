# DDS 전달 방식 대조 근거

상세 조건·판정은 [A3 보고서](../../../../docs/simulation/JETSON_REPLAY_2026_09_28.md#dds-전달-방식-대조--929-후속)에 한 곳으로 정리한다. 짧은 개방루프 재생이며 실차 주행이나 센서 드라이버 검증이 아니다.

## 준비·검사

- `preflight_v1.log`, `preflight_v1_detail.log`: 설치된 Fast DDS 개발 헤더/라이브러리·도구·프로세스 확인. 처음의 넓은 프로세스 검색에 잡힌 커널 이름을 ROS 잔존 프로세스로 해석하지 않는다.
- `probe_build_v1.log`: 첫 C++ 컴파일의 namespace 모호성 오류. 이후 namespace를 명시해 수정했다.
- `bundle_manifest_v1.json`: 전송만 했고 적용하지 않은 전체 72개 파일 묶음. 자동 승인 검토에서 적용 범위가 거절되어 시험에 사용하지 않았다.
- `tools_manifest_v2.json`: 사용자 재승인 후 실제 적용한 **재생·검사 6파일**. archive 21,133 bytes, SHA-256 `81a8afa62d705a768585f367c91749150fba40a5d4d882e772d5298c099e56bc`. 기존 4개 파일은 Jetson의 `/home/q/backup_dds_tools_v2_20260929_01/`에 보존하고 해시를 대조했다. 제어기 소스·설치본과 원시 기록은 변경하지 않았다.
- `jetson_tools_compile_v2.log`, `jetson_tools_tests_v2.log`: ARM64 probe 컴파일 및 신규 검사 7개 통과.
- `pc_regression_v1.log`, `pc_system_v1/`: PC/Jazzy 관련 회귀 79개 및 기본 옵션 1배속 재생. DDS 선택/감사 옵션 없이 실행했으며 기능·스케줄·계측·운영·EOF 0명령·정상 종료 통과. PC의 새 transport 검증으로 확대하지 않는다.

## 기본 방식 / UDP 단독

- `system_a_v2.log`, `udp_a_v2.log`, `udp_b_v2.log`, `system_b_v2.log`: 이 순서로 실행한 Jetson 시험 로그.
- `jetson_results/dds_transport_v2_{system_a,udp_a,udp_b,system_b}/`: 시험별 소스/매개변수 스냅샷, 활성 participant 감사 4개, 발행/콜백 시각, 보고서·자원 로그.
- `comparison_v2.json`: 위 4회의 전달·명령·자원 집계. `scripts/summarize_dds_trials.py`로 원자료를 읽어 생성했으며 이전 결과는 덮어쓰지 않는다.
- 회수 archive SHA-256 `57511d6b0605702455d5305468706ba0403b95a32a1e64e2d76218c0a8b3f44e` 원격/로컬 일치. 멤버 98개, 원시 bag 0개. archive는 Git 제외 `build/`에 보존한다.

네 회의 소스 해시가 같고 각 participant의 설정은 실제 요청과 일치했다. 기본 재생/종료 통과와 운영 실패를 구분한다. UDP 단독은 두 번 모두 지연·보호 정지가 남아 채택하지 않았다. 원시 센서 bag·수신 기록의 Git 제외 정책과 과거 실패 자료를 유지한다.

## SHM 단독 용량 대조

- `shm_default_a_v2.log`, `shm16_a_v2.log`, `shm16_b_v2.log`, `shm_default_b_v2.log`: 이 순서로 실행. 위 시험과 같은 소스/기록, 변경된 제어기·QoS 없음.
- `jetson_results/dds_transport_v2_{shm_default_a,shm16_a,shm16_b,shm_default_b}/`: 각각 활성 participant 감사 4개, 소스/매개변수·보고서·상세 계측·자원 로그.
- `shm_comparison_v2.json`: 위 4회의 집계. 기본 SHM B는 운영 실패, 16 MiB 두 회는 운영 통과지만 IMU 일부 누락이 남는다.
- 별도 회수 archive SHA-256 `12614e979e590ea5d61050c0010cdd538a6c8671bde975636947a732ae78d540` 원격/로컬 일치, 원시 bag 0개. 기존 기본/UDP 결과는 덮어쓰지 않았다.

전체 8회는 소스 스냅샷 해시 동일, 기능/스케줄/계측/EOF 정지/정상 종료 및 잔존 프로세스 없음 확인. **16 MiB는 후속 검증 후보이며 기본값으로 전환하지 않았다.**
