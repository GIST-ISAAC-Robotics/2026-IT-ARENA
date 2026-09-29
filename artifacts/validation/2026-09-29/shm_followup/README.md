# IMU 발행 경계 계측

기준 해석은 [A3 보고서](../../../../docs/simulation/JETSON_REPLAY_2026_09_28.md#shm-반복-검증과-imu-누락-분리--929-추가)에 둔다. 발행→콜백 경계 계측과 DDS 내부의 실제 도착/큐 손실 계측을 구분한다.

## 적용 범위

- `trace_tools_manifest_v1.json`: `scripts/replay_sensor_bag.py`와 `scripts/run_jetson_replay_trial.py` **2개만** 포함하는 소형 묶음. archive 12,800 bytes, SHA-256 `3a7db7cf6c44f9130dbeb073207314ab299fe59f836a9de4f946df51b8bd6b31`.
- `--trace-imu-publication`: 기본 비활성. IMU header만 읽어 원래 시각, 재생 발행 시작/끝의 단조 시각과 스케줄 지연을 `imu_publication.jsonl`에 저장한다. 발행 CDR·주기·QoS·보호 기준은 바꾸지 않는다. 추가 역직렬화/계측 비용이 있으므로 비계측 결과와 무조건 같은 조건이라고 보지 않는다.
- `analysis_source_v1.zip`: 분석 도구·보조 지연 분석 함수·단위 검사의 소스 보존본. 현재 IMU 분석 단위 검사 9개 통과. 음수 발행 종료→콜백 간격도 0으로 숨기지 않는다.

`scripts/analyze_imu_delivery.py`는 진단 완전성·원본 개수·취득 시각의 유일성을 검증한다. 발행 추적이 있을 때에는 1 ms 미만 간격의 연속 발행 묶음과 누락의 상관을 보조 자료로 제시한다. 이 문턱은 **분석용**이며 제어기가 데이터를 허용하는 시간 기준이 아니다. 보호층의 콜백 시각도 DDS 내부 도착 시각을 대신하지 않는다.

## 실행·회수

- 기존 원격 2파일은 `/home/q/backup_imu_trace_v1_20260929.tar.gz`에 보존·검증한 뒤 적용했다. 제어기·원시 기록·QoS는 미변경.
- `jetson_trace_tools_tests_v1.log`: Jetson 관련 회귀 54개 통과. `jetson_qos_profile_v1.log`: 기존 sensor-data QoS의 depth 5·BEST_EFFORT·KEEP_LAST 읽기 전용 확인.
- `imu_trace_v1_{a,b}.log`, `jetson_results/imu_trace_v1_{a,b}/`: 같은 20.968초 입력, SHM 16 MiB·legacy·엔코더/IMU 발행 추적 ON. 각 4개 participant 실제 설정 확인.
- `comparison_v1.json`: 기능/운영/계측/종료·입력/엔코더·자원 집계. 모두 통과지만 IMU 일부 누락은 남는다.
- `imu_analysis_v1.json`, `imu_analysis_v2.json`: 원시 콜백과 IMU 발행 추적 대조. v2는 연속 IMU 콜백 사이 발행 6개 이상인 구간을 추가 집계했고 원자료/실행 소스는 바꾸지 않았다. 대응 분석 소스는 `analysis_source_v1.zip`, `analysis_source_v2.zip`에 보존. 최종 분석 단위 검사 10개 통과.
- 회수 archive 52멤버·원시 bag 0개, SHA-256 `4a4a2dc9d2840900e9cbfd9e28349779a24657a41e239c1d0261e1c9dd75b8dc` 원격/로컬 일치. Git 제외 `build/`에 원본 archive 보존.

두 회 각 IMU 발행 추적 5,242행과 원본 bag에서 직접 추출한 header 시각 배열/순서가 일치했다. 실행기·제어기 정상 종료, EOF 0명령, 관련 잔존 프로세스 없음 확인. 기본 system/legacy는 유지하며 장시간·실차 검증으로 확대하지 않는다.
