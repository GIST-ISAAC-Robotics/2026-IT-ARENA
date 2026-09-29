# 영상 인식 프로세스 분리 검증 — 2026-09-29

판정·한계의 기준 문서는 [A3 보고서](../../../../docs/simulation/JETSON_REPLAY_2026_09_28.md#영상-인식-프로세스-분리--929-후속)다. 아래 파일을 같은 기록의 무손실/실시간/실차 성공으로 확대하지 않는다.

- `pc_first_tests.log`: 초기 선택 회귀 59개. `pc_final_tests.log`: 제어 함수 정지 경계 추가 후 62개. `ssh_transport_tests.log`: 기존 키 검증을 유지하는 주소 선택 4개.
- `pc_equivalence_v1.json`, `jetson_results/vision_process_equivalence_v1.json`: 같은 영상 218장의 순차 판정 비교.
- `pc_process_v1/`: 초기 PC 1배속·전달 감사·소스. Python 보고서 통과와 WSL wrapper 종료 코드 1의 불일치는 미확인으로 보존한다.
- `pc_closed_loop_v1/`: 계측 비활성 12 m 가상 MCU 주행·정지·종료와 소스. 전체 코스 시험 아님.
- `jetson_process_v1.log`, `jetson_process_cd_v1.log`, `jetson_results/vision_process_v1_{a,b,c,d}/`: Jetson 실행·계측·전달 감사·소스 보존본.
- `jetson_comparison_final_v1.json`: 4회 최종 집계. `active_window_no_zero_commands`는 A/B 참, C/D 거짓이다. `jetson_comparison_v1.json`과 `jetson_comparison_all_v1.json`은 중간 집계로 보존한다.
- `stop_evidence_v1.json`: C/D의 보호 정지 시각과 엔코더 지연 표본. 원시 근거는 각 실행의 `replay/timing.jsonl`, `report.json`, 제어기 로그다. DDS/수신 큐/executor 중 원인 분리는 미완료다.
- `bundle_v1_manifest.json`: 기존 원격 기록을 제외한 소스 64개 파일 전송 묶음. Jetson 설치 사본은 재빌드했다.

## 보존·무결성

원시 bag은 기존 `2026-09-24/record_replay/capture_v2/sensor_recording`에만 두며 기존 Git 제외 규칙을 유지한다. 이번 결과 묶음에는 원시 bag을 넣지 않았다. 결과/소스 스냅샷과 과거 실패는 덮어쓰지 않았다.

| Git 제외 묶음 (`build/`) | SHA-256 |
|---|---|
| `jetson_vision_process_v1.tar.gz` | `4a0e16458f8851241c59af83326aee8f2e12e8a38ee7e9204e170a864abf9609` |
| `jetson_vision_process_results_v1.tar.gz` | `c9dd9fa6934e273df68a260a68980c3482705f13ef3973cf3db16b71afb1d7a2` |
| `vision_process_cd_results_v1.tar.gz` | `2226c29d6e211b5795be1c521a49f75094e9a4e5b126d42750ec4fb7a5e5e9dc` |

결과 두 묶음은 원격/로컬 해시가 일치했다. 시험 종료 후 해당 재생기·제어기·영상 작업·수집기·계측 프로세스가 남지 않았음을 확인했다. Jetson과 PC 전원은 유지하며 GitHub 게시하지 않았다.
