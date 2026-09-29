# A3 Jetson 시험 산출물

해석의 기준은 [기술 보고서](../../../../docs/simulation/JETSON_REPLAY_2026_09_28.md)다.

| 경로 | 내용 |
|---|---|
| `preflight.log` | 설치 전 OS/L4T·전력 모드·자원·ROS 부재 확인. 비관리자 `jetson_clocks --show`/Docker 조회 거부도 보존 |
| `install_humble.log` | 사용자 승인 후 ROS 공식 저장소·Humble 의존성 설치, 제거 0개 |
| `import_check.log`, `cv2_check.log` | ROS/NumPy/OpenCV 기능 확인 |
| `bundle_manifest.json` | 초기 소스/테스트/원시 기록 62개 파일과 비공개 전송 묶음 해시 |
| `prepare_trial.log` | 두 패키지 빌드·88회귀 통과 후 v9 메타데이터 감사 실패 |
| `direct_mcap_probe.log` | MCAP 직접 열기·86,306개 메시지 확인 |
| `prepare_compat_v1.log`, `pc_compat_tests.log` | 원본 보존 호환 경로와 계약 검사 41개, Jetson·PC 결과 |
| `half_initial_remote.log`, `full_initial_remote.log` | 실제 재생 실행·수집·결과 묶음 해시 |
| `regression_final.log` | Jetson 94회귀 통과·자료 의존 2개 제외 |
| `results_initial/` | 0.5배속 1회·1배속 2회 원본 보고서·시각별 진단·자원 기록·소스 ZIP·입력 감사 |
| `comparison_reviewed.json` | 기준 시험을 다시 집계하며 진단 수집 완전성·표본 한계를 명시한 분석본. 원본 결과는 변경하지 않음 |
| `results_improved/` | 후속 수집 스레드 실패 2회·프로세스 수집 기준 2회·최종 영상 최적화 2회와 영상 동일성 비교. 원시 bag 제외 |
| `comparison_observer_v2.json`, `comparison_vision_mask_v1.json` | 수집기 기준선과 영상 최적화 후의 지표·자원 집계 |
| `delivery_correlation_*.json` | IMU 누락과 영상 콜백 시간대 상관 분석. 인과 확정 아님 |
| `observer_v2_startup_failure/`, `pc_observer_v2_tests.log` | 시작 연결 준비 전 60개 누락 재현과 최초 회귀 실패 보존 |
| `pc_observer_v2_ready_tests.log`, `jetson_vision_mask_v2.log`, `pc_improvement_final_tests.log` | 준비 절차 수정·최종 빌드/회귀/재생 검사 |

각 실행에는 `trial.json`, `tegrastats.log`, `resources.jsonl`과 `replay/report.json`, `timing.jsonl`, `source_snapshot.zip`, `delivery_audit.json`이 있다. **`passed=true`는 기능/발행 스케줄이며 입력 전량·진단 전량 전달을 뜻하지 않는다.** 후속 `measurement_passed`는 내부 스냅샷까지 진단 일련번호의 누락/중복 없음, `input_callback_delivery_complete`는 실제 센서 입력 완료 수다. 최종 영상 두 회는 앞의 두 판정은 참, 입력 완전 전달은 거짓이다.

입력 묶음은 `build/jetson_a3_20260928.tar.gz`(Git 제외)에 있으며 SHA-256은 `508371d440486b2708c3eaa73f430323e8a15e85c26f39676d482ecce2cc8524`다. 원격 작업 폴더는 `/home/q/it_arena_a3_20260928`. 초기 감사기는 그 안에 `scripts/audit_sensor_bag.pre_compat.py`로 보존했다. 최종 실행 소스는 각 `source_snapshot.zip`과 `source_sha256`가 기준이다.

회수한 원시 센서 제외 결과 묶음의 SHA-256은 `ef36c8499e9d64be8146ec0ad648afd9672cf3df6deb77a61d874e6a8306f541`이며 원격/로컬 일치를 확인했다. 비밀번호·DPAPI 파일은 이 폴더나 Git에 저장하지 않는다.

최초 세 시험 뒤 추가 개선 전송은 자동 승인 502로 중단됐으나, 이후 사용자 재개 요청으로 접속 복구·후속 검증을 완료했다. `results_initial`은 당시 소스, `results_improved/vision_mask_v1_*`는 현재 채택 소스를 보존한다. 최종 실행의 소스 18개는 현재 로컬 SHA-256과 일치했다.

후속 통합 결과 묶음(`build/results_improved_v1.tar.gz`, Git 제외)의 원격/로컬 SHA-256은 `c65bfea33169dffb6b89814c510ca40d17225e143594495e266bf5df1ac54bcb`다. `results_observer/`, `observer_v1_results/`는 중간에 먼저 회수한 동일 시험 자료다. 실패 기록을 최종 성공으로 덮어쓰지 않았다. 9/29에 마무리했지만 연결된 동일 실험 자료는 이 9/28 묶음에 유지한다.
