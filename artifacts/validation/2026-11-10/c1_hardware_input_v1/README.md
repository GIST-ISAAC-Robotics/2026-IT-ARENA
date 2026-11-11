# C1 실물 입력 검증 산출물

2026-11-10, 실제 Jetson·USB C1 시험. 조건·원문 판본·실패·시뮬 차이와 후속 명세는 [기준 보고서](../../../../docs/sensors/C1_HARDWARE_2026_11_10.md) 한 곳에 둔다.

- [native_standard_summary.json](native_standard_summary.json): 정렬 전 Standard 10초·102스캔, 원시 순서·점 수·각도·sync·SDK 추정 시각.
- [ros_summary.json](ros_summary.json): 기본 true 5분·3,003스캔과 false 20초·201스캔. 201.028 ms 공백1회·정상 종료 포함.
- [native_dense_summary.json](native_dense_summary.json): DenseBoost20초·202스캔과 장치 모드 metadata. 정확도 개선 판정 아님.
- [recorded_vendor_replay_summary.json](recorded_vendor_replay_summary.json): 실제 원시102스캔을 고정 원문 함수에 넣은 타입/전송 stub 재생. 표시 칸 재사용189~209개를 원본 ID로 추적.
- [sdk_motor_query_failure.txt](sdk_motor_query_failure.txt), [current_official_sdk_source.json](current_official_sdk_source.json): 최초 회전수 조회 timeout과 최신 공식 SDK 분기 확인. 아래 별도 사본 재시도 성공을 최초 실패에 소급하지 않는다.
- [speed_retry_summary.json](speed_retry_summary.json), [speed_retry_verification.json](speed_retry_verification.json): 별도 SDK 사본에서8.012739/12.015371 Hz와 최종10.015914 Hz 복원 실측. 시작 직후12 Hz 요청의9.961163 Hz 미적용도 보존.
- [c1_rpm_classification.patch](c1_rpm_classification.patch), [speed_retry_source.json](speed_retry_source.json): C1 major4를 기존 RPM `0xA8` 분기로 보내는 진단 사본의 조건 한 줄. 공식 작업 트리·운영 드라이버는 불변.
- [source_manifest.json](source_manifest.json), [verification.json](verification.json): 실행 판본·probe hash·원격/회수 기록 해시·독립 JSONL 재집계·권한 복원/잔여 프로세스 확인.
- [telemetry_summary.json](telemetry_summary.json): Jetson 온도/전체CPU·보드 입력 전력. LiDAR 광학 온도·USB 전압을 측정한 값이 아님.

기본 수신 기준은 워밍업 후300초 이상·평균9.5 Hz 이상·driver 오류/재시작/강제kill 없음·header 단조 증가·정상 Stop motor/종료다. 이는 순간 공백이나 원천 무손실 보증이 아니다. 5분 시험은 기준을 통과했지만 201.028 ms 공백1회의 원인은 미확정이다.

원시 기록은 Git 제외 `build/c1_hardware_2026_11_10/remote/`와 약11 MiB인 `c1-private-recording.tar.gz`에 보존한다. 압축본 SHA-256은 `dda26a7934549576fde9a052e8a2f81b1f787096168627c5ea9f746d07c3f82d`다. `capture-manifest.sha256`의 원격 원본과 회수본은 모두 일치했다. 공개 Git에는 실내 ranges/각도 점군·serial number·상세 로그를 넣지 않았다.

추가 RPM 재시도 원본은 같은 Git 제외 경로의 `speed_remote/speed_retry/`와 `c1-speed-retry-private.tar.gz`에 있다. 후자 SHA-256은 `82da2b831d177ec5245df0055d7aba5fa192d7ed4fe91bdb73e08422f0f78524`다. 전체 전송 해시와 PC에서의 스캔 CSV 재집계를 확인했다. 최초 `verification.json`·`source_manifest.json`은 RPM 재시도 전 판본이며, 최종 probe/재시도 검증은 `speed_retry_verification.json`을 따른다. 당시 probe v2/v4와 최종 지연 명령 판본을 각각 보존했다.

재현 자료 위치:

| 비공개 경로(위 remote/ 아래) | 내용 |
|---|---|
| `raw_quick*`, `raw_quick_v2*` | 최초 2초 timeout 직후 종료한 실패 |
| `raw_startup_checked-*` | 초기 15초 허용·이후2초 warmup/10초 측정. 첫 timeout1회와 실제102스캔 보존 |
| `ros_continuous/`, `ros_no_angle_fill/` | 원래 LaserScan JSONL·driver log·집계 |
| `options_standard8*` | RPM 지원 조회 실패. 속도 설정 전 중단 |
| `options_dense10_checked-*` | DenseBoost의 정렬 전 자료·metadata |
| `replay_root/recorded_replay/` | 원문 추출 함수·BSD license·stub·예시입출력·전체102사례 집계 |
| `probe_c1_sdk_v2.cpp`, `probe_c1_sdk_v4.cpp`, `probe_c1_ros.py` | 시험 당시 정확한 probe 소스 |

재사용 진입점은 [`probe_c1_sdk.cpp`](../../../../scripts/probe_c1_sdk.cpp), [`probe_c1_ros.py`](../../../../scripts/probe_c1_ros.py)다. SDK probe는 해당 고정 SDK object와 링크하며, CLI는 `PORT OUTPUT_PREFIX DURATION_S WARMUP_S [MODE [RPM [--rpm-after-warmup]]]`이다. RPM을 주면 성공한 제어 지원 확인 없이는 보내지 않는다. 이 C1의 RPM 시험은 위 patch를 적용한 별도 SDK object로 링크했다. 지연 옵션은 첫 스트림 안정화 후 명령을 보내고 다시 warmup한 뒤 측정한다. ROS probe는 시스템 Humble 환경에서 별도 domain·localhost만 사용하며 manufacturer node 경로와 새 비공개 결과 경로를 받는다. 영구 권한/udev/서비스를 자동 설치하지 않는다.

실행 실패 보존: 처음 colcon 진입은 ROS setup과 `set -u`의 환경변수 충돌로 빌드 전 중단했다. 이후 `set -e`로 같은 공식 소스를 빌드했다. 최초 raw2회는 첫2초 scan timeout을 즉시 실패 처리했고, 시작 과도기를 분리한 뒤 정상 취득했다. 이 실패를 SDK/센서 성능 성공으로 소급하지 않는다.
