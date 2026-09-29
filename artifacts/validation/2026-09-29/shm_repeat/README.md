# SHM 16 MiB 반복 기동 검증

상세 해석은 [A3 보고서](../../../../docs/simulation/JETSON_REPLAY_2026_09_28.md#shm-반복-검증과-imu-누락-분리--929-추가)에 정리했다.

- `jetson_results/shm_repeat_v1_{a,b,c,d,e,f}/`: 기존 소스·SHM 16 MiB·legacy·동일 20.968초 입력. 엔코더 발행 추적 ON/OFF 교차, 콜백 진단은 모두 ON.
- `shm_repeat_v1_{a,b,c,d,e,f}.log`: 실제 실행 명령·실행기 종료/잔존 프로세스 판정.
- `comparison_v1.json`: 6회 DDS 실제 설정·입력/명령/자원 집계. `scripts/summarize_dds_trials.py`로 생성.
- `imu_analysis_v1.json`: 원본 IMU 취득 시각과 콜백 대조. `scripts/analyze_imu_delivery.py`로 생성. 원시 센서 기록은 수정하지 않았다.
- 회수 archive SHA-256 `7948c321058972a54d18e8724c6f7eb28aad41b8620366114e64412c339d7e3d` 원격/로컬 일치. 원시 bag 제외, archive는 Git 제외 `build/`에 보존.

모두 기능/스케줄/진단/운영/EOF 정지/정상 종료 통과, 관련 잔존 프로세스 없음. 각 trial의 4개 participant SHM-only·16 MiB 확인. 원시 센서 내용·제어기·보호 기준·전력 설정을 변경하지 않았다. 6개의 짧은 재생을 연속 장시간 실행으로 해석하지 않는다. IMU 일부 누락과 50 Hz 실행 간격 변동은 남았다.
