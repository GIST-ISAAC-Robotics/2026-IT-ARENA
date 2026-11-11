# ESP32·LSM6DSOX 실물 입력 집계

GPT-6.1 Sol의 직접 실행이다. [실측 보고서](../../../../docs/sensors/ESP32_LSM6DSOX_HARDWARE_2026_11_11.md)가 조건·판정·한계의 기준이다. 기울어진 미장착 상태에서 형식·단위·주기 위주로 확인했으며 보정/취득 시각/장착 검증 완료가 아니다.

| 파일 | 범위 |
|---|---|
| `initial_summary.json` | 첫10초 센서/메모리/설정 확인 |
| `failed_first_summary.json`, `audit_failed.json` | 원래 수신기5분의 CRC4·sequence49프레임 누락 실패 |
| `no_gc_summary.json`, `audit_no_gc.json` | 수신 중 순환 GC를 중지한5분: gyro/accel 각59,721개·199.0685 Hz·CRC/sequence 공백0 |
| `reset_reopen_summary.json`, `audit_reset.json` | 리셋/재개방 뒤10초 수신 |
| `reset_in_place.json` | 포트를 유지한 software reset과 포트 재개방 시 새 boot 관측을 구분 |
| `source_manifest.json` | 실제 app BIN·수신기 판본·현재 재현 소스 해시와 ST/IDF 커밋 |
| `cleanup.json`, `verification.json` | 종료/원래 권한 복원·전송 해시·독립 원문 대조·미완료 항목 |

sensor timestamp raw의 공장 편차는 명목208 Hz와 실측약199 Hz의 차이를 설명하는 근거다. `time_contract_verified=false`, `calibration_adopted=false`를 유지한다. gyro 평균 후보와 가속도 통계는 운영 보정값이 아니며 장착 후 재확인한다. FIFO watermark8보다 빠른 폴링 때문에 I1 인터럽트 동작은 미검증이다.

원래 Flash·원문 UART·전체 프레임·고유 장치 정보·빌드/업로드 로그는 Git 제외 경로 `build/esp32_imu_hardware_2026_11_11/`에 보존했다. 공개 JSON에는 집계와 설정만 남겼다. [원문 감사 도구](../../../../scripts/audit_imu_probe_recording.py)와 [수신기](../../../../scripts/probe_esp32_imu.py)는 원시 입력을 승인된 제어 입력으로 연결하지 않는다. 실제 UART 앱은 build v2이며 watermark1 후보는 빌드 후 WSL 오류로 회수/업로드하지 못했다.
