# 가속도 독립 기록 계약 검증

- 범위: `arena_hardware_capture_v2` 오프라인 감사. 기존 v1은 유지한다.
- 실행: `run_checks.sh`, WSL Python/pytest. 관련 77통과, `tests_exit.txt=0`, `audit_exit.txt=0`, `source_hash_diff.txt` 비어 있음.
- `report.json`: 합성 9행(encoder/gyro/accel 각 3), 기록 무결성 통과. 가속도 available/unavailable/saturated 각 1이며 전량 이용 가능/실물 검증/자동 교정으로 해석하지 않는다.
- 원시 실측·보드·포트·ROS/주행·Jetson 실행 없음. 실행 시점 소스는 `source_snapshot.tar.gz`와 전체 해시 목록에 보존한다.
- [계약과 한계](../../../../docs/hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md#8-1023--독립-가속도-표본을-보존하는-v2)
