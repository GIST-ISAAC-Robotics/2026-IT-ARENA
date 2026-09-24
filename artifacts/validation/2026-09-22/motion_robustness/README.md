# 운동 센서 오류 주입 검증 근거

해석과 최종 결과는 [시험 기록](../../../../docs/autonomy/MOTION_SENSOR_ROBUSTNESS_2026_09_22.md)을 우선한다. 이번 값들은 실물 측정값이 아니라 민감도 검사 조건이다.

완료: `stage_a` 7조건·`stage_b` 4조건 모두 수락, 회귀 302개 통과. 복합 한 바퀴 48.6527 m·최고 8.26·코너 중앙값 4.37 km/h, 실제 정지·정상 종료·벽/노면 이탈 표본 0이다. 조건별 1회이며 모든 오차 조합/실물 안전을 보증하지 않는다.

## 파일 읽는 순서

1. 배치 폴더 `progress.json`: 회귀 검사·조건별 종료 코드·전체 완료 여부.
2. 실행 폴더 `motion_summary.json`: 주행·실제 정지·종료·노면·실제 주입·주행 중 고장 판정의 결합 결과.
3. `report.json`, `road_audit.json`: 원래 평가기의 상세 결과. 요약이 원본 실패를 덮어쓰지 않는다.
4. `trajectory.jsonl`, `stop_trace.json`: 주행 중/최종 정지 단계의 평가 표본.
5. `motion_injection.jsonl`, `motion_test_profile.json`: 실제 값 변환·취득/방출 시각·누락 근거와 정규화된 설정. `checks`는 감사 항목 수이며 메시지 수는 `published`/`dropped`다.
6. `source_snapshot.zip`, `input_sha256`: 해당 주행의 소스와 해시. 고정 제어기 비교인지 확인한다.

`report.json`의 원래 `passed`와 추가 판정 `accepted`는 별개다. 동적 고장은 출발 뒤 실제 이동→입력 이상에 따른 보호층 정지→실제 차체 정지→입력 복구 후 이동을 모두 요구한다. 처음부터 움직이지 않거나 명령만 0인 경우는 성공이 아니다. 약 10 Hz 표본의 벽/노면/제동 검사는 연속시간 무충돌 또는 실차 제동 보장이 아니다.

## 재현

ROS 2 Jazzy·Gazebo가 설치된 WSL에서 저장소 루트와 빌드된 환경을 사용한다. 기존 산출물을 덮어쓰지 않도록 새 배치명을 지정한다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_motion_robustness.py --batch repeat_a --cases clean gyro_plus1 gyro_minus1 wheels_plus5 wheels_minus5 delay15 clock_opposed5
python3 scripts/run_motion_robustness.py --batch repeat_b --cases imu_drop wheels_drop imu_delay_burst combined
```

각 배치는 회귀 검사 후 순차 실행하며, 실패하면 다음 조건을 자동으로 진행하지 않는다. 테스트 전용 중계기 실행 시에만 오류를 넣는다. 일반 `local_pursuit.launch.py`의 `motion_test_profile` 기본값은 빈 문자열이며 오류 주입이 꺼져 있다.

Jetson·실제 구동계에 이 배치를 그대로 실행하지 않는다. 30초 벽시계 센서 유예는 느린 오프라인 시뮬레이션용이며 실물 watchdog 값이 아니다. 센서/제어 시각과 지연 판정은 시뮬레이션 시간으로 수행한다.
