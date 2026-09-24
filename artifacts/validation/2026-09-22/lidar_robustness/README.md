# C1 거리 오차·지연 검증 근거

설계·해석·최신 결과는 [시험 기록](../../../../docs/autonomy/LIDAR_ROBUSTNESS_2026_09_22.md)을 읽는다. 이 디렉터리의 성공/실패 자료를 대체하거나 삭제하지 않는다.

## 이미 완료한 비교

- `probe_v7`, `probe_v8`: 정지 직선 벽의 조건별 50 seed 계산. `source_snapshot.zip`에 해당 시점 알고리즘과 진단 도구를 보존한다. 실제 주행이 아니다.
- `uniform30_v7`: 수정 전 균일 ±3 cm, 슬롯 3 한 바퀴·정지·종료 통과. 속도 둔화와 일시 경로 거부 표본 12개.
- `uniform30_v8`: 수정 후 같은 오차 조건, 한 바퀴·정지·종료 통과. 최고 8.07 km/h, 코너 중앙값 4.44 km/h. 일시 경로 거부 표본 1개.
- `combined_v8_two_laps`: ±3 cm+50±20 ms 추가 지연, 두 바퀴 95.3056 m·정지·종료 통과. 최고 8.06 km/h·코너 중앙값 4.26 km/h. 최초 장애로 보류했던 **노면 감사도 사용자 재개 요청 후 완료**했다. 주행 726·정지 530표본에서 이탈 면적 모두 0이다.
- `resume_v8`: 첫 재개 배치. 전체 276개 회귀와 `resume_v8_bias_plus30`/`bias_minus30`/`delay80` 각 12 m의 주행·정지·종료·노면 감사를 통과했다. 이어진 `resume_v8_clean`은 빈 `lidar_test_profile:=` 인자를 ROS CLI로 넘겨 시뮬레이터 시작 전 실패했다. 초기화 실패 원본과 배치 중단 기록을 보존한다.
- `resume_v8_remaining`: 빈 선택 인자 처리 수정 후 **전체 277개 회귀 통과**, 남은 두 조건만 재실행한 배치다. `pytest.log`·`pytest.xml`·`progress.json`을 읽는다. `resume_v8_remaining_clean`은 12.0585 m·최고 8.16 km/h, 주행·실제 정지·정상 종료·벽/노면 이탈 표본 0이다.
- `resume_v8_remaining_delay300_stop`: 7.074초·71표본 동안 보호층 `stale_input`·안전 명령 모두 0·실제 평면 속도 1 cm/s 미만·정상 종료로 **출발 거부**를 확인했다. 원래 `passed=false`는 완주하지 않아 남으며 바꾸지 않는다. 제어기 자체 상태는 `RUNNING` 22/`SENSOR_STOP` 49표본이었다. 기존 경로 제어기 450 ms/보호층 300 ms 제한 차이이며 두 계층이 모두 내내 정지한 결과로 해석하지 않는다.
- `recorded_corner_noise_v8_verified`: 이전 커브 스캔 네 건·50 seed씩 v7/v8 비교. 입력·소스·결과를 보존한다. 실제 주행 시험이 아니다.

## 자료 읽는 순서

1. `report.json`: 입력 파일 해시, 소스 ZIP, 조건, 주행/정지/프로세스 종료 판정. `passed`는 전체 검사 결과다.
2. `road_audit.json`: 실제 SDF 노면과 20×15 cm 차체 외형의 주행·정지 표본 대조. 공식 패널티 판정이나 연속 접촉 검사가 아니다.
3. `robustness_summary.json`: 위 결과와 상태 메시지 표본을 압축한 사후 집계. 상태 개수는 사건 수나 제어 주기 수가 아니다.
4. `acquisition.json`: 순차 취득 시각 근사 오차와 실제 주입한 거리 오차/지연 통계. 지연은 첫 점 시각을 바꾸지 않는다. `release_source_stamp_s`가 실제 방출 때의 원시 센서 시각이며, 기존 `source_stamp_at_publish_s` 필드는 조립 완료 때의 시각으로 남는다.
5. `trajectory.jsonl`, `stop_trace.json`, `simulation.log`: 세부 상태·평가용 정답 자세·정지·종료 근거. 정답 정보는 평가기에만 사용하고 제어기로 전달하지 않는다.

과도한 지연의 `expected_stale_rejection.protection_passed`는 출발 거부 검사다. 원래 `passed=false`인 결과를 완주 성공으로 읽지 않는다.

## 재현 예

WSL의 프로젝트 디렉터리에서 ROS/설치 환경을 불러온 뒤 새 출력 경로로 실행한다. 기존 출력 경로를 재사용하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/validate_basic_autonomy.py \
  --autonomy-mode local_pursuit --lidar-acquisition sequential --lidar-compensation both \
  --control-rate-hz 50 --render-backend wsl_nvidia --collision-detector configured \
  --sensor-wall-timeout-s 30 --disable-speed-bump --red-duration-s 1 \
  --speed-profile local_fast --laps 1 --grid-slot 3 --shutdown-mode service \
  --max-sim-seconds 130 --wall-timeout-s 1500 --stop-wall-timeout-s 120 \
  --lidar-test-profile config/tests/lidar_uniform30.json \
  --output artifacts/validation/2026-09-22/lidar_robustness/NEW_RUN
python3 scripts/audit_local_pursuit_road.py artifacts/validation/2026-09-22/lidar_robustness/NEW_RUN
python3 scripts/summarize_lidar_robustness.py artifacts/validation/2026-09-22/lidar_robustness/NEW_RUN
```

`lidar_test_profile`을 생략하면 거리 오차/추가 전송 지연을 주입하지 않는다. 이 자료는 실물 C1의 전체 오차 분포·통신 지연·Jetson 실행 성능을 측정한 결과가 아니다.
