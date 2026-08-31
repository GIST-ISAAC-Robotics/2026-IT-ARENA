# 공식 기반 트랙 전환 검증 — 2026-08-31

대상은 `v2026.08.31` 공식 자료의 본선 45 cm·지름길 20 cm와 문서화된 마커/시설 보정을 적용한 `official` 월드입니다. 기존 지름길 25 cm `experimental` 결과를 복사한 것이 아니라 새 월드에서 실행했습니다.

## 고정 보고서

| 검사 | 결과 | 근거 |
|---|---|---|
| 짧은 센서·주행 검사 | 통과 | [smoke_low_load_30.json](smoke_low_load_30.json) |
| RGB/깊이·ToF | RGB/깊이 848×480, 약 30.30 Hz; 여섯 ToF 8×8, 약 15.15 Hz | 위 smoke의 시뮬레이션 타임스탬프 기준. 실제 벽시계 RGB/깊이는 약 8.41 Hz, 실시간 비율 약 0.278 |
| 직진·조향·엔코더·IMU·watchdog | 직진 0.23065 m, yaw 변화 0.18363 rad, 엔코더 변화 5445/5763틱, IMU 수신, 명령 중단 정지 | 위 smoke; 정지 속도 약 0.000035 m/s, 종료 코드 0 |
| `brisk` 본선 한 바퀴 | 통과, 진행 48.6927 m | [brisk_one_lap.json](brisk_one_lap.json); 한 바퀴 46.6329 m + 약 2 m |
| 주행 상태 | 최고 실속도 0.79431 m/s, 최대 중심선 거리 0.12421 m, 고정 외형·정적 장애물 겹침 0표본 | 단일 차량·본선 주행이며 고속/다중 차량/지름길 검증 아님 |
| 인지·출발 | 빨강/노랑/초록, 마커 0/20/30/45, 좌우 벽 선택, 조기 출발 없음 | 제어 노드의 실제 구독 목록은 RGB/라이다, ToF 보호층은 여섯 점군/엔코더/명령. 정답·지도 구독 없음 |
| 종료 | 안정 정지, 정지 요청 뒤 0.08012 m 이동, 종료 코드 0/0 | 두 프로세스 오류 없음, `shutdown_clean: true` |

본선 검사 입력 16개의 SHA-256은 최종 생산 코드·월드와 일치함을 다시 확인했습니다. 이후 변경에서는 같은 성공 보고서가 현재 입력을 검증한 것으로 오해하지 않도록 입력 해시를 비교해야 합니다.

## 정적·빌드 검사

- ROS 패키지 5개 빌드 통과.
- `python3 -m pytest tests src/arena_vehicle_interface/test -q`: 105개 통과.
- `ctest --test-dir build/arena_gazebo --output-on-failure`: 단일 모터/차동 수식 검사 1개 통과.
- 공식 생성기의 `--check`, 이전 실험 생성기의 `--check`, 공식 실행 월드의 `gz sdf -k` 통과.
- 초기 ZIP과 해제 파일 23개 보존 확인. 신규 공식 ZIP은 SHA-256 `897183d2e2541458d190a0a1e3f76bff754c67fab894f2302c3110830a86149b`, 파일 23개.
- 고정 20×15 cm와 조향 최대 돌출을 감싼 직사각형 각각에 대해 본선 2,332·분기 155/141·그리드 6 표본의 정적 통과 검사 성공. 연속 궤적·노면 경계 준수·실차 통과 증명은 아닙니다.

## RGB 자료의 증거 범위

- [ID 0](marker_0.png), [ID 20](marker_20.png), [ID 45](marker_45.png)는 저장 PNG 자체를 다시 ArUco 검사하여 해당 ID가 검출됨을 확인했습니다.
- [ID 30 상태 통지 시점의 참고 영상](marker_30_status_snapshot_not_detection.png)은 저장 이미지 자체에서는 마커가 검출되지 않았습니다. ROS 상태 메시지와 최신 RGB가 비동기로 도착하여, 상태 통지를 본 뒤 저장한 프레임이 해당 검출 프레임과 같지 않았습니다. 이 사진을 ID 30 가시성 성공의 증거로 사용하지 않습니다. 실행 중 ID 30 검출·벽 전환은 보고서와 원시 궤적에 따로 기록되어 있습니다.
- [빨강](signal_red.png)·[노랑](signal_yellow.png)·[초록](signal_green.png)은 실행 중 참고 RGB입니다. 세 상태의 같은 타임스탬프 동기화 시험은 아닙니다.
- 새로운 낮은 7 cm 코드의 전 출발 위치·접근 거리별 반복 인식률은 아직 측정하지 않았습니다.

원시 로그·궤적·최종 스캔은 Git 제외 경로 `artifacts/tests/basic_autonomy_official/`에 보존했습니다. 보고서·참고 PNG만 이 폴더에 고정했습니다. 전체 트랙과 시설 확인용 실제 Gazebo 사진은 [촬영 묶음](../../../screenshots/2026-08-31/official_update/README.md)에 있습니다.

## 재실행

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/smoke_simulation.py --track official --d435i-profile low_load_30
python3 scripts/validate_basic_autonomy.py --laps 1 --speed-profile brisk \
  --max-sim-seconds 300 --output artifacts/tests/basic_autonomy_official_new_run
```

실물 노면 마찰·조향/제동 지연·마커 조명·ToF 가림과 간섭이 보정되지 않았습니다. 이 결과는 기본 소프트웨어 회귀 확인이며 실제 경주의 안전 속도나 완주 보증이 아닙니다.
