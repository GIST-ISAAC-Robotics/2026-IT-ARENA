# B0183+C1+6축 IMU 지역 Pure Pursuit 검증 자료

설정·해석은 [구현 보고서](../../../../docs/autonomy/LOCAL_PURE_PURSUIT_2026_09_21.md)를 우선한다. 전역 지도·정답 위치는 평가기에서만 사용한다. 각 폴더의 `source_snapshot.zip`, `report.json`, 실행 입력 SDF, 메시지 기록을 함께 확인한다.

## 실행 순서

| 실행 폴더 | 변경 및 결과 |
|---|---|
| `smoke_v1` | 표시 최적화 이전, 녹색 신호 전 벽시계 시간 초과 |
| `smoke_v1_retry2` | 정적 표시 배칭/EGL 추가. 신호등 빨강 0.5초가 허용 범위 밖이어서 실패 |
| `smoke_v1_retry3` | 초기 함수 적합 경로, 12.058 m 통과. 4.83 km/h·정지·정상 종료 |
| `lap_v1_retry3` | Bullet 조건에서 출발 한쪽 후륜 헛돎, 0 m 실패 |
| `lap_configured_video_v2` | 기존 충돌 설정으로 출발. 19.617 m 급커브 경로 불확실 정지 |
| `lap_configured_v3` | 반대 벽 대체 추가. 19.897 m 전방 경로 부족. Gazebo 강제 종료 실패 |
| `lap_configured_v4` | 연결 벽/법선 경로. 1.740 m 기둥 옆 연결 구간 선택 실패 |
| `lap_configured_v5` | 긴 연결 구간 후보 추가. 2.980 m 경로 흔들림/과주행 정지 |
| `lap_configured_v6` | 매개곡선 강건 평활화·전방 곡률/관측 길이 속도 제한. 48.713 m·8.18 km/h, 정지/정상 종료 통과 |
| `lap_configured_v6_repeat2` | 같은 제어기의 반복 시험은 2.200 m에서 전방 목표 부족 정지. 한 번의 성공과 반복성 구분 |
| `lap_configured_v7_repeat2` | 접힌 평행 경로 배제·전방 목표 부족 때 반대 벽 대체. 95.326 m·두 바퀴 주행/정지 성공. Gazebo SIGKILL 종료로 **전체 `passed=false`** |
| `smoke_service_v7` | 같은 v7 주행 코드와 서비스 종료. 3.040 m·정지/정상 종료·잔존 없음 통과 |
| `lap_service_v7_grid3` | 같은 v7으로 다른 슬롯 3에서 48.693 m·8.08 km/h·코너 중앙값 4.78 km/h. 주행/정지/정상 종료 전체 통과 |

앞선 짧은 성공을 후속 알고리즘의 성공으로 승계하지 않는다. 벽 겹침 0만으로 잔디 미침범이나 실차 안전을 선언하지 않는다. 후속 `road_audit.json`은 실제 SDF 노면과 20×15 cm 평면 외형을 사후 비교한 표본 검사이며 바퀴 접촉/심판 판정이 아니다.

## 영상과 진단

- **[v7 한 바퀴 3인칭 영상](video_service_v7_grid3/local_pursuit_third_person_drive.mp4)**: 슬롯 3·48.6927 m·최고 8.22 km/h·코너 중앙값 4.85 km/h. 960×540·20fps·33.25초/665프레임 전체 디코드 통과. [보고서](video_service_v7_grid3/report.json)의 주행/정지/정상 종료 전체 통과와 [노면 감사](video_service_v7_grid3/road_audit.json)의 주행/정지 노면 이탈 표본 0을 확인했다. 별도 사용자 요청으로 Luna가 녹화·검증했으며 기존 주행 코드는 유지했다.

- [v2 3인칭 진단 영상](lap_configured_video_v2/local_pursuit_third_person_drive.mp4): **급커브에서 정지하는 실패 영상**이다. 성공 완주 영상으로 사용하지 않는다. 960×540·20fps, 18.25초/365프레임 전체 디코드 확인.
- `stationary_scan_diagnostic.png`: 현재 소스로 과거 정지 스캔을 재계산한 진단 그림이다. 당시 실행의 경로와 구분한다. 생성은 `scripts/inspect_local_path_scans.py`.
- `last_scan.json`을 회귀 입력으로 참조하므로 v2~v5와 `v6_repeat2` 폴더의 해당 파일은 삭제하지 않는다.

`runtime_world.sdf`는 당시 입력을 바이트 그대로 보관한다. 배칭 메시의 URI는 당시 임시 디렉터리를 가리킬 수 있고, 메시 자체는 같은 실행 폴더에 별도 보존한다. 재실행은 원본을 임의 덮어쓰는 대신 전용 launch와 기록된 옵션으로 생성한다. 공식 ZIP·원래 월드·기존 `vehicle.yaml`은 변경하지 않았다.

## 정적·회귀 확인

- 최신 전체 Python/ROS 검사: **252개 통과**. 재개 후 종료/조향/저장 스캔 재현 하위 검사는 65개 통과했다. 기존 모드를 포함하며 C++ 변경은 없다.
- 원래 공식 실행 월드 SHA-256: `f73b59d2ea4afe0569559cdeab2b5bff646e3c5ef264541a085f0bf12bdb42af`.
- 기존 `vehicle.yaml` SHA-256: `e59508b8600e5bc6b9d56dc99797fbb23d5c36124ac2d4970d3af53832ddb920`.
- 새 차량 총질량 2 kg·깊이/ToF 센서 부재·별도 IMU·엄격한 SDF 검사, 배칭 전후 충돌/공식 마커 XML 및 꼭짓점 보존을 확인했다.
- `v7_repeat2`와 결과 정리 이후 일시 중단했다가 사용자 요청으로 재개했다. 이후 서비스 종료·짧은 주행·슬롯 3 한 바퀴·전체 회귀까지 수행했다. 추가 녹화·GitHub 게시·컴퓨터 종료는 수행하지 않는다.

재개 후 최신 [슬롯 3 보고서](lap_service_v7_grid3/report.json)는 `passed=true`, 최고 8.08 km/h·코너 중앙값 4.78 km/h·중심선 RMSE 1.94 cm다. [노면 감사](lap_service_v7_grid3/road_audit.json)는 주행 332·정지 491표본의 도로 밖 면적 0을 기록한다. 서비스 종료는 두 시험에서 1.435/1.583초, 자식 12/12 정상 종료·잔존 없음이다. 과거 두 바퀴 종료 실패를 소급해 통과로 바꾸지 않는다.

최신 [v7 보고서](lap_configured_v7_repeat2/report.json)는 최고 8.16 km/h·코너 중앙값 4.91 km/h·중심선 RMSE 1.81 cm·주행/정지 벽 겹침 0을 기록한다. [노면 감사](lap_configured_v7_repeat2/road_audit.json)는 주행 628·정지 494표본 모두 외형의 노면 밖 면적 0이다. [로그](lap_configured_v7_repeat2/simulation.log)의 Gazebo 종료 지연과 −9 반환값을 함께 확인한다. 이번 실행은 종료했고 추가 재실행하지 않은 상태다.
