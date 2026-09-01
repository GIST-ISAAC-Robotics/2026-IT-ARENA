# 공식 트랙 v2026.09.01 재검증 근거

검증일: 2026-09-01

## 고정한 입력

- 주최 측 릴리스: [`v2026.09.01`](https://github.com/MOSW626/istech-it-arena/releases/tag/v2026.09.01)
- 태그 커밋: `921f3f9a044f1a38ff849cb8e19d00182dd5533b`
- 자산: `it_arena_track_v2026.09.01.zip`, 822,391 bytes
- SHA-256: `f40aca619a6207f48f33741a56716edc65e948a674c7937f70b44476205b894c`
- 로컬 보존 위치: `assets/track/official/v2026.09.01/`

기존 `v2026.08.31`, 초기 전달본, 실험본과 당시 성공·실패 기록은 덮어쓰지 않았다.

## 배포 원본 재검증

| 항목 | 결과 |
|---|---|
| ZIP 경로·파일 수 | 경로 이탈 없음, 파일 24개. 실행에 불필요한 `__pycache__/track_gen.cpython-314.pyc` 1개가 추가로 동봉됨 |
| SDF | `gz sdf -k` 통과 |
| 원본 링크·형상 | 링크 20개, visual 1,113개, collision 1,107개 |
| 첫 물리 갱신 | DART에서 약 2.73초 후 정상 종료 |
| 물리 2,000회 | 약 4.87초 후 정상 종료 |
| ArUco 재질 | 잘못된 `script` 0개, `aruco/` 상대 경로·흰 diffuse·PNG 네 개 존재 |
| 정면 진단 카메라 | ID 0·20·30·45 모두 표시·검출 |
| #8 정적 간섭 | 20×15 cm 차량 외형으로 본선 2,332, 지름길 155/141개 표본 모두 통과 |
| #9 | `longitudinal_stagger_m=0.2`, 실제 같은 행의 좌우 차이와 일치 |
| #10 | 그리드 중심 z=0.0037 m, 상단 z=0.0042 m, 노면 상단 z=0.003 m보다 높음 |
| #11 | 설계·scene의 출발/결승 기준, 신호등·ID 0의 경로 기준이 `s=0`으로 정렬 |

구버전과 새 버전의 `visual`·`collision`을 링크 pose까지 세계 좌표로 정규화해 비교했을 때, 형상 수는 같고 변경은 ArUco 4개의 visual/collision 8개와 그리드 visual 6개뿐이었다. 그 밖의 도로·벽·잔디·접촉 형상 변경은 발견하지 못했다.

생성기는 ROS Jazzy에 포함된 OpenCV 4.6에서 `generateImageMarker` API가 없어 그대로 실행되지 않았다. 동등한 구 API `drawMarker`로만 호환시켜 재생성했을 때 19개 주요 출력 중 16개가 배포본과 바이트 단위로 같았다. PDF·preview·DXF만 폰트·문서 메타데이터·마지막 비트 수준 실수 표현 때문에 달랐고, SDF·scene·CSV·지도·마커 PNG는 같았다. 공식 README는 별도 가상환경에 최신 `opencv-contrib-python` 설치를 안내하므로 이 관찰만으로 배포 결함을 주장하지 않는다.

## 팀 실행 월드와 차량 회귀 검사

- [`smoke_low_load_30.json`](smoke_low_load_30.json): RGB·깊이 848×480 약 30.30 Hz, ToF 여섯 개 8×8 약 15.15 Hz, IMU·엔코더·직진·조향·명령 중단 정지와 정상 종료를 통과했다. 주기는 시뮬레이션 시간 기준이며 실제 처리율은 약 9.85 Hz였다.
- [`brisk_one_lap.json`](brisk_one_lap.json): 본선 1바퀴+약 2 m, 총 48.6927 m를 통과했다. 최고 지면 속도 0.784933 m/s, 최대 중심선 거리 0.122350 m, 정적 벽 겹침 표본 0개, 조기 출발 없음, ID 0/20/30/45 검출, 좌·우 벽 전환, 실제 정지, 종료 코드 0/0을 확인했다.
- Python 검사 114개, C++ 차동 검사 1개와 ROS 패키지 5개 빌드가 통과했다. 파생 공식 월드는 엄격한 SDF 검사와 입력·출력 해시 검사를 통과했다.

## 새로 남은 위험: 전방 카메라의 벽 부착 ArUco 접근각

이번 파생 월드는 새 릴리스의 공식 PNG/PBR 판과 pose를 그대로 보존한다. 차량의 D435i RGB 모델을 본선 중심선 위에 정지시켜 확인하면 정면 진단 카메라와 달리 접근각에 따라 검출 구간이 짧다.

| ID | 1.50 m | 1.00 m | 0.75 m |
|---:|:---:|:---:|:---:|
| 0 | 실패 | 통과 | 통과 |
| 20 | 실패 | 실패 | 통과 |
| 45 | 실패 | 통과 | 통과 |
| 30 | 실패 | 통과 | 실패 |

촘촘한 정지 표본에서는 ID 0·45가 1.00/0.80/0.75 m에서, ID 30이 1.20/1.10/1.00 m에서만 검출됐다. [`default_cases_report.json`](marker_vehicle_view/default_cases_report.json)과 [`dense_sweep_report.json`](marker_vehicle_view/dense_sweep_report.json)은 실패를 포함한 원본 결과다. 장시간 시인성 검사 중 한 번은 ToF bridge가 `-11`, 한 번은 Gazebo 서버가 종료 기한 안에 끝나지 않아 강제 종료됐으므로 두 보고서의 `shutdown_clean=false`도 지우지 않았다. 같은 최종 월드의 독립 스모크와 한 바퀴 검사는 정상 종료했다.

낮은 속도의 실제 주행에서는 네 ID를 모두 잡았으므로 “인식 불가능”이라고 결론 내리지 않는다. 다만 20 km/h 목표, 다른 차량 가림, 모션 블러와 실제 노출을 고려하면 짧고 비연속적인 검출 구간은 설계 위험이다. 운영진 게시 전에는 실제 D435i와 인쇄 마커로 속도·각도별 시험을 추가하고, 마커 pose가 최종 시공 방향인지 확인한다.

- [ID 30, 1.50 m 실패](marker_vehicle_view/id30_150cm_fail.png)
- [ID 30, 1.00 m 통과](marker_vehicle_view/id30_100cm_pass.png)
- [ID 30, 0.75 m 실패](marker_vehicle_view/id30_075cm_fail.png)
- [ID 30, 저속 한 바퀴 중 검출](marker_vehicle_view/id30_detected_during_lap.png)

이 결과는 단일 차량·본선·이상적인 렌더링 조건이다. 지름길 연속 진입/합류, 다중 차량, 실물 재질·조명, 고속 검출과 실제 안전성을 보증하지 않는다.
