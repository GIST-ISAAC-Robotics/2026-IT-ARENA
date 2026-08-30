# 실험 트랙 시설과 차량 카메라 확인

갱신: 2026-08-31. **주최 측 확정 사양이 아닌, 알고리즘 준비용 실험 시설입니다.** 원본은 변경하지 않습니다.

## 적용한 배치

| 시설 | 실험본 설정 | 유지하거나 구분한 항목 |
|---|---|---|
| 신호등 | 도로를 가로지르는 가로대, 접근 방향을 향한 렌즈. 가로대 42 cm·렌즈 중심 30 cm·반지름 2.6 cm | 기존 경로 위치 유지. 초기 빨강, 데모에서 동적 전환 |
| 과속방지턱 | 진행 방향 20 cm × 횡단 방향 45 cm × 노면 위 높이 1 cm. 검정·노랑 곡선 형상 | 기존 위치·높이 유지. 잘못 놓인 길이·폭 축 수정 |
| ArUco 표지판 | ID 0·20·30·45, 중심 높이 20 cm, 흰 지지판 13×13 cm | 기존 `s`와 ID 유지. ID 30의 반대편 보정 유지 |
| 출발 표시 | 기존 6개 출발 위치에 길이 26 cm·폭 16.6 cm의 U자 표시와 번호 | 실제 차량 생성 좌표는 변경하지 않음 |
| 피니시 | 기존 출발·도착 위치에서 노면 전체 폭 45 cm × 진행 방향 깊이 10 cm, 체크무늬 9×2칸 | 노면 표시이며 물리적 장애물이 아님 |

원래 도로 중심선·누적 거리·본선 45 cm·지름길 25 cm와 차량 20×15 cm, 축거 14.5 cm·윤거 13.5 cm는 유지합니다. 노면 상단은 z=0.003 m이고, 출발·피니시 표시 중심은 z=0.0038 m·두께 0.0004 m여서 노면에 묻히지 않습니다.

화면의 번호는 앞쪽부터 1~6번입니다. 기존 `grid_slot` 인덱스와는 다음처럼 대응합니다.

| `grid_slot` | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| 표시 번호 | 5 | 3 | 1 | 6 | 4 | 2 |

### 방지턱의 곡선과 물리 접촉

진행 방향 좌표를 x, 길이를 L, 높이를 H라고 할 때 `h(x) = H/2 × (1 + cos(2πx/L))`, `-L/2 ≤ x ≤ L/2`를 사용합니다. 양 끝이 노면과 만나며, 중심에서 가장 높습니다. 40개 선분으로 근사한 윗면의 이상적 곡선 대비 높이 차이는 0.016 mm 미만입니다.

시각 형상은 색 띠별 메시이고, 충돌 형상은 윗면 양 끝이 같은 곡선 표본과 일치하는 경사 상자 40개입니다. 메시 충돌은 이 컴퓨터의 DART/ODE 경로에서 오류가 발생해 사용하지 않습니다. OBJ에는 면 법선을 명시해 OGRE2에서 검정·노랑 재질이 정상적으로 보이도록 했습니다. 파일이 존재하는지만 확인한 것이 아니라 실제 차량 RGB와 저속 통과로 확인했습니다.

### 표지판의 크기와 방향

이번 **10 cm는 검은 테두리의 바깥 변 길이**입니다. 흰 여백은 사방 1.5 cm씩이고 전체 판은 13 cm입니다. 원본 PNG의 코드 영역은 전체 1,000 px 중 700 px이므로, 전체 PNG를 10 cm 면에 놓으면 코드 영역은 7 cm가 됩니다. 이번 실험은 이 크기 기준을 명확히 바꾼 것이며, 원본과 물리적 인쇄 크기까지 같다고 주장하지 않습니다.

표지판은 본선의 해당 `s`보다 1.2 m 앞선 상류 지점을 바라봅니다. 검은 장식 테두리가 실제 코드와 별개의 후보로 잡히는 문제를 없애기 위해 지지판까지 흰색입니다. 원본 PNG·ID는 그대로 보존하고, 실행 화면의 셀 형상만 `DICT_4X4_50`에서 생성합니다. `scene.json`의 마커 `pose`는 이제 검은 코드 앞면의 **중심**이며 `pose_reference`에 기준을 명시합니다.

## 실제 차량 영상 검증

- D435i RGB의 위치·각도·명목 시야각 69.4°×42.5°·848×480 해상도는 바꾸지 않았습니다. 시험 실행만 `low_load_30`을 선택했습니다.
- 출발 위치 6곳 모두에서 빨간 렌즈를 확인했습니다. 실제 세계 좌표·`CameraInfo`로 계산한 예상 렌즈 위치와 빨간 픽셀 군집을 대조하여 다른 빨간 물체를 잘못 인정하지 않도록 했습니다.
- 마커 4개를 각각 본선 중심선을 따라 1.5·1.0·0.75 m 상류에서 바라본 총 12조건 모두에서 해당 ID를 검출했습니다. 이 거리는 카메라와 표지판 사이의 직선 거리가 아닙니다.
- 합계 **18/18 가시성 조건 통과**입니다. OpenCV 4.6.0 기본 ArUco 검출 설정을 사용했습니다. 위치를 옮긴 서비스 응답뿐 아니라 별도의 Gazebo 세계 위치·방향 관측과 새 RGB 시각을 확인한 뒤 판정합니다.
- 시험용 정답 위치는 배치와 결과 검사용이며, 대회용 인지·주행 노드의 입력이 아닙니다. [Gazebo 위치 변경 시스템](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1UserCommands.html), [OpenCV ArUco 안내](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

[출발 위치의 실제 RGB](../../artifacts/screenshots/2026-08-31/facilities/signal_from_grid.png) · [ID 20 접근 영상](../../artifacts/screenshots/2026-08-31/facilities/marker_20_1m.png) · [ID 30 접근 영상](../../artifacts/screenshots/2026-08-31/facilities/marker_30_75cm.png) · [방지턱 접근 영상](../../artifacts/screenshots/2026-08-31/facilities/bump_approach.png)

### 통과 결과와 별도 발견 사항

0.16 m/s 명령으로 한 번 통과하며 약 1.11 m 전진했습니다. 차체 기준점은 평지 z≈3.00 mm에서 최대 z≈8.06 mm까지 올라갔다가 평지 높이로 돌아왔습니다. 이는 차체 기준점의 상승량이며, 방지턱 높이 10 mm를 측정한 값과 혼동하지 않습니다. 차량의 고속 거동·서스펜션·실물 타이어를 검증한 것은 아닙니다.

시설 수정 직후에는 정지 명령 후 `/odom` 속도가 흔들려 **6초 안에 0.01 m/s 미만을 0.5초 연속 유지**하는 조건을 통과하지 못했습니다. 동일 속도로 방지턱 이전 평지만 주행한 대조 시험에서도 마지막 2초 속도가 약 -0.055~+0.053 m/s로 흔들렸습니다. 실제 차체의 마지막 0.5초 양 끝 위치 차이는 약 1~2 mm였지만, 이를 완전 정지로 선언하지 않았습니다.

그 당시 검사 보고서는 `facility_checks_passed: true`와 `stop_check.passed: false`를 분리하며 종합 실패 상태 그대로 보존합니다. 이전 짧은 기본 동작 검사(smoke test)의 순간 속도 기준 통과가 지속적인 정지 안정성을 보장하지 않는다는 추가 근거입니다.

[시설 수정 직후 실패 보고서](../../artifacts/screenshots/2026-08-31/facilities/visibility_report.json) · [당시 평지 정지 대조 보고서](../../artifacts/screenshots/2026-08-31/facilities/flat_control_report.json)

보고서는 원본 그대로 보존하므로 그 안의 `image` 경로는 전체 시험 산출물이 있는 로컬 `artifacts/tests/`를 가리킵니다. 저장소에는 위에 링크한 대표 영상 4장과 보고서를 별도로 보존합니다.

### 같은 날 후속 수정과 재검사

기초 자율주행을 추가하면서 Gazebo Math 제한기 자체의 jerk ±20 설정에서 같은 정지 진동을 재현했습니다. 속도·가속도 제한을 유지하고 이 jerk 제한만 제외했습니다. [결정 0006](../decisions/0006-sensor-wall-following-demo.md)에 독립 재현 조건과 선택 근거가 있습니다.

수정 후 시설 검사에서는 **가시성 18/18·방지턱 통과·지속 정지·정상 종료를 모두 통과**했습니다. 0.16 m/s 통과에서 이동량 1.11597 m, 차체 최대 z≈8.057 mm, 복귀 z≈3.000 mm였고, 정지 기준 만족까지 약 0.54초·최종 속도 0·마지막 0.5초 양 끝 위치 차이 약 0.004 mm였습니다. 이전 실패 보고서를 덮어쓰지 않고 `artifacts/tests/facilities_after_limiter_fix/report.json`에 새 결과를 보존합니다. 공개용 사본은 [시설 수정 후 통과 보고서](../../artifacts/screenshots/2026-08-31/facilities/visibility_after_limiter_fix_report.json)와 [평지 수정 후 통과 보고서](../../artifacts/screenshots/2026-08-31/facilities/flat_after_limiter_fix_report.json)입니다.

신호등의 빨강→노랑→초록 전환과 차량 RGB 출발 판단도 추가했습니다. 일반 실행은 정적 빨강을 유지하고, `demo.launch.py` 또는 `traffic_light:=true`로 제어기를 켭니다. 수동 색 변경·출발 순서 재시작은 [기초 데모 안내](../autonomy/BASIC_DEMO.md)를 참고하십시오. 시설의 형상·배치나 원본 파일은 이 추가에서 변경하지 않았습니다.

## 실제 GUI 사진과 관찰 구도

[수정 후 트랙 사선 항공뷰](../../artifacts/screenshots/2026-08-31/facilities/track_oblique.jpg) · [출발 그리드 6개·차량·피니시 근접 사진](../../artifacts/screenshots/2026-08-31/facilities/start_grid.jpg)

두 사진은 실제 Gazebo 창에서 촬영한 1707×1019 px 화면입니다. 영상 합성이나 사후 형상 수정은 하지 않았습니다. 2026-08-31 시설 작업 당시에는 사용자가 직접 둘러볼 수 있도록 GUI를 일시정지·사선 항공뷰로 남겼습니다. 이후 사용자의 테스트 마무리 지시에 따라 모든 관찰·검사 프로세스를 종료했습니다. 다시 보려면 아래 실행 절차를 사용하십시오.

관찰 카메라는 `View Angle` 패널에서 다음 값으로 재현할 수 있습니다. 이 설정은 GUI 시점이며 차량의 D435i 센서 설정과는 별개입니다.

| 사진 | 위치 XYZ (m) | 회전 RPY (rad) | 수평 시야각 (rad) |
|---|---|---|---|
| 최종 사선 항공뷰 | 15, 2, 14 | 0, 1.02, 2.625 | 1.1 |
| 출발 구역 근접 사진 | 11.6, 4.3, 3 | 0, 1.02, 2.625 | 1.1 |

해당 탐색 실행은 `low_load_30`, `ROS_DOMAIN_ID=164`, `GZ_PARTITION=it_arena_facilities_view_20260831`을 사용했습니다. 이 실행에 별도 명령·관찰 도구를 연결할 때는 같은 통신 영역을 사용해야 합니다. 일반적인 새 실행의 기본값을 변경한 것은 아닙니다.

## 실행·수정·재검사

WSL의 프로젝트 폴더에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/build_experimental_track.py
colcon build --symlink-install
source install/setup.bash
ros2 launch arena_bringup simulation.launch.py d435i_profile:=low_load_30
```

치수는 `config/tracks/experimental.yaml`의 `facilities`를 수정하고 같은 절차로 재생성합니다. 소스·원본 실행 월드를 직접 편집하지 않습니다. 실험 월드는 13개 링크·1,116개 충돌체·1,295개 시각 형상으로 구성됩니다.

```bash
python3 -m pytest -q tests src/arena_vehicle_interface/test
python3 scripts/validate_track.py
python3 scripts/build_experimental_track.py --check
python3 scripts/validate_facility_visibility.py
python3 scripts/validate_facility_visibility.py --flat-control --output artifacts/tests/flat_stop_control
```

영상·전체 위치 기록·로그는 `artifacts/tests/`에 저장하며 검사마다 독립된 ROS/Gazebo 통신 영역을 사용합니다. 검사 실행은 자신이 시작한 프로세스만 종료합니다. 결과는 매 실행의 보고서로 판정하며, 과거 실패 결과를 새 성공 결과로 덮어써 해석하지 않습니다.

## 남은 범위

- 신호등 형상과 자동 순서는 실험 가정입니다. 실제 대회 신호 장치·시간표는 아직 확정하지 않았습니다.
- 정지 영상 검사와 저속 벽 추종 주행에서 표지판을 확인했습니다. 고속 모션 블러·노출·흔들림·다른 차량의 가림·지름길 진입 궤적은 미검증입니다.
- 배치·높이·인쇄 크기는 공식 규격이 아닙니다. [2026-06-30 미팅](https://maddening-cause-ce7.notion.site/2026-06-30-38f99fd42e3080f6956fe5a5b90d0824)은 이번 재조회에서 본문 접근에 실패했으므로, 과거 확인 기록과 후속 주최 측 확정을 구분합니다.
- 정지 속도 흔들림은 후속 제어 설정에서 해결했지만 실물 제동·서보 응답을 검증한 것은 아닙니다.

원본 보존과 선택 근거는 [시설 결정 기록 0005](../decisions/0005-experimental-facilities.md), 원본의 문제는 [트랙 감사 기록](TRACK_AUDIT.md)에 있습니다.
