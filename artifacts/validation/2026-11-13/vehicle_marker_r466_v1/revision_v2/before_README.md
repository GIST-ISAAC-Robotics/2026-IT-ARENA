# r466 차량 후방 ArUco의 선택형 점유 보조

2026-11-14 격리 stage. Windows 검사 뒤 실제 WSL ROS 대표 1회를 수행했으나 중복 quad 제외로 정면 cap이 실패했고 사용자 요청으로 일시중지했다. [최신 실패·회수·재개 상태](PAUSED.md)를 우선 읽는다. 공통 소스·Jetson·센서·ESP·모터는 변경/실행하지 않았다. 주행 성공, 설치 교정, 실차 안전의 증거가 아니다.

## 구현 결과와 범위

기존 RGB 작업 프로세스가 코스용 4×4 검출을 재사용해 등록된 차량 마커의 **검은 코드 외곽**으로 근사 PnP를 계산한다. 후방 중앙 부착을 가정해 차량 전방으로 길이 0.20 m, 좌우 폭 0.15 m인 별도 점유 polygon을 만든다. 상대차 높이를 고정하지 않으며, 이 크기는 사용자 지정 가상 영역이다.

- 기본 OFF. 빈 `vehicle_marker_profile_path`이면 추가 topic publisher/subscription과 PnP를 만들지 않는다.
- 옵션 ON은 현재 `g1_g2_node`의 등록된 dual plan/current runtime에 한정한다. 명목 FOV/CAD를 사용하므로 교정 완료를 요구하지 않는다.
- `LocalFollow`의 최종 점유 sweep와 `M3ExecutionFollow`의 각 추월 후보 sweep는 별도 cloud의 clearance만 기존 결과에 최솟값으로 합성한다. 기존 `swept_limit`, 제동·headway 계산을 재사용한다.
- plan/current 최종 후보도 같은 보조 점유로 제한하며, `/perception/vehicle_marker_regions`를 받은 `LidarSafety`는 자체 운동 이력과 현재 시각으로 다시 제한한다. `/drive` → 독립 보호 → `/drive/safe` 경계는 유지한다.
- scene.rays, scene.occupied_endpoints, 벽 fit, ReturnGrid, 관측 free, 측방 기억은 바꾸지 않는다. 보조의 미검출/만료/형상·시각 오류/운동 보정 불가는 기존 LiDAR 결과를 유지한다.
- **미연결:** 마커만으로 새 추월 시작 후보를 생성하는 단계. 현재는 기존 추종 감속 및 이미 생성·선택된 추월 후보의 점유 충돌 검사다. 이 한계가 남아 있으므로 차량 추월 기능 전체 완료로 부르지 않는다.

## 공식 원본과 크기

공식 저장소 current main은 `6c2589c61acb83928339291ae237180b17c2cb54`로 고정했다. [공식 MANUAL](https://github.com/MOSW626/istech-it-arena/blob/6c2589c61acb83928339291ae237180b17c2cb54/MANUAL.md), [등록 issue](https://github.com/MOSW626/istech-it-arena/issues/1)의 원 JSON·실제 첨부 바이트를 `sources/`에 보존했다. 실제 출력물 크기를 측정한 결과는 아니다.

| 팀 | 실제 제출 파일의 검출/표기 | 파일 전체를 50 mm로 출력할 때 검은 코드 변 |
|---|---|---:|
| KAIST | DICT_4X4_50, 1; 1000 px 중 696 px | 34.8 mm |
| UNIST | DICT_4X4_50, 7; 1000 px 중 696 px | 34.8 mm |
| GIST | DICT_4X4_50, 49; 1000 px 중 696 px | 34.8 mm |
| DGIST | 수정 제출 PNG는 11; 710 px 중 590 px | 약 41.5493 mm |
| POSTECH | DICT_7X7_100, 91; SVG 50 mm·검은 영역 viewBox 전체 | 50 mm, 외부 흰 여백 없음 |

DGIST의 [수정 제출](https://github.com/MOSW626/istech-it-arena/issues/1#issuecomment-5978592693)은 11이지만 주최 표에는 14가 남아 있다. stage는 실제 제출 이미지 11을 명시하고 14를 임의로 추가하지 않았다. POSTECH은 [제출 SVG](https://github.com/MOSW626/istech-it-arena/issues/1#issuecomment-5779593670)의 사전/ID이며 PNG 디코드 실패를 검출 실패로 숨기지 않았다. 그 원본 SVG를 OpenCV로 검출한 증거는 없고, 7×7/91의 별도 합성 픽셀 검출은 검사했다. GIST 49는 runtime 자기 차량에서 제외하고 `calibration_only` profile만 제공한다. 트랙 4×4 ID 0/20/30/45는 차량 대상이 될 수 없다.

50 mm를 34.8 mm 코드의 PnP 변 길이로 잘못 넣으면 이상적 핀홀 거리에서 약 1.437배 차이가 난다. [OpenCV ArUco 문서](https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html)의 검출 코너·실제 마커 변 길이·카메라 행렬 계약을 따른다. 인쇄 전체, 흰 여백, 검은 코드 변을 같은 값으로 취급하지 않는다.

## 데이터 계약

| 필드 | 의미 |
|---|---|
| source_stamp_s / reference_s | 원 image header 시각; packet 생성 시 서로 같음. transport가 원 취득 시각을 갱신하지 않음 |
| source_frame_id / frame_id | 명시 camera optical frame / `base_link` at image stamp |
| clock_domain | 실제 node의 `use_sim_time`에 따라 `ros_sim` 또는 `ros_system` |
| time_basis / timing_verified | `image_header_unverified_exposure` / false; 노출 시각 동기화 검증 아님 |
| vision_epoch / profile_id | VisionPipeline 세대 / 선택한 크기·K·외부파라미터 profile 식별자 |
| dictionary / id | 정수 ID만으로 사전을 합치지 않음 |
| rear_center_xy_m / heading_rad | 후방 마커 중심의 평면 추정; x 전방, y 좌측, 양의 heading 반시계 |
| polygon_xy_m | 뒤오른쪽→앞오른쪽→앞왼쪽→뒤왼쪽; 길이 .20, 폭 .15 m |
| black_code_size_m / printed_image_size_m / size_basis | 코드/인쇄 전체의 서로 다른 물리 변과 근거 |
| uncertainty | 명목 K/FOV·CAD·크기 근거; 거리/heading 보장 오차는 null, 통계 신뢰도 미교정 |
| mode | 항상 `approximate_v1`; `metric_accuracy_verified=false` |

명목 B0390 profile: 1280×720, HFOV 62.2°, camera body xyz `[.055, 0, .198363]` m/RPY 0. optical x우·y아래·z전방을 base x전방·y좌측·z위로 변환한다. 설치 위치·왜곡 교정 완료를 뜻하지 않는다. 알려지지 않은 crop/화면 비율 변경은 근사 입력으로 섞지 않으며, 실제 해상도와 동일 비율 resize만 허용한다.

최대 수명은 원 영상 시각에서 .25 s이다. 새 검출 결과가 오기 전 최근 양수만 짧게 사용하며, 최신 accepted 결과가 empty이면 그 선택 점유는 즉시 비워진다. 이는 차량 부재/자유 공간 증명이 아니다. 자차의 실제 wheel/gyro 이력으로 현재 base frame에 이동 보정하고 상대차 운동은 미보정으로 표시한다. 이력이 없으면 무보정 위치를 몰래 재사용하지 않는다. input boundary/시계 역행은 cache를 비운다.

## 선택 설정과 간이 조정

통합 이후 기존 planner와 `lidar_safety` 실행 명령의 **각 `--ros-args` 묶음 안**에 같은 절대 경로를 넣는다. 기존 센서·시각 정책·설치 상한·B3/STOP 설정은 유지한다.

```text
-p vehicle_marker_profile_path:=/절대/repo/config/perception/vehicle_marker_approx_b0390.json
```

simulation launch의 새 launch argument는 추가하지 않았다. 기존 실제 runtime ROS 실행 명령에 위 parameter를 명시하는 경로다. 기본 `local_pursuit`/비등록 runtime에 붙여 실제 연결된 것처럼 조용히 실행하지 않는다.

차량 완성 후 같은 규격의 마커와 한두 정면 거리 표본으로 간이 조정한다. 지금 장치/실물 조작은 수행하지 않았다.

1. 인쇄물의 검은 코드 변과 전체 변을 따로 확인한다. GIST 49와 실제 상대팀 마커의 크기를 혼동하지 않는다.
2. 마커가 정면을 향하고 코드 상단이 차량 상단인 저장 이미지를 얻는다. 아래 도구는 파일만 읽으며 카메라를 열지 않는다.
3. `black_code_pixels[].mean_horizontal_edge_px`와 **카메라 광학 중심에서 마커 평면까지 전방 거리**를 표본 JSON에 적는다. 범퍼 간격·기울어진 slant range를 그대로 쓰지 않는다. 템플릿의 null은 실측값으로 채우기 전 실행을 거절한다.
4. `fx_px ≈ pixel_width × optical_depth / black_code_size_m`. 2표본은 중앙값과 상대 차이를 기록한다. 픽셀 종횡이 같다는 근사로 fy=fx를 둔다. 대안 `range_scale`은 기존 추정 depth의 배율만 조정한다. 두 방법을 동시에 중복 보정하지 않는다.
5. 같은 전방 평면판을 LiDAR가 맞췄다면 `distance_basis=lidar_forward_same_plane`, `same_frontoparallel_plane=true`, `lidar_base_x_m`을 적어 `camera depth = LiDAR forward range + lidar_x - camera_x`로 환산한다. 현재 단순 도구는 body RPY=0에서만 이 offset식을 허용한다.
6. 선택 `estimated_heading_rad`/`reference_heading_rad`으로 heading offset을 조정할 수 있다. 왜곡/전체 외부파라미터 교정이나 정밀 정확도 보장으로 승격하지 않는다.

```bash
python3 scripts/inspect_vehicle_marker_image.py --profile config/perception/vehicle_marker_gist49_calibration_only.json --image /절대/저장사진.png --output /절대/새측정.json
python3 scripts/adjust_vehicle_marker_profile.py --profile config/perception/vehicle_marker_approx_b0390.json --samples /절대/실측표본.json --method focal_pixels --output /절대/새profile.json
```

도구는 기존 output을 덮어쓰지 않는다. 자기 마커 관측에 사용한 profile을 runtime으로 쓰지 않는다. GIST용 간이 K를 상대팀 profile에 반영할 때는 카메라 K/배율/heading만 옮기고 상대팀 dictionary/ID/코드 크기는 그대로 유지한다.

## 검증과 다음 한 묶음

- `windows_tests_v11.log`, `windows_tests.xml`: **197 passed, 2 deselected**. Windows Python 3.14/OpenCV, 기존 vision/observer/LocalFollow/M3/runtime backend 회귀 포함.
- 새 검사: 이상적 정면·비스듬한 projection, 실제 4×4/7×7 합성 픽셀, 크기 혼동, 미검출·사전/shape/clock/TTL/세대, 간이 조정 CLI, nested/exception ContextVar 복구, 실제 LocalFollow/M3 별도 sweep, 실제 Evaluator plan/current 함수 및 guard.control 본문(I/O double).
- 제외 2개는 Windows의 ROS import 검사이며 이후 실제 ROS에서는 통과했다. 실제 ROS 대표 전체는 정면 중복 quad 문제로 실패했다. native ARM·실차 주행은 검증하지 않았다. guard AST 본문 검사와 실제 ROS 결과를 구분한다.
- 중간 실패는 `windows_tests_v1`~`v8`에 보존했다. 초기 test 사본 경로 충돌, 합성 입력의 시각 공백/방향 기대값, ROS 미설치 및 IPPE rear-face 수치 문제가 포함된다. PnP는 일반 planar ITERATIVE 한 가지로 고쳤고 여러 solver fallback은 추가하지 않았다. `diagnose_pose.py`는 최초 IPPE 진단을 재구성하는 도구다.
- 실제 Linux 대표는 ROS import·VisionProcess→packet·독립 guard 부재/front/side/TTL·마지막0으로 좁혀 한 번 실행했다. 원 자료는 `execution_v1/run1`에 있고 정면 cap 실패를 보존했다. native plan/current 전체 재생은 이번 대표에 포함하지 않았다. 현재 중복 처리 수정과 재시험은 미실행이며 명시 재개를 기다린다.

`changes.json`, `source_integrity.json`, `stage.diff`, `stage_source.zip`이 이관 입력이다. 원본 SHA가 바뀌었다면 자동 덮어쓰지 않고 root가 충돌을 확인한다.
