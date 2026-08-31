**어떤 파일에서**

`track/track_gen.py:markers_from_design()`, `track/output_final/world.sdf`, `track/output_final/scene.json`, `track/output_final/branch_1.csv`

**무엇이 이상한가요**

README는 코스 마커를 벽에 부착한다고 설명하지만 실제 판이 벽 안쪽에 떠 있고, ID 30 판이 대회 규격 차량의 지름길 2 중앙 통과 경로를 막습니다.

**환경**

- 재현 환경: Windows 호스트의 WSL Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.11.0
- repository: `MOSW626/istech-it-arena`
- commit: `d61c5db9252cedfbc163cd044a47671df91e1660`
- release: `v2026.08.31`
- asset: `it_arena_track_v2026.08.31.zip`
- asset SHA-256: `897183D2E2541458D190A0A1E3F76BFF754C67FAB894F2302C3110830A86149B`
- 확인 파일: `track/design_final.json`, `track/track_gen.py`, `track/output_final/world.sdf`, `track/output_final/scene.json`, `branch_1.csv`
- 검사 차량 평면 외형: 대회 규격 길이 0.20 m × 폭 0.15 m

## 재현 방법

1. 릴리스 ZIP을 풀고 `output_final/world.sdf`의 `walls_*` 충돌 상자와 `aruco_*` 판 충돌 상자를 읽습니다.
2. 각 마커 판 중심과 실제 벽 충돌 폴리곤 사이의 최소거리를 계산합니다.
3. `design_final.json`으로 `track_gen.py --design` 경로를 재생성하거나 `branch_1.csv`를 읽습니다.
4. 지름길 2 중심선의 각 표본에 0.20 × 0.15 m 직사각형 차량 외형을 진행 방향으로 놓고, SDF의 저층 충돌체와 면적 교차를 검사합니다.

## 기대동작

- README의 설명처럼 네 ArUco 판이 지정된 측면의 실제 벽면에 부착되어 있어야 합니다.
- 폭 0.20 m로 공개된 지름길 2를 폭 0.15 m 차량이 중앙 정렬로 통과할 때 마커 판과 충돌하지 않아야 합니다.
- `scene.json`의 pose와 `world.sdf`의 실제 충돌 pose가 같은 수정 결과를 가리켜야 합니다.

## 실제결과

- 판 중심에서 벽 폴리곤까지의 최소거리는 ID 0/20/45가 각각 약 0.070 m, ID 30이 약 0.0964 m입니다. 즉 README의 벽 부착 설명과 달리 판이 벽면에서 떠 있습니다.
- `aruco_30`은 지름길 2의 차량 외형과 겹칩니다. `branch_1`의 시작점 기준 s≈0.262509~0.444246 m, 표본 index 13~22의 10개 표본에서 재현됩니다.
- 같은 검사에서 본선 2,332개 표본, 지름길 1의 155개 표본, 출발 슬롯 6개는 이 문제로 막히지 않았습니다.
- 공식 `branches_ok`는 분기 중심선의 반경·자기교차만 확인하여 시설 충돌을 발견하지 못합니다.

## 영향

- 공개된 지름길을 대회 규격 차량이 사용하지 못하거나 시뮬레이션 충돌이 발생할 수 있습니다.
- 마커의 실제 거리·방향이 README와 달라 카메라 인식 거리 및 예상 픽셀 크기가 달라집니다.
- 공식 검증 결과가 PASS인 것으로 오해하기 쉽지만 시설 포함 통과 가능성은 검사되지 않습니다.

## 근거자료

- [공식 마커 배치 함수](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L524-L542), [공식 ID 30 충돌체](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/output_final/world.sdf#L15140-L15152)
- [수치 감사](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/blob/main/docs/track/OFFICIAL_SOURCE_AUDIT.md)

![원본 좌표를 유지한 ID 30과 지름길 2의 실제 실행 화면](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/upstream_marker_id30.png)

![릴리스 branch_1.csv 141표본과 20x15 cm 외형을 대조한 간섭 도식](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/upstream_id30_collision_plan.png)

도식은 릴리스 CSV의 141개 표본과 SDF 판을 계산해 그린 정량 검사 결과이며 Gazebo 화면 캡처가 아닙니다. 10개 충돌 표본을 확인했습니다. 실행 화면은 로드에 필요한 호환성 변환을 적용했지만 좌표·치수는 변경하지 않았습니다. 화면의 검은 마커 판은 정상 코드 렌더링 성공의 근거가 아니며 재질 문제와 구분합니다.

## 수정제안

1. 각 마커의 `side`와 경로상 `s`를 기준으로 해당 측면의 실제 wall-union 경계를 찾습니다.
2. 판 뒷면을 벽면에 접하도록 pose를 옮기고, 판 법선이 트랙 쪽을 향하도록 yaw를 설정합니다.
3. 수정 pose를 `scene.json`과 `world.sdf`에 함께 기록합니다.
4. 본선·두 지름길 전체에서 0.20 × 0.15 m 차량 외형과 모든 저층 시설의 정적 간섭 검사를 추가합니다.
5. ID 30은 단순히 반대쪽으로 임의 이동하기 전에 설계의 `side=left` 의도와 벽 부착 상태에서 간섭이 해소되는지 확인합니다.

## 완료 체크리스트

- [ ] 네 마커가 실제 벽면에 접함
- [ ] 네 마커의 앞면이 트랙을 향함
- [ ] ID·dictionary·경로상 s·인쇄 크기 기준이 보존됨
- [ ] 본선과 지름길 1·2의 0.20 × 0.15 m 정적 외형 검사 통과
- [ ] `scene.json`과 `world.sdf` pose 일치
- [ ] preview/DXF/README가 수정 결과와 일치
