**어떤 파일에서**

`track/track_gen.py:write_sdf()`, `track/output_final/world.sdf:bump_0`, `track/output_final/scene.json:speed_bumps`

**무엇이 이상한가요**

진행 길이 0.05 m·횡단 폭 0.45 m의 명목 치수와 반대로 SDF가 진행 방향 0.45 m·횡단 0.05 m 상자를 만듭니다.

기존 [#2 과속방지턱 3D 프린팅 파일 공유](https://github.com/MOSW626/istech-it-arena/issues/2)도 주행 방향 길이 0.05 m를 명시합니다. 이 이슈는 실물 제작 진행을 중복 요청하는 것이 아니라 현재 명목 치수가 SDF로 내보내질 때의 축 반전을 다룹니다.

**환경**

- 재현 환경: Windows 호스트의 WSL Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.11.0
- repository: `MOSW626/istech-it-arena`
- commit/release: `d61c5db9252cedfbc163cd044a47671df91e1660` / `v2026.08.31`
- 확인 파일: `track/track_gen.py`, `track/output_final/scene.json`, `track/output_final/world.sdf`

## 재현 방법

1. `scene.json`의 `speed_bumps.bumps[0]` pose와 `bump_length_m`, 해당 위치의 본선 yaw를 확인합니다.
2. `world.sdf`의 `link[name=bump_0]` pose와 `collision/geometry/box/size`를 확인합니다.
3. 생성기의 `write_sdf()`에서 `bmp["width"]`, `bmp["length"]`가 `_sdf_box_link()`의 로컬 X/Y 인수로 전달되는 순서를 확인합니다.

## 기대동작

- `bump_length_m=0.05`는 진행 방향인 링크 로컬 X 길이여야 합니다.
- 방지턱의 횡단 폭은 본선 폭 0.45 m로 링크 로컬 Y에 놓여야 합니다.
- 즉 box size는 진행 X=0.05 m, 횡단 Y=0.45 m, 높이 Z=0.01 m여야 합니다.

## 실제결과

- `world.sdf`의 `bump_0` box size는 `0.4500 0.0500 0.0100`입니다.
- 링크 yaw는 도로 진행 방향이므로 방지턱이 도로를 가로지르지 않고 진행 방향으로 긴 상자로 생성됩니다.
- 생성기에서 `bmp["width"]`를 X, `bmp["length"]`를 Y로 전달하여 의미가 반전됩니다.

## 영향

- 차량이 의도한 5 cm 길이의 횡단 방지턱을 밟는 접촉 시험이 되지 않습니다.
- preview의 과장 표시와 SDF 실제 충돌 형상이 달라 시각 확인만으로 문제를 놓치기 쉽습니다.
- 차량 피치·접지·속도 제한 시험 결과가 실제 배치 의도와 달라집니다.

## 근거자료

- [공식 생성기의 X/Y 전달 순서](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L1677-L1679)
- [배포 SDF의 방지턱 pose와 box size](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/output_final/world.sdf#L14969-L14979)

![원본 SDF의 진행 방향으로 긴 방지턱](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/upstream_bump_axis.png)

![도로 진행축에 투영한 배포 형상과 명목 치수의 비교](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/upstream_bump_axis_plan.png)

첫 번째 화면은 로드에 필요한 호환성 변환을 적용하되 좌표·치수를 변경하지 않은 실행 캡처입니다. 두 번째 그림은 원본 SDF의 좌표·치수를 계산해 그린 축 비교 도식이며, 화면 캡처와 구분합니다.

## 수정제안

1. `write_sdf()`에서 방지턱 box size의 X/Y를 `bmp["length"], bmp["width"]` 순서로 전달합니다.
2. DXF·preview·SDF가 동일한 진행/횡단 정의를 쓰는 회귀 검사를 추가합니다.
3. `0.05 m`가 최종 실물 방지턱의 진행 길이인지 담당 동아리의 후속 3D 프린팅 자료와 다시 대조합니다.
4. 실물 단면이 공개되기 전이라면 현재 상자/코사인 등 시뮬레이션 단면을 placeholder라고 명시합니다.

## 완료 체크리스트

- [ ] SDF 로컬 X=진행 길이 0.05 m
- [ ] SDF 로컬 Y=횡단 폭 0.45 m
- [ ] box 중심과 최고점의 z 기준 명시
- [ ] preview·DXF·scene·SDF 치수 일치
- [ ] 실제 방지턱 후속 자료 대조 상태 문서화
