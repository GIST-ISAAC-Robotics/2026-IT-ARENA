**어떤 파일에서**

`track/track_gen.py:write_sdf()`, `track/output_final/world.sdf`의 `surface_main_*` 및 `grid_slot_0`~`grid_slot_5`

**무엇이 이상한가요**

그리드 시각 형상의 상단 z=0.0017 m가 노면 상단 z=0.0030 m보다 낮아 출발 슬롯 표시가 노면 아래에 묻힙니다.

**환경**

- 재현 환경: Windows 호스트의 WSL Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.11.0
- repository: `MOSW626/istech-it-arena`
- commit/release: `d61c5db9252cedfbc163cd044a47671df91e1660` / `v2026.08.31`
- 확인 파일: `track/track_gen.py`, `track/output_final/world.sdf`, `track/output_final/scene.json`

## 재현 방법

1. `world.sdf`의 `surface_main_*` pose와 box z size를 확인합니다.
2. `grid_slot_0`~`grid_slot_5`의 link pose와 visual box z size를 확인합니다.
3. 각 box의 상단 z를 계산하거나 Gazebo에서 출발 구역을 상단에서 관찰합니다.

## 기대동작

- 그리드 표시는 노면보다 약간 위에 있어 z-fighting 없이 보여야 합니다.
- 표시는 시각 형상이어야 하며 주행 접촉을 바꾸는 충돌체가 없어야 합니다.

## 실제결과

- 본선 노면은 중심 z=0.0015 m, 두께 0.003 m이므로 상단 z=0.0030 m입니다.
- 그리드 visual은 link 중심 z=0.0012 m, 두께 0.001 m이므로 상단 z=0.0017 m입니다.
- 표시 전체가 노면 상단보다 아래에 있어 정상적인 깊이 검사에서 가려집니다.
- 그리드에 collision은 없다는 점은 의도에 맞습니다.

## 영향

- 6개 공식 출발 슬롯을 화면에서 확인하기 어렵습니다.
- 카메라 기반 grid 확인, 운영자 배치 확인, README/preview와 실제 Gazebo 화면의 일관성이 떨어집니다.

## 근거자료

- [공식 생성기의 노면 높이](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L1651-L1654)
- [공식 생성기의 그리드 높이](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L1681-L1691)
- [배포 SDF의 첫 번째 그리드 visual](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/output_final/world.sdf#L14982-L14989)
- [z 수치 감사](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/blob/main/docs/track/OFFICIAL_SOURCE_AUDIT.md)

![공식 배포 월드의 전체 배치 참고 화면](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/upstream_full_overview.png)

그리드가 묻히는 직접 근거는 SDF의 중심 z와 두께 비교입니다. 전체 화면만으로 1.3 mm 높이 차이를 측정했다고 주장하지 않습니다.

## 수정제안

1. 그리드 visual의 하단이 노면 상단보다 소폭 높도록 중심 z를 계산합니다. 예: `road_top + epsilon + thickness/2`.
2. collision은 계속 생성하지 않습니다.
3. epsilon을 과도하게 키워 차체 카메라에 떠 보이지 않도록 렌더링 시험을 추가합니다.
4. 공식 슬롯 0.25 × 0.17 m와 실제 슬롯 pose는 그대로 유지합니다.

## 완료 체크리스트

- [ ] 여섯 슬롯이 Gazebo 상단/사선 시점에서 보임
- [ ] z-fighting 없음
- [ ] grid collision 없음
- [ ] 슬롯 치수와 pose 보존
- [ ] scene·preview·실제 월드 표시 대응 확인
