# r554 추월·조각벽 경로의 방위 정렬 후속 확인

관련 `src/arena_autonomy` 함수만 읽었다. 공통 수정·시험·WSL/장치 실행 없음. 다음은 코드의 실제 연결 경로이며, r553 또는 교통 주행에서 새 실패가 관측되었다는 주장은 아니다.

## 같은 패턴이 남은 경로

1. **추월 최초 진입의 두 벽 기준:** `moving_pass.MovingPassGuide.evaluate`의 새 경로 시작 분기는 `straight_reference(scene, config)`를 호출한다(`moving_pass.py:296`). 이는 `bypass_entry_trial.observed_straight_wall`에서 한쪽 반환 subset을 `local_path`에 주지만 `bearings_rad`를 전달하지 않는다(`bypass_entry_trial.py:81~86,103~108`). 따라서 새 optional 인자 기본값 None→차체 원점 `atan2(endpoint)` 정렬→인접 거리 분할이 남는다. 가까운 가는 상대와 먼 벽 또는 벽 틈 너머의 먼 배경이 섞이는 관측에서 같은 조기 분할이 가능한 구조다. 실패하면 `moving_pass_two_walls_and_front_required`이며 상위의 기존 FOLLOW 유지 경로가 있으므로 곧바로 전 차량 정지와 동일시하지 않는다.
2. **교통 조각벽 가설:** `LocalFollow.observed_path`는 기본 local_path에는 실제 bearing을 전달하지만, `traffic_boundary_refresh_trial`이고 piecewise corner가 아닌 경우의 `fragmented_wall_hypothesis.propose` 호출에는 전달하지 않는다(`local_follow.py:832~835`). `propose`는 직접 endpoint atan2로 정렬한 뒤 같은 거리 상한으로 component를 나눈다(`fragmented_wall_hypothesis.py:19~22`). 가는 상대에 가린 벽 조각의 회복 후보가 그 영향을 받을 수 있다. 이 기능은 기본 False인 선택 기능이므로 모든 현재 실행에서 발생한다고 단정하지 않는다.
3. **현재 코너 회랑 보조:** `corridor_observer`→`corridor_roles.role_aware_evidence`→`corner_corridor.corridor_evidence`에서도 bearing 없는 local_path 호출과 `_components`의 endpoint atan2 거리 분할이 남는다(`corner_corridor.py:14~23,50`, `corridor_roles.py:19,29,35`). 이는 조각벽/교통 fallback 연결상의 같은 패턴이다. 특정 저장 입력에서 실패 원인인지는 이번에 추가 재생하지 않았다.

## 같은 원인이 확인되지 않은 경로

- **이미 시작한 MovingPassGuide의 외벽 재관측:** 생성 시 `tracked_wall_fit=True`를 선택하고 `EntryGuide._outer_wall_fit`→`bypass_wall_fit.tracked_straight_wall`로 간다. 이 함수는 실제 return 순서/`source_index`로 후보를 정렬한다(`bypass_wall_fit.py:55~57,79~82`). endpoint atan2 재정렬을 하지 않으므로 이번 parallax 오류와 같지 않다.
- **틈 연결:** 같은 파일 `small_wall_joints`는 원 광선 index 사이의 실제 UNKNOWN 여부·끝점 거리·양쪽 방향/잔차를 확인한다. 뒤의 먼 실제 반환을 UNKNOWN으로 바꾸거나 건너뛰지 않는다. 이 부분에 추가 수정 근거는 없다.
- **가는 상대 추적:** `moving_pass` tracker는 scene return의 실제 순서를 유지한 채 점 간격/index gap으로 cluster를 만든다(`moving_pass.py:116~122,203~220`). 차체 atan2 재정렬은 없다. 가는 부품의 부족한 반환/가림 자체와 이번 정렬 오류를 혼동하지 않는다.
- 경로 접선의 atan2 및 투영 호길이 순서(`boundary_evidence.fragment_evidence`)는 광선 방위를 대체하는 정렬이 아니므로 이번 오류로 분류하지 않았다.

## 판단

실행 중 소스는 그대로 둔다. **tracked 외벽/틈 연결/상대 tracker에는 추가 수정 없음.** 이후 교통 대표에서 진입 벽 소실 또는 조각벽 회복 실패가 실제 관측되면, 첫 우선 후보는 scene을 이미 받는 `observed_straight_wall`에 실제 return bearing을 같은 subset으로 전달하는 작은 연결이다. 나머지는 scene→부분집합 bearing 전달이 필요한 후속 범위다. 세 경로를 한 번에 넓게 수정하거나 실제 실패 없이 새 관문/시험 행렬을 추가할 근거는 없다.
