# 실제 광선 방위 정렬 후보

공통 파일·장치·WSL 변경/실행 없음. 최종 제안은 `stage2/`의 3파일이며, 먼저 전달한 `stage/`와 공통 원본 `original/`은 보존했다. **임시 `minimum_ahead_m` API를 제거하고 원래 0.28 m 기준을 복구**했다. 신규 인자는 `local_path(..., bearings_rad=None)` 하나다. SHA는 [manifest_stage2.json](manifest_stage2.json), 공통 원본 대비 패치는 `*.stage2.diff`에 있다.

## 원인과 정상 LocalFollow의 거절

- r544 attempt2 첫 정지의 반환점 source 166~172는 같은 근거리 벽 위에 있다. source 173에서 실제 반환 깊이가 약 0.567→2.464 m로 뛰며 먼 배경이 관측된다. UNKNOWN·자기 가림·무반환이 아니다.
- 원래 `atan2(endpoint.y, endpoint.x)`는 **차체 원점**의 각도다. 실제 보정 광선 원점은 약 `(0.066, 0.002)`이다. 이 차이로 차체 각도 순서가 `166(벽),173(배경),167(벽),174(배경)…172(벽)`가 된다. 첫 간격 1.93818 m가 연결 상한 0.11959 m를 넘으며 벽을 source 166에서 자른다.
- `narrow_occluder_bridge=False`로 해도 똑같이 0.254690 m에서 끊긴다. 이후 기둥 처리기가 source 167~172를 제거하는 것은 이미 잘못 섞인 순서의 2차 효과다. 기둥 처리기 전체를 끄는 수정은 해결책이 아니다.
- 첫 정지의 정상 계획은 `boundary_candidate_reason=wall_segment_short`, `corridor_candidate.reason=long_outer_reference_unavailable`이다. 오른벽은 길지만 기본 다항 경로의 `wall_distance_m=.651571810`, offset `.505`로 anchor 오차 `.146571810 > .12`여서 corridor가 거절한다. helper 오른벽의 별도 path anchor `.130350 > .12`도 같은 방향의 근거이나 정상 1.8 m piecewise 경로와 동일 수치라고 취급하지 않는다. 정확한 정상 오른벽 piecewise 값은 native 재생에 포함했다.
- 정상 LocalFollow는 반대벽을 무조건 새 경로로 채택하지 않는다. `sufficient()`의 길이/anchor와 기존 경로·선호벽 지지가 필요한 구조다. 현재 변경은 이 기준을 풀거나 선호 방향을 바꾸지 않는다.

## 후보와 Windows 확인

`current_wall_candidate`는 `occupied_endpoints_xy_m`와 같은 return 순서로 각 `direction_xy`의 atan2를 전달한다. `LocalFollow.observed_path`는 실제 endpoint→bearing 사전을 사용해 후보 부분집합에도 대응시킨다. 기존 `rays` 번호는 연결 간격용으로 유지한다. raw 점유, ray 원점·취득시각·UNKNOWN, gap/fit/기둥·slew·제동 검사는 변경하지 않는다.

| 저장 current 입력 | 기존 관측 ahead | 실제 방향 정렬 ahead | 다항 pursue 속도/조향 |
|---|---:|---:|---:|
| last_positive | .300646 m | .359240 m | .1 m/s, −.201499 rad |
| first_zero_discarded | .278353 m → 원 .28 거절 | .331998 m | .1 m/s, −.157041 rad |
| first_zero_candidate | .254690 m → 원 .28 거절 | .311140 m | .1 m/s, −.142578 rad |

[check_windows.py](check_windows.py)의 `python check_windows.py stage2` 결과는 [windows_result_stage2.json](windows_result_stage2.json)이다. NumPy 2.5.4의 2D `np.cross` 제한 때문에 **piecewise=False 순수 경로/추종만 실행**했으며 환경을 패치하지 않았다. 전체 helper 양수·독립 guard 허가·폐루프 완주 검증이 아니다.

소수 반례: 원점 0에서는 출력 배열/정보 완전 동일, 같은 두 깊이 벽의 고정 광선 원점 60/75 mm 및 취득 중 연속 15 mm 원점 변화에서 앞부분 보존, 실제 ray 교차로 생성한 폭 10 mm 기둥은 기존 bridge 1개 유지, 폭 150 mm 실제 차량 면은 제거하지 않음, 잘못된 bearing 길이/NaN 거절. 모든 점유 배열 불변을 검사했다. 기본값 None의 기존 경로도 유지한다.

초기 `windows_result.json`에는 원점을 매 광선마다 15 mm 왕복시킨 비현실적 진단도 보존되어 있다. 이 입력은 기존 호길이 계산을 .65 m까지 부풀릴 수 있다. 검증된 연속 운동 입력의 통과 근거로 사용하지 않았으며, `windows_result_stage2.json`은 고정 두 원점과 연속 원점 이동을 각각 검사한다. 방향 정렬 자체가 임의 운동의 벽 연속성을 인증하는 것은 아니다.

## 실제 벽 끝과 재정지 가능성

[depth_jump_evaluation.json](depth_jump_evaluation.json)은 저장 평가 자세 38.569/38.671 s를 38.620 s에 선형 보간해 **평가에만** 사용했다. source 166~172는 `walls_main_left_21`의 수평 면 및 그 끝의 `walls_main_left_20` 모서리에 맞으며, source 173은 약 0.55 mm 잔차로 `walls_main_left_126` 먼 배경에 맞는다. exact ray collider ID 기록은 아니므로 모서리 시야 설명은 저장 기하에 근거한 해석이다.

조기 절단은 약 56.45 mm 회복되지만 실제 모서리 이후 같은 면이 무한히 관측되지는 않는다. 3개 저장 입력만으로 정상 경로 갱신/다시 정지를 단정할 수 없다. 수정 후 ahead도 .359→.332→.311 m로 줄어든다. 기존 .207 완화의 첫 입력 여유 47.69 mm, 이 후보의 원 .28 여유 31.14 mm는 **다음 관측의 진행 보장 수치가 아니다**. r550 폐루프 결과는 root가 별도로 판정한다.

## native 전달과 테스트 영향

소유권 회수 뒤 root/runner가 아래 스크립트만 실행할 수 있다. 코드 설치 없이 3파일을 프로세스 읽기 overlay하며 저장 compute/current 6입력의 helper, 원시 점유 불변, 정상 오른벽 anchor를 기록한다.

```text
python replay_native.py --mode original28
python replay_native.py --mode stage --candidate-dir stage2
```

테스트 영향은 다음과 같다. 공통 테스트는 수정하지 않았다.

- `tests/test_current_wall_reacquire.py:180`의 `left_end=.26` 긍정 검사는 임시 길이 완화 전제이므로 최종 후보에서는 다시 `None`이어야 한다. 실제 장애물·직전 조향·선호 방향·raw 보호 반례는 그대로 유지한다.
- `LocalFollow.local_path`의 strict mock에 새 keyword를 받는 조정이 필요하다: `test_corner_follow.py:38`, `test_corner_distance_follow.py:34,113`, `test_fragmented_follow.py:85`. 마지막 wrapper는 `**kwargs`를 원 함수에도 전달해야 한다.
- `test_current_wall_refresh`, `test_solo_retained_opposite_corner`, `test_current_wall_reacquire`의 recorder 및 `scripts/analyze_follow_trace.py`는 이미 `**kwargs`를 받는다. corridor 모듈 전용 mock은 새 호출 경로가 아니므로 변경 필요 없음.
- 실제 subset endpoint→bearing 매핑의 통합 검증은 native 관련 회귀에서 확인해야 한다. 동일 좌표의 중복 반환은 기존 endpoint→source_index 매핑처럼 사전의 마지막 항목을 사용한다. 큰 새 추적기/관문은 추가하지 않았다.
