# observed_straight_wall 단일 stage 후보

`stage/src/arena_autonomy/arena_autonomy/bypass_entry_trial.py`에서 **observed_straight_wall 한 곳만** 변경했다. 공통 원본은 `original/`, 변경 patch는 `observed_straight_wall.patch`, SHA는 `stage_manifest.json`이다. corridor/fragmented 경로는 건드리지 않았다.

전체 return 점 순서와 같은 순서로 `direction_xy`의 bearing 배열을 만들고, 기존 `sign*y>.10` 마스크를 점과 bearing에 똑같이 적용한다. 좌표 사전을 새로 만들지 않아 동일 좌표의 복수 반환도 각 원 광선과 짝을 유지한다. 원 scene/점유/UNKNOWN을 수정하지 않는다.

`check_stage.py`는 stage 파일을 명시 경로로 로드하며 실제 `candidate.__file__`를 확인한다. 같은 현재 공통 `local_path`를 사용한 Windows NumPy 2.5.4 순수 straight-wall 검사 결과는 `stage_check_result.json`에 있다.

| 대표 | 기존 | stage | 원시 관측 보존 |
|---|---|---|---|
| sensor 원점 (0,0) | slope .3, fit 유효 | fit tuple/진단 모두 정확히 동일 | 상대 상부 19점 포함 전체 211점·UNKNOWN289 불변 |
| sensor 원점 (.066,.002) m | wall_segment_short | slope .3, residual 2.50e−16 m, 관측 경로 x −.06424~.49609 m | 상대 상부 22점 포함 전체 213점·UNKNOWN287 불변 |

두 깊이 면의 원점 기준 정렬은 두 번째 대표에서 near/far가 11번 교차한다. 실제 광선 방향 정렬은 한 번만 바뀐다. 상대 상부 15×20 cm 상자는 광선 교차/반환 보존 확인에만 사용했으며 label은 제어 입력으로 전달하지 않았다. 치수/배치는 합성 민감도이며 실제 교통 성공 증거가 아니다. piecewise/ROS/WSL/장치/폐루프 실행은 0이다.

현재 r553 실제 저장 header의 execution_config는 `moving_pass_trial=true`, `moving_pass_profile=medium_v1`, `entry_policy=corridor_local_v1`, `initial_reference_policy=ready_directional_pair_v2`다. 따라서 기능이 꺼진 사장 경로는 아니다. `MovingPassGuide.evaluate`의 새 path 진입→`straight_reference`→이번 함수로 직접 연결된다. 이미 진행 중인 추월 외벽은 기존 `tracked_straight_wall`을 쓴다. r553 단독 주행에서 이 함수가 실제 호출됐다는 trace 증명은 하지 않았으며, 다음 교통 묶음 채택 여부는 root가 판단한다.

stage SHA256: `354239c472dc95a116259ec3051239bdb945feddea30f8c51f3a08b788971185`.

원본 공통 SHA256: `7ec85a945bc78d6bde801e0f6ab2ff5951473e0506098f728164e705438158ec`.
