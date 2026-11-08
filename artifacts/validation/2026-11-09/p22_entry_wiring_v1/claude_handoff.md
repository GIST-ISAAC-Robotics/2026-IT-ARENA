# Claude 인계 (5시간 사용량 경고 0.90, 2026-11-09)

상태: **조사만 완료, 저장소 파일 변경 없음.** 하위 에이전트·Gazebo·재생 실행 없음. Luna replay_v2 결과는 GPT 전달 내용만 참고했다.

## 확인한 연결 사실
- `LocalPursuit.command`는 `self.follow.command(scene, history, speed, side, last_steering, dt, raw_scan=...)`를 호출하고, `self.follow.config.partial_observation_policy`를 읽는다. `P22Entry.config`가 baseline config이므로 호환된다.
- 예외(MotionUnavailable/ValueError) 경로는 `self.follow.reset()` 후 `controller_mode(self.follow)`를 기록한다. 현재 `controller_mode`는 P22Entry를 `'g1_follow'`로 오인한다(follow_trace.py 수정 또는 노드 재정의 필요).
- **결정 필요:** 진입 경로 시작 뒤 baseline/scene 입력 예외나 자동 상태(SENSOR_STOP/MOTION_STOP/SCAN_INVALID)의 `reset_follow`가 `P22Entry.reset()`으로 `latched_fault`와 경로를 지운다. 그 뒤 본선 LocalFollow가 지름길 입구 중간 자세에서 재개될 수 있다. 제안: 경로 시작 뒤 자동 리셋·입력 예외는 래치 HOLD로 남기고, 운영자 `/reset`·DISABLED만 해제(노드 쪽 처리, p22 모듈 설계 불변).
- `P22Entry.command`는 경로 활성 중에도 `baseline.command`를 먼저 호출하므로 baseline 예외가 활성 경로를 중단시킨다(위 결정과 같은 문제).
- `vision_pipeline.pump` 결과의 `result['stamp']`(취득 시각)·`vision_pipeline.epoch`·`vision_pipeline.ids`를 `LocalPursuit.control`에서 쓴다. P22 노드는 `control()` 재정의에서 새 result일 때만 `self.follow.frame(ids, stamp, now, epoch)`를 호출하면 된다. 중복 프레임은 `BranchIntent.frame`의 `stamp <= last_frame` 검사로 이미 제외된다.
- `FollowTrace`는 mode 화이트리스트가 있고 `replay_events`는 미등록 header를 `unsupported trace header`로 거부한다. schema 6 P22 header를 추가하면 기존 replay는 자동 거부된다.
- 상태 가시화는 `WallFollow.control`의 `status.update(details)`이므로 G1G2Node처럼 `command()`에서 detail을 축약해 `p22_trial` state/reason/start/remaining을 남기면 된다. `path_points` 전체를 /autonomy/status에 싣지 않도록 축약 필요.
- launch: `src/arena_bringup/launch/simulation.launch.py`(58개 계약에 등장).

## 출처 해시 영향
- `follow_trace.py`(19), `setup.py`(3), `simulation.launch.py`(58), `validate_basic_autonomy.py`(31), `local_pursuit.py`(43), `g1_g2_node.py`(22) 계약에 등장. 수정 시 r48(`config/tests/observed_corner_continuity_r48_sha256.json`, parent r47) 뒤 새 successor 계약과 before 스냅숏, `tests/test_source_provenance_r0_plus_b3.py`의 `M3_SUCCESSORS +=` 한 줄 추가가 필요하다. r48 계약은 고치지 않는다.
- 새 모듈(p22_entry_trial_node.py, p22 trace)은 `new_dependencies`로 등록.

## 미완료 (전부)
1) p22_entry_trial_node.py·setup entrypoint 2) schema 6 trace/replay 거부 3) launch `p22_entry_trial` 옵션·runner 4) 관련 검사 5) 해시 사본·successor 계약·README.
시험 실행 방법: WSL에서 `artifacts/validation/2026-11-07/track_migration_v1/run_tests.sh <label> <files>`(PYTHONPATH 포함). Windows Python으로 파일 편집 시 utf-8/LF 유지.
