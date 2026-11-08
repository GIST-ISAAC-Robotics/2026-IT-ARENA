# r50 P22 입출구 연결 검토 — 중단 인계

5시간 사용량 경고로 검토 도중 중단했다. 아래는 코드 읽기로만 확인한 내용이며 실행·재생·단위 검사는 하지 않았다. `evaluate_live.py`·`test_evaluation.py`는 만들지 않았다.

## 읽은 범위

- `src/arena_autonomy/arena_autonomy/p22_trial.py`, `p22_route_trial.py`, `p22_exit_path.py`, `p22_entry_trial_node.py`, `follow_trace.py`
- `local_pursuit.py`의 `reset_follow`/`on_reset`/`command`(예외 경로·`raw_scan` 전달)
- `scripts/validate_basic_autonomy.py`의 `--p22-route-trial` 인자, 입력 해시, `p22_route_end`/`p22_entry_end` 종료 분기
- 이 디렉터리의 `PLAN.md`·`replay.py`·`prepare.py`·`run_live.py`, r49 `evaluate_live.py`·`live_summary.json`

읽지 않음: `p22_intent.py`(`BranchIntent`), `p22_observation.py`, `p22_path.py`, `tests/test_p22_route_trial.py`, `simulation.launch.py`, r49 `follow_decisions.jsonl` 내용.

## 코드상 확인한 점

- 입력 epoch 변경 시 활성 경로가 있으면 강제 초기화 후 `epoch_reset` 래치, 영상 epoch 변경 시 `vision_epoch` 래치. 자동(비강제) reset은 경로/래치가 있으면 `active_input_reset`으로 래치하고 `P22Route` 단계 상태를 보존. 명시 RESET(`autonomy_reset`)만 강제 해제.
- 진입 허가 뒤 `path`가 해제되지 않으므로 재무장되지 않는다(입구 1회).
- 출구 전환: 같은 `source_first_s` 재사용 금지, 두 판단의 `t_ref_s` 간격 0.05–0.20 s, 이전 후보 꼬리를 측정 운동으로 이송해 종점 8 cm·방향 8° 비교. 입구 꼬리 방향 8°·조향 0.14 rad 조건 미충족 시 이전 후보 폐기.
- 본선 인계: 서로 다른 스캔 두 개에서 기본 주행기 양수 명령 확인 뒤 `main`. `main`에서 `raw_scan`이 기본 주행기로 전달된다.
- `route_complete`는 `main_progress_m>=1.2`(제어기 휠·자이로 이력 누적)이며 검증기는 이 값으로 `p22_route_end`를 남기고 종료한다. 이는 평가용 실제 진행 1 m와 다른 값이다.

## 확인 필요 사항(심각도 추정, 미재현)

1. **재생 영상 입력 비대칭(중간, 재생 충실도).** 실시간 노드는 매 판단마다 `follow.frame()`을 호출하지만 `p22_frame` 사건은 `(epoch, stamp, ids)`가 바뀔 때만 기록한다. `P22Entry.reset()`은 `self.vision=None`으로 지운다. 경로가 없는 상태에서 자동 reset(`reset_follow`)이나 `follow_input_stop` 예외 reset이 일어나면, 실시간은 다음 판단에서 같은 영상 입력을 다시 받지만 재생은 키가 바뀔 때까지 영상 입력이 없다. `BranchIntent.frame`이 같은 stamp 재입력에 반응하는지 확인하지 못했다. 재생 불일치로 드러나므로 평가가 잘못 통과할 위험은 낮다. 노드 `on_reset`만 `p22_recorded_frame=None`으로 초기화한다.
2. **B3 경기 감독 사용 시 reset 강도 불일치(낮음, 단독 시험에 경기 감독이 없으면 해당 없음).** `P22EntryNode.on_reset`은 강제 reset 후 상위 `on_reset`을 부르고, 경기 감독이 있으면 `legacy_reset` 사유로 기록된다. 재생은 `autonomy_reset`만 강제로 처리하므로 활성 경로 중 RESET이면 실시간(해제)과 재생(래치)이 갈린다. 이번 등록 인자에 경기 감독이 켜지는지 확인하지 못했다.
3. **본선 인계 측 고정(설계 확인).** 출구 대기와 `main`에서 기본 주행기를 `'left'`로 호출한다. 진입 전에는 노드의 `side`를 쓴다. 의도한 선택인지 확인 필요.
4. **`main` 중 기본 주행기 예외(낮음).** `_active`가 예외를 잡아 래치 없는 `_hold`만 반환하고 기본 주행기를 reset하지 않는다(노드 예외 경로는 reset함). 다음 판단에서 `main`을 그대로 계속한다.
5. **종료 분기(평가 해석 참고).** `active_input_reset`/`epoch_reset`/`vision_epoch` 래치나 `entry_command` 오류 정지는 검증기 종료 분기에 없어 `--max-sim-seconds`까지 정지 상태로 대기 후 시간 초과로 끝난다. 실패 판정은 유지되지만 평가기는 `report.error`와 마지막 사유를 함께 기록해야 한다. `_hold` 상세에는 `phase`가 없다.

## 평가기 작성 시 참고(미구현)

- schema 7·`controller_mode=p22_route_trial` 확인, `P22Route(LocalFollow(FollowConfig(**follow_config)))`, `p22_frame`→`frame`, `reset`→`reset(force=reason=='autonomy_reset')`, `call`→`replay_call` 후 `comparable_decision` 비교. footer의 `accepted/written/dropped/failed/pending/worker_alive/capture_errors`와 사건 `seq` 연속성 확인.
- 전이 시각은 재생 출력 `p22_trial.first_command_admitted`, `phase`, `merged_s`, `route_complete`에서 얻고 최종 좌표만으로 완료를 판정하지 않는다.
- r49 `evaluate_live.py`의 50 Hz 표본·명령 조향 기반 접지면 평가·벽 겹침 계산을 재사용할 수 있다.
