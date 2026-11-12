# 공통 관측 정책의 ROS·계산 프로세스 연결

11/12 `g1_guard_observation_policy:=selected_r181`로 r190 공통 정책을 선택하도록 연결했다. 기본은 `original_v1`이며 기존 교통 GUARD 설정이 있어야 선택할 수 있다. 프로세스 자식은 직렬화된 `FollowConfig`를 통해 같은 정책을 받는다. `g1_current_path_recheck:=selected_v1`을 함께 선택하면 기존 r181의 현재 접근 관측 저속 재검사도 공통 함수로 사용한다.

평가 지연·r186 경로 참조 보존은 포함하지 않았다. 기본값/실물 시간 관문·독립 보호·STOP·기존 관측/간격·속도 조건은 변경하지 않았다. 새 정책의 완주 성립이나 실차 배포가 아니다.

- r182 현재 검사409개·r187 318개, 합727반환이 실제 기록과 모두 같다(시험 종료 뒤 호출 포함).
- 실제 ROS 노드의 원/선택 정책6조합 STOP·RESET·계산 프로세스 종료, 기본 OFF473반환/기록 동등성, 현재193반환 등 관련48검사를 통과했다.
- `arena_autonomy`, `arena_bringup` 빌드와 `--show-args`를 완료했다.
- 전체 소스 계보199검사를 통과했다(`provenance.log`).

근거는 `rebase_records.json`, `tests.log`, `build.log`, `launch_args.log`, 변경 전 `before_sources.zip`과 `config/tests/guard_policy_node_r191_sha256.json`이다. 설정 전달과 저장 재현 검증이며 새 Gazebo/Jetson 주행은 아니다.
