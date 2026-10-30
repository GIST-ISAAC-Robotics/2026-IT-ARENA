# 관측 직선 차선 선호 — 명시 정책 연결

`FollowConfig.rear_lane_keep_policy`와 ROS `g1_rear_lane_keep_policy`를 추가했다. 기본은 기존 `timed`이며, `rear_lane_keep_trial=true`와 `observed`를 함께 명시한 경우에만 새 후보를 사용한다. 새 정책의 실행·재생 선택은 판단 기록 헤더에 남는다. 기본 채택은 아니다.

새 선택기는 [오프라인 v3](../rear_lane_preference_shadow_v3/README.md)와 설명/모듈 경로만 다르다. 현재 직선에서의 경로 유지와 현재 관측에 근거한 속도 제안, 엔코더 목표 여유를 포함한다. 기존 2초 후보는 수정하지 않았다. `observed_parallel_speed_trial`은 기존 평행 제안의 목표 상한만 명시 인자로 받는 시험 복사본이며, 기존 제동·점유·횡이동 조건은 유지한다.

## 출처와 검사

`before_connection.zip`은 연결 직전 파일, `config/tests/observed_lane_preference_r1_sha256.json`은 이전→현재 변경의 근거다. 과거 계약을 덮어쓰지 않는다. `connected_sources*.zip`은 검사 단계별 소스 보관본이다.

- 최초 셸 래퍼는 CRLF 때문에 검사 전에 실패했다. 원래 파일과 `wrapper_initial_failure.txt`를 보존했고 LF 래퍼를 별도로 작성했다.
- 첫 실제 검사 `tests.log`는523통과·4실패였다. 세 검사는 이전 해시와 현재 파일을 직접 비교하던 사슬의 후속 계약 누락, 한 검사는 재생 묶음의 `drive_feedback` 전이 의존성 누락이었다. 해당 검사 사슬과 의존성 목록을 보완했으며 `before_manifest_repair.zip`에 수정 전 파일을 남겼다.
- 후속 검사는 `check_connection2.sh`로 수행한다. 관련527검사 → 기존 Gazebo OFF/2초 ON 기록 재생 → 빌드 → 설치 파일 대조의 순서이며 어느 단계든 실패하면 다음 단계로 넘어가지 않는다. 실제 완료 결과는 아래 또는 종료 코드 파일을 따른다.

다음은 [사전 등록한 Gazebo1회](../rear_lane_preference_gazebo_v1/DRIVE_PLAN.md)다. 함수 검사나 빌드를 주행 성공으로 해석하지 않는다.

## 연결 확인 결과

관련 527개 검사가 통과했고, 기존 OFF 649/649·2초 ON 653/653판단이 동일하게 재생됐다. `arena_autonomy`빌드와설치5모듈의실제경로/바이트일치,기본OFF·timed정책을확인했다. [재생요약](replay_summary.json).

이후Git정규화로출처해시가달라지지않도록8개파일의CRLF를LF로변경했다. 모든Python AST가같으며[변경목록](eol_normalization.json)과수정전ZIP을보존했다. 새계약의after해시·의존성해시를현재LF바이트로맞췄고,과거실행/실패ZIP은바꾸지않았다. `check_final.sh`는관련검사·재빌드/설치·Gazebo사전배치검사만수행한다.

최종 확인: LF 정리 뒤 관련 73개 검사·재빌드·설치 5모듈 대조·어댑터 12개 사전 검사를 통과했다. Git text=LF 대상 파일에 CRLF가 남지 않은 것도 확인했다. 실제 정지·주행 성능은 별도 Gazebo 실행 결과를 따른다.
