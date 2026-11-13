# r365 실제 evaluator 연결·제한 동등성·ROS 종료 확인

후속 실행(2026-11-13): 원 r353와 새 factory의 같은 저장 입력에서 계획12개·현재 검사10개 출력이 모두 일치했고 네 자식을 정상 회수했다. [비교 결과](comparison.json). 실제 ROS `G1G2Node`에 새 backend를 연결하여 두 spawn 자식 생성, STOP0출력·세대 무효화·RESET·늦은 결과 보관 해제·정상 종료1검사를 통과했다. [노드 검사](node_tests.log). 아래 ‘미실행’은 초기 설계 시점 기록이며, 여전히 Gazebo 주행·10Hz·공통 기본 노드 연결·실물 양수 실행 증명은 아니다.

2026-11-13. GPT가 r362를 기존 계산 정책과 연결할 최소 경계를 조사했다. **공통 src/기존 테스트/기존 시험 원문을 수정하지 않았고 WSL·Jetson·ROS·Gazebo 실행도 하지 않았다.** r364의 소스 동결/주행과 독립적인 준비물이다. 아래 초안은 AST만 확인했으며 동등성·ROS 연결 통과를 주장하지 않는다.

## 결론: 먼저 시험 entry의 backend만 교체한다

가장 작은 다음 실행은 r353의 정책과 입력 구성을 그대로 두고 `node_entry_draft.py`를 명시 prefix로 선택하는 것이다. factory가 기존 r278 계획·r352 현재 검사를 자식별로 설치하고, 슬롯·기한·발행 확인은 공통 r362로 바꾼다. **공통 제품 모듈의 artifacts import는0으로 유지**된다. artifact entry/factory가 원 시험 정책을 참조하는 것은 이 한정 대조의 명시 의존이며, 아직 운영 패키징이 끝났다는 뜻은 아니다.

- `factory_reference.py`: spawn 가능한 최상위 factory. plan은 원 r278 설치 뒤 `replay_call`, current는 원 r352를 직접 호출한다. 원 `overlap.evaluate_current`의 scene 복원·예외0출력·`require_current_region=True`를 보존한다. 숫자 커널과 reset도 원 자식과 같다.
- `node_entry_draft.py`: r336의 센서/신호→후보→실제 발행 확인 흐름을 유지하고 새 Worker/Delivery만 선택한다. `use_sim_time=true`, native 수치 실행, 기존 선택형 현재 검사/경로 보존 구성만 허용하며 hardware shadow는 거절한다. 지연 주입은 없다.
- `profile.draft.json`: 파일과 prefix를 직접 지정할 때만 쓰는 초안 선택이다. 기존 launch나 기본값에 등록하지 않았다.
- `compare_evaluators.py`: 원 r353 자식과 새 factory 자식에 같은 저장 입력을 넣는 준비 스크립트. 최대12call 기본/상한32call, 단계별 최대2자식으로 제한한다. plan/current 전체 출력 일치와 정상 종료를 비교한다. 실제 원 주행 재현·주기 성능·ROS 증명은 아니다.

초안 entry는 r336처럼 최초 공통 직렬 자식을 생성·정상 종료한 뒤 두 슬롯으로 바꾼다. 시작 시 한 번의 불필요한 생성이 남지만, 원 경로를 바꾸지 않고 factory/발행 인계를 비교할 수 있다. 실제 제품 연결 때 아래 생성 hook으로 제거한다. 이 초기화 교체 경로의 실패·완료 알림 정리는 ROS 연결 시험에서 확인해야 한다.

## 얇은 factory만으로 공통 정책을 바꿀 수 없는 이유

원 `candidate_recheck`와 공통 `current_path_recheck`의8개 Python 파일을 AST로 대조했다. **7개는 동일하며 `_geometry.py`만 다르다.** 원 `candidate_policy.py`와 공통 `guard_observation_policy.py`는 `common` import 대상만 다르다. 따라서 전 패키지를 복사할 필요는 없지만 공통 `rebase`로 바로 대체해서도 안 된다.

| 계약 | r353가 사용하는 r266 geometry | 공통 geometry |
|---|---|---|
| 원 경로 나이 | 고정500ms | `cfg.memory_ttl_s` |
| 직전 요청각 | 바뀐 값으로 현재 조향/영역 재검사 | 제출 당시 요청각과 다르면 거절 |
| 입력 시각 | `timing_verified` 필수 | 명시 `host_estimated_v1` 경계도 수용 |
| 조향 한계 | 현재 previous 각이 `cfg.max_steering_rad` 이내인지 검사 | 이 위치에서 같은 검사를 하지 않음 |

이 차이를 없애는 것은 정책 변경이며 이번 목표인 r353 동등성과 다르다. factory를 연결한다는 이유로 추정 시각을 검증된 시각으로 바꾸거나 실제 엔코더를0으로 보충하지 않는다.

실제 current 의존은 `r352 → r346 → r343 → r281 → candidate_policy → candidate_recheck`다. plan은 별도로 r278의 약10줄 조건을 `_baseline_command`의 지정 anchor 앞에 삽입한다. r361/r364의 새 후보를 이 r353 비교에 자동으로 섞지 않는다.

## 공통 코드로 옮길 때의 최소 변경 제안

1. **정책 준비물:** r353용 geometry 함수만 별도 모듈로 보존하고, 동일한7개 공통 재검사 모듈을 재사용한다. current 전용 자식에서 그 geometry를 명시 선택하는 방법이 가장 작다. 기존 r278처럼 선택 자식 내부의 정책 설치로 시작할 수 있으며 부모/다른 backend의 기본 함수를 바꾸면 안 된다. r281/r343/r346/r352의 얇은 재시도 함수를 명시 이름으로 옮기고 공통 `guard_observation_policy`를 재사용한다. 이를 별도 정책 선택으로 등록한 뒤 실제 출력 대조를 해야 한다.
2. **노드 생성 한 지점:** `G1G2Node.__init__`의 worker/delivery 생성(현재 대략92–110행)을 선택형 생성 함수로 분리한다. 기존 선택은 그대로 두고, 새 명시 선택만 r362+등록 factory를 반환한다. 이때 최초 직렬 자식 생성/교체가 사라진다. 명령·현재 검사·generation·STOP 로직을 새 노드에 복제할 필요는 없다.
3. **control 두 지점:** 실제 `LocalPursuit.control` 반환 직후 r336의 pending publication 확인을 호출하고, 입력 제출의 idle 조건만 두 슬롯에 맞춘다. 현재 `command()`의 `poll`/`offer_recheck` 반환 형식, 완료 알림, STOP/RESET, `close()` 보고는 r362와 이미 맞는다. 기본 직렬 경로의 idle 조건은 그대로 둔다.
4. **명시 경계:** 최초 등록은 시뮬 전용으로 유지한다. 실제 운영 clock/encoder/독립 보호 승인과 hardware shadow의 추정시각은 이 정책 이관과 별개다. r362/r365의 연결 자체는 r358 계획약132ms·10Hz미달 해결이 아니다.

전 정책 복사나 제어기 재작성 대신 이4개 접점으로 제한할 수 있다. 공통 geometry의 옵션화를 택하면 여러 하위 재검사 함수에 전달해야 하므로, 현 시점에는 기존 시험의 선택 자식 범위와 동일한 정책 설치 방식이 더 작은 변경이다. 어떤 방식을 쓰더라도 지원 범위/원 시각/수치 결과 차등검증을 통과하기 전 기본 채택하지 않는다.

## 다음 실행 순서와 수용 조건

1. r364 실행·정상 회수가 끝나 WSL 소유권이 반환된 뒤 `compare_evaluators.py --limit 12 --output <새 결과 JSON>`을 실행한다. plan/current 결과를 원 자식과 전부 비교하고 두 단계 자식을 정상 회수한다. 불일치가 있으면 실제 노드 연결을 진행하지 않는다.
2. 동일 r353 선택의 짧은 ROS 계약 실행에서 r336 대비 source/command reference, 실제 발행 ack, STOP 이후 늦은 결과, 두 자식·알림 정리를 확인한다. 이미 검증한 개별 센서를 반복하지 않는다.
3. 이 두 단계가 성립하면 공통 생성/발행/idle 접점과 등록 정책을 한 묶음으로 옮긴다. 한정 대표 주행의 목적은 backend 이관 동등성이지 r353 정체나10Hz문제의 해결 선언이 아니다.

현재 근거는 [정적 감사](static_audit.json)뿐이다. Python 초안4개를 AST로 확인했고 관련 원문/공통 파일/초안21개 해시를 기록했다. 실행 결과 파일은 아직 만들지 않았다.
