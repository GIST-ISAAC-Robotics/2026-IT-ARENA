# r555 희소 상부 상대 — r553 소스 재사용 예정 레시피

**Windows 파일 준비 완료, 동결·실행0.** 원 r546 파일50개는 그대로 보존했다. 장면/actor/원 평가는 r546를 재사용하며 **제어 소스는 r553의 최종 동결본을 root가 확인·선택한 뒤 재사용할 예정**이다. r553은 별도 단독 주행이며 이 레시피에 결과를 복사하거나 성공으로 간주하지 않는다. `source_selection.json`의 freeze/drive/WSL 권한은 모두 false, 최종 선택 pending이다.

## 같은 장면, OFF 한 번만 준비

- `right_side_start`, seed0,12 sim초, actor1대. 시작 중심 전방0.50m·우측0.23m, 일정0.5m/s. 원 굴곡벽/시설3개, grid3, RTF 설정0.25, `g1_open_1p5`·`medium_v1`을 유지한다.
- 실측 count 파생 차량 YAML과 중앙 명령 상한0.28475436568260193rad를 유지한다. 바퀴각 자체는 미측정이며 CAD 환산값이다.
- `sparse_upper_posts`: 기존8mm 지주4개·얇은 상판, actor 하단20mm/상단240mm. collision은200×150×220mm의 보수 외접 상자이며 지주별 정밀 충돌기하가 아니다. 실제 상대 형상 측정값도 아니다.
- 후면 ID1·DICT_4X4_50 마커는 계속 붙어 있다. **50mm는 흰 여백 포함 전체 판**, 검정 코드34.8mm, 흰 여백은 한쪽7.6mm다. 마커 중심 높이200mm와 자차49/코스 ID 구분을 유지한다.
- 이번에는 `r537_marker_profile=''`, observer false의 **보조 OFF 1회만** 준비한다. ON은 동일 actor/월드로 별도 한 번을 root/runner가 선택·준비하며 이번 레시피에 포함하지 않는다. OFF에서 마커 검출 성공을 요구하지 않는다.

## 소스 선택과 실행 인계

옛 `r540 미채택`·`r544 대기` 표기는 새 선택 파일에서 제거했다. 아직 채택 목록을 추정해서 채우지는 않았다. `source_baseline_observation.json`은 현재 r553 준비 영수증을 읽었다는 참고 기록이며 실제 주행 결과·최종 선택·r555 동결의 증명이 아니다.

`prepare.py`는 **장면 기준 r539와 소스 기준 r553을 분리**했다. 장면 인수/판정은 기존 기준으로 대조하고, 실제 공통 파일은 root가 지정한 `source_baseline/prepared.json`의 필수 `frozen_inputs` 및 archive SHA와 대조한다. 소스 변경이 있으면 root가 기존 변경 목록/해시로 명시한다. 참고 inventory를 실제 실행 입력과 혼동하지 않는다. 공통 source 수정·빌드·설치0이다.

희소 actor 어댑터가 직접 import하는 기존 `scripts/experimental_facilities.py`는 r553 단독 동결에 없으므로 추가 실행 입력 후보와 현재 SHA를 명시했다. 이는 기존 평가용 부품/마커 생성기 재사용이며 파일 변경은 없다. 로컬 마커 PNG도 향후 동결 목록에 포함한다. 최종 채택과 동결은 계속 root 소유다.

root/runner가 r553 회수 결과와 최종 소스를 확인하고 WSL 소유권을 넘겨받은 뒤 수행할 순서는 다음과 같다. 아래 파일은 **명령 준비**이며 이번에는 실행하지 않았다.

1. `source_selection.json`의 최종 소스·변경 목록·권한과 빌드 재사용 여부를 확정한다. 측정 vehicle YAML·기본 상태 기록 `full`을 유지한다.
2. 기존 `prepare_once.sh`로 preflight/동결을 수행하고 새 `prepared.json`·`sources.zip`을 생성한다. 실행 전 root가 이 둘의 SHA를 `root_execution_review.json`에 연결한다.
3. `execution_v1/execute_once.sh`로 OFF 1회만 실행한다. 소유 PID/시작/종료 로그를 남기고8 sim초 무진행·기술 오류·기존 STOP 규칙을 유지한다. 실패 자동 반복은 없다.
4. `execution_v1/finish_collection.bash`로 기존 원 보고서·정지·간격·소유 PID0·입력 무결성을 회수한다. 저장 자료의 정상 FOLLOW 복귀는 `analyze_controller_guard.py`로 보조 설명하며 원 `passed`/gate를 고치지 않는다.

## 원 평가와 회수 항목

`config.json`의 `acceptance`·case·STOP 조건은 r546와 정확히 같다. 기존15cm 판정과80mm/바퀴 중심 등의 별도 보조 판정을 합치거나 새 성공 기준으로 대체하지 않는다. 제어 모드 하나를 새 필수 통과 조건으로 만들지 않는다.

- 원 `run1/report.json`의 `evaluation.case_metrics`·`progress`·`gap`, 최장정지·최종/최소 간격·접촉·노면·전체 원 gate를 회수한다. 제어기 `/drive`0과 guard `/drive/safe`0을 구분한다.
- `scans_received.jsonl`, `acquisition.json`, `ego_poses.jsonl`, `actor_poses.jsonl`, `actor_commands.json`, `actor_contacts.jsonl`, 실제 `runtime_vehicle.sdf`·`runtime_local_sensors.yaml`·actor 기하를 함께 보존한다. **전체 scan 유효반환수는 actor 반환수가 아니다.** 기존 수집기는 raw를 저장하지만 actor 귀속수를 자동 산출하지 않는다. 회수 후 평가 전용 분석에서 각 ray의 취득시각·실제 ego/actor 자세·sensor frame을 맞춰 actor 반환수를 집계하고, 표본별 분포/최솟값/무반환 연속구간과 귀속 불명확 수를 분리한다. 미측정·판정 불가를0으로 바꾸지 않는다. 이 항목은 회수 계획이며 새 raw 분석 실행은0이다.
- `follow_decisions.jsonl`과 실제 발행을 연결해 저속 재획득의 적용·일반 양수 FOLLOW 복귀 시각·복귀 뒤 속도와 최종 상태를 보고한다. 단순 plan 수만으로 실제 출력/복귀를 주장하지 않는다. 기존 `analyze_controller_guard.py`의 정상 정의는 `handoff_state=FOLLOW`, `normal.reason=local_follow`, 양수 계획, helper 없음이다.
- 위 actor/ego 정답 자세·접촉·일정은 **평가/회수 전용**이다. 제어기 입력은 센서/기존 피드백이며 `/drive`→독립 보호층→`/drive/safe` 경계를 유지한다.

정적 재확인은 `python artifacts/validation/2026-11-14/r555_sparse_actor_ready_v1/check_recipe.py`다. JSON·해시·AST만 검사하며 geometry 생성·ROS import·동결·WSL/빌드/Gazebo/Jetson 접근은 하지 않는다. `preparation_receipt.json`과 `recipe_provenance.json`이 준비 근거다. 실제 동적 희소 감지·회피·마커 검출·실차 성능은 미검증이다.
