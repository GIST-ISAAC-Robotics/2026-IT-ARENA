# 재사용 허용 뒤 단1회 통합 계측 레시피

**현재 실행 승인 또는 실행 결과가 아니다. Jetson 재사용 전달 전에는 접속·조회·설치·실행하지 않는다.** 아래는 재개 시 root가 배정할 독점 실행 1회이며 Windows 분석으로 Jetson 처리율을 예측하지 않는다.

## 고정 조건

1. tooling 담당이 만든 **최신 공통 설치 묶음 하나**를 새 경로에 설치하고 source/install manifest SHA를 고정한다. r500 설치는 보존한다. r523 controller 6 ms, guard 기본 2 ms/64개, r526 fresh current/실제 dt, r524 `g1_trace_detail=compact_v1`가 로드된 모듈에서 확인돼야 한다. 최신 묶음에 들어간 다른 계획·벽·guard 수정도 manifest에 명시한다. r523과의 차이를 이 세 수정의 순수 인과 효과로 해석하지 않는다.
2. r523의 `reconstructed_inputs.json.gz` SHA `c2df9fe72436204d933eb0c078fde972053f8086521ce0701cfce93f6e332ad1` 하나를 그대로 사용한다. 20초 선택 `[7.62,27.62)`, 앞부분 운동 표본·신호 준비 포함, active 집계 `[7.62,27.52)`. 원 `simulation_acquisition`·epoch·stamp를 유지한다. gyro 200 Hz, wheel 100 Hz, 저장 scan 약10 Hz, 합성 RGB 848×480·20 Hz→실제 detector를 유지한다.
3. 기존 simulation profile을 사용하되 최신 설치 경로/trace 경로와 명시 compact 선택만 필요한 차이로 기록한다. `g1_complete_plan_trace=false`; trace 경로는 비우지 않는다. TTL·신호·현재 점유·기하 검사를 바꾸지 않는다. ROS domain 158, `/replay_r508`, `r506_reconstructed_sim_imu`, `laser_frame`을 그대로 유지해 의미 없는 이름 치환을 피한다.
4. controller·독립 guard·read-only RaceOps·vision/plan/current와 입력/관찰자만 실행한다. SHM 단독16 MiB는 이 실행의 환경에서만 선택하고 4 participant의 실제 적용을 확인한다. 물리 host/센서/포트/MCU/출력 노드는 없다. `/drive/safe` 발행자는 guard 하나여야 한다. 같은 장치 경쟁 작업을 두지 않는다.

## 기존 r523 실행기에서 반드시 제거할 시험용 개입

| r523 부분 | 최신 설치를 측정할 때 |
|---|---|
| `ControlReceipt.control`의 `drain_ready(...)` 직접 호출 | 제거. 실제 `MotionInput.drain_pending`을 관찰만 하여 controller6/guard2 ms가 각각 **한 번** 실행되는지 기록한다. wrapper가 다시 소비하지 않는다. |
| `proof_candidate_bootstrap` import/모듈 재실행 | 제거. 최신 설치의 proof/current 구현을 사용한다. |
| `ObservedG1.create_follow_trace`의 close→`TraceOff` 교체 | 제거. 실제 `FollowTrace(detail='compact_v1')`를 사용하고 header/footer로 확인한다. |
| r500 절대 설치 경로 assert | 새 동결 bundle/install SHA·모듈 경로 assert로 교체한다. |

모든 producer topic/frame/time_basis와 두 consumer가 실제 읽은 profile을 한 표로 대조한다. 첫 scan/명령 계산 전에 controller·guard 각각 gyro/drive_motor accepted≥1을 확인한다. 실패하면 성능 실행으로 세지 않고 준비 오류로 종료한다. `SingleThreadedExecutor`와 같은 callbackgroup 전제도 고정한다.

## 경량 scalar 계측: 한 실행에서 필요한 종단 시각

각 노드에서 **실제 구현을 한 번 호출하는 관찰 wrapper**만 사용한다. 원문 scan/history/path나 전체 출력 dict를 deepcopy/freeze/직렬화하지 않는다. 수집 중에는 메모리에 작은 고정 필드만 넣고 종료 후 저장한다. 예를 들어 노드별 30,000행 상한과 dropped 카운터를 두며 초과 시 판정 불능으로 표시한다. 로그 누락을 숨기지 않고 TTL/출력은 변경하지 않는다.

| 관측 경계 | 보존할 scalar |
|---|---|
| 모든 control 진입/종료 및 실제 drain | monotonic_ns, ROS now, timer/direct 구분, 호출 ID, drain 전후 최신·현재사용 가능 gyro/wheel 원 stamp, 소비 개수·시간·상한 도달, drain 뒤 제어 reference, 상태 |
| plan/current offer·실제 worker collect | stage/generation/sequence/revision, retained, 원 current reference, source epoch/first/last, original path source, previous angle, checked speed/angle, submitted/began/completed/received monotonic, discard reason |
| `prepared`/`application_reason` 실제 호출 | 동일 token, poll ROS/monotonic, last_application, 실제 output_dt, 기존 함수가 반환한 이유, ready 존재. 검사를 계측 때문에 다시 호출하지 않는다. |
| 실제 `/drive` publish 진입/복귀 | token과 원 command header stamp, speed/angle, ROS/monotonic. 메시지 발행은 한 번만 한다. |
| publication ack·실제 ready 취소/교체 | token, ack ROS/monotonic, retained/reason, 이전 ready token→새 token/없음, 원 revision/generation. 0/STOP·실제 정책 전이와 분리한다. |
| guard `on_command` 진입/반영 | 원 `/drive` stamp, 수신 ROS/monotonic, callback 뒤 실제 보관 stamp(역순 미채택 식별). |
| guard 평가와 `/drive/safe` 발행 | 원 command stamp·원 나이, guard reference/새 header, command_stale/reason, safe speed, ROS/monotonic. |

compact `call_summary`는 현재 계산의 모든 완료·ready 대기를 기록하지 않는다. 따라서 위 작은 sidecar를 사용한다. poll의 호출 시각으로 새 source stamp를 만들지 않고 기존 scalar를 읽는다. 내부 `application_reason`이 commit에서 여러 번 호출되는 것은 기존 검증이다. 관찰 wrapper가 호출 수를 추가하지 않아야 한다.

## 단일 실행·정지·회수

입력/그래프/경로 SHA 사전 검사가 통과한 뒤 **20초 재생 한 번**만 시작한다. 실제 소유 PID 영수증은 시작 시 한 번 남긴다. 기존 실행기와 같이 노드 최대58초 wall cap, 종료 service 제한 및 소유 자식 회수 규칙을 둔다. source/profile/hash 불일치, worker fault, 예상 외 물리 host/포트/process, 계약 위반 또는 사용자의 STOP이면 즉시 재생을 중단하고 STOP·회수한다. 다른 예산값으로 자동 재실행하지 않는다.

정상 종료 시 원 입력 끝/EOF와 active를 분리하고 STOP service 성공, 자식 exit, 강제 종료 여부, 소유 PID0, source/install 입력 SHA 불변을 기록한다. 실제 모터 정지 증명으로 확대하지 않는다.

## 한 번으로 답할 질문과 판정

- **회수→허가:** 새 current의 ready 이후 첫 eligible poll까지 지연과 거절 이유. 20 ms 미만/실제 조향 예산/정상 epoch·위협 전이를 분리한다. 조건이 충족됐는데 100 ms까지 반복 기다리는 사례가 있으면 실제 token·dt로 재현한다.
- **허가→발행→guard:** 각각 별도 지연을 계산한다. source age는 guard가 보관한 원 `/drive` header로 계산하고 `/drive/safe` 새 header와 섞지 않는다. last_application은 실제 publish 시각보다 뒤일 수 있으므로 publish→ack/상태 처리 시간도 분리한다.
- **발행 누락:** 모든 current token에 published/명시 폐기/정상 종료/미분류 중 하나를 연결한다. 이미 준비된 양수 결과의 중복 발행·옛 stamp 복제·이유 없는 ready 소실을 찾는다. 새 계획에 따른 정상 교체를 모두 버그로 간주하지 않는다.
- **주행 손실 지표:** active motion_stale/command_stale, `/drive`·`/safe` 0/양수, 양수 최대 공백, 0 구간 수·중앙/p95/최대·총 시간·우측절단을 함께 낸다. r523처럼 최대 공백만 줄고 총 0 시간이 늘어나는 경우도 표시한다.
- **비용/무결성:** 원 입력 accepted/rejected, timing drop, plan/current 완료 비용, 직접 control 포함 p95/max, 실제 drain 사용량, compact bytes/drop을 낸다. r523·r526 저장값은 비교 기준일 뿐 최신 조합의 측정값을 대신하지 않는다.

예상은 100 ms gate 때문에 대기하던 fresh current가 조향 예산 안에서 더 빨리 전달되는 것이다. **0 command stale, 일정 50 Hz/10 Hz, 최대 지연·완주·실차 안전은 미검증**이다. 남는 실패는 이 1회의 token timeline으로 구분하고, 새로운 원인 없이 반복 시험하지 않는다.
