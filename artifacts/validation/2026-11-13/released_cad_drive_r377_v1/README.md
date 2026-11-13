# r375 현재 RELEASED parallel 후보 주행: SOLO 정체·적용 0

**10.01m 진행 뒤 정체로 실패했다. r375 적용은 0회다.** [r375 저장 비교](../released_current_r375_v1/README.md)를 바탕으로 [r371 원 실패](../relative_body_cad_drive_r371_v1/README.md)와 같은 공식 코스 조건에서 1회 실행했으나, 이번에는 RELEASED 상태가 나타나지 않았다. 원 [보고서](run1/report.json)를 보존하고 [별도 감사](audit.json)에 원인·종료·해시를 기록했다. 저장 비교에서 확인한 r375 효과를 이번 주행 성공으로 확대하지 않는다.

## 실행 결과와 원 실패 구분

- 현재 검사 **223개=SOLO 208·GUARD 15·RELEASED 0**이다. r375 current 자식 설치와 정상 반환은 [영수증](worker_policy_receipts/)에서 확인했지만 실제 후보 적용은 0회다.
- 마지막은 SOLO baseline의 원 계획 0.45m/s를 현재 검사에서 0으로 거절했다. 원 계획 조향 0.394931rad, 재검사 조향 0.394927rad, 직전 조향 0.406621rad, 실제 측정속도 0m/s·`dt=0.1s`·스캔 나이 약 0.12s다. 현재 점유 clearance 0.52m만으로 전체 이동 영역이 관측됐다고 보지 않는다.
- 기존 0.15m/s 재시도는 미관측 차체 옆 초과 **15.209mm**, 마스크 꼬리 13.578mm였다. r370 0.10m/s 재시도도 미관측 차체 옆 초과 **10.766mm>기존 10mm**로 `outside_small_body_side`에서 거절됐다. 이때 마스크 꼬리 9.744mm 자체는 10mm 안이므로 두 경계를 혼동하지 않는다.
- 전체 상대 거리 재시도 100개 중 94개는 `outside_small_body_side`, 6개는 `unbounded_nonreturn`이며 적용 0이다. 원 0.15m/s SOLO 저속 재시도 5개와 이전 0.10m/s 자체 마스크 재시도 1개는 적용됐으나 끝 정체를 해소하지 못했다.
- r371의 RELEASED parallel·고정 이웃 0.6m 실패와 **다른 상태/실패**다. 이번 결과는 r375가 실행되지 않은 연결 오류도, RELEASED 후보의 주행 효과 확인도 아니다.

원 보고서는 최대 정지 구간 9.702sim초, green 뒤 기록 구간 약 22.716sim초/요구 80sim초, 접촉 이벤트·겹침 표본·노면 이탈 0을 기록한다. 명시 STOP·촘촘한 실제 정지 표본·차체 안정은 통과했다. 외부 소유 STOP이 평가기 STOP보다 먼저 들어가 사전 MCU latch 조건은 실패했고, 전체 기간·상대 추적/명령 조건도 실패했다. 따라서 `passed=false`, `harness_valid=false`, `run_valid=false`를 그대로 유지한다.

14자식 모두 정상 종료·잔여 PID 0·강제 회수 0, 입력 1,350개와 source archive 불변을 확인했다. 원 CAD·센서·`blank_20_30`·`road_level`·RTF 0.25와 독립 보호 조건을 유지했다. 정답 자세는 평가에만 사용했다. 이 문서의 후속 분석에서 새 rebase·시뮬레이션·소스 변경은 하지 않았다.

## 유지 조건과 변경

원 r371 `config.json`, `actors.json`, `registered.json`의 명령을 직접 읽는다. 설정 차이는 시험 설명 `scope`와 `launch.entry_g1`뿐이다. 평가 명령의 실질 차이는 `--config`, `--actor-overlay`, `--output`의 새 시험 폴더 경로 3개이며, CAD/센서 파일 경로는 같은 파일을 가리킨다. [`prepared.json`](prepared.json)에 원/새 인자·원 timeout·소스 해시를 기록했다.

- 같은 공식 `official_v2026_11_06`, 구형 610mm 시설 파생본, ID20/30 blank, 두 220mm 상대, `road_level`.
- 같은 축거 128mm/윤거 123.9mm/계획 외곽 207×150mm, 최종 CAD 조향·센서/지주/6mm 배선, 81T 구동계·물성.
- 같은 `g1_open_1p5` 프로필, 계획 r361, r336 ROS 인계, native_v1, 무주입 계산 지연, RTF 0.25. 최고속도/가속/정지선·TTL을 변경하지 않는다.
- 같은 80sim초 1회. 시작 wall timeout 150초, 전체 wall timeout 1,500초, STOP wall timeout 120초, green 이후 초과 여유 20sim초.
- 같은 `/drive → 독립 보호층 → /drive/safe`, 가상 MCU, STOP/접촉/노면/전체 기간/상대 추적 판정. 지도·정답 위치는 평가에만 사용한다.

주행 판단 변경은 current 자식에 연결하는 파일을 r370에서 `released_current_r375_v1/candidate.py`로 바꾼 것 하나다. r375가 내부에서 r370을 먼저 실행하므로 기존 SOLO 후보도 보존한다. `low_solo_worker.py`의 이름은 기존 연결을 유지하기 위해 그대로 두었으며, 실제 설치 영수증은 `policy=r375`와 후보 SHA256을 기록한다. 원 계획 r361 설치/복구 영수증도 유지한다.

원 r371 이후 저장소 변경은 `steering_zero_profile.py`, `steering_calibration.py`, 해당 테스트 3개가 별도로 있다. 이는 조립 후 수동 영점 프로필 직렬화 준비이며 이번 launch에 새 구동 연결을 하지 않았다. 원 소스가 전부 바이트 동일하다고 주장하지 않으며 [`prepared.json`](prepared.json)에 세 파일의 이전/현재 해시를 남겼다. 나머지 원 실행 소스/정책 계보는 준비 당시 원본과 같고, `prepare.py --check`는 동결 뒤 변경 시 거절한다.

## 당시 실행 담당에게 넘긴 고정 명령

주 에이전트가 지정한 한 WSL 담당자만 실행한다. 기존 ROS/native 환경과 저장소 루트 cwd를 사용한다. 명령이 완료될 때까지 동일 장치의 새 Gazebo/빌드/처리시간 측정을 겹치지 않는다.

먼저 새 경로의 실제 plan/current 자식 설치·STOP·RESET·정상 종료 연결을 검사한다. 이 검사는 Gazebo를 시작하지 않는다.

```bash
python3 artifacts/validation/2026-11-13/released_cad_drive_r377_v1/prepare.py --check
python3 -m pytest -q artifacts/validation/2026-11-13/released_cad_drive_r377_v1/test_node.py
```

연결 검사가 통과하고 주 에이전트가 1회 실행을 배정한 뒤 다음 **상대 경로 명령 그대로** 실행한다. `stop_owned.py`는 이 argv와 cwd가 일치하는 소유 프로세스 하나에만 연결한다.

```bash
python3 artifacts/validation/2026-11-13/released_cad_drive_r377_v1/run.py
```

원 r371과 같이 8sim초 무진행 또는 접촉/노면 이탈 등 위험이 관측되면 소유 시험에 STOP을 요청하고 원 평가기의 회수를 기다린다. 정체 관측은 평가용 기록에서 판단하고 제어기에 넣지 않는다. 정지 자체를 완주로 처리하지 않는다.

```bash
python3 artifacts/validation/2026-11-13/released_cad_drive_r377_v1/stop_owned.py
```

`registered.json`/`run1`이 있으면 다시 실행하지 않는다. 새 출력은 `run1/`, 자식 설치/종료는 `worker_policy_receipts/`, 실제 argv는 `launch_command.json`, 소스 보관은 `sources.zip`/`registered.json`, 종료 불변성은 `integrity.json`이다. 실패가 발생하면 원 로그·판정을 그대로 보존한다.

## 판정과 종료 인계

원 10.13m 정체를 지났는지, `follow_decisions.jsonl`의 `current_rebase.output.current_released_relative_body.applied`가 실제 true인지 함께 확인한다. 부모가 옵션 이름을 기록하거나 자식이 설치된 것만으로 후보 주행 적용을 선언하지 않는다. 적용 시 원 목표 이하·현재/직전 조향·원 경로 시각·미관측 수 보존도 확인한다.

진행 거리/랩·긴 저속/8초 정체·접촉 이벤트/겹침 표본·노면 이탈·명령 만료와 MCU latch·명시 STOP 후 실제 정지·정상 자식 종료/잔여 PID·입력/소스 archive 불변을 분리해 남긴다. 원 전체 기간·상대 움직임·사전 MCU latch 조건을 실패하면 일부 진행 개선과 별개로 원 시험 통과로 표시하지 않는다. 원 시험은 RTF 0.25의 제한 시뮬레이션이며 실시간성·실차 안전·전체 조건 검증이 아니다.

## 고정 속도 후보 누적을 대체할 설계 의견

**0.10 다음에 0.09m/s를 추가하는 수정은 이번에 구현·시험하지 않았다.** 속도가 아니라 기존 반응·제동 이동거리에서 허용 목표를 역산하는 구조가 가능하다. 다만 이 구조가 현재 공백과 점유 검사를 대신해서는 안 된다.

현재 [`envelope_distance`](../../../../src/arena_autonomy/arena_autonomy/m3_execution_trial.py)는 다음 식이다.

\[
v=\max(|v_\mathrm{measured}|,|v_\mathrm{target}|),\quad
\tau=dt+\max(0,t_\mathrm{ref}-t_\mathrm{first})+0.10,\quad
D(v)=v\tau+\frac{v^2}{2a},\qquad a=2.0\;\mathrm{m/s^2}.
\]

마지막 입력의 \(\tau\simeq0.32s\)에서 목표 0.45/0.15/0.10m/s의 \(D\)는 각각 194.625/53.625/34.500mm다. 0.10m/s라도 현재/직전 조향 원호의 회전은 약 6.44°/6.65°다. [`motion_queries`](../../../../src/arena_autonomy/arena_autonomy/pass_release_trial.py)는 두 조향의 원호·차체 극값·바퀴 지름 외곽·얇은 unknown 셀 교차를 사용하고, 기하 계산 거리를 최소 10mm로 유지한다. 단순 목표 속도 크기만 보고 이 회전 외곽을 생략할 수 없다.

같은 검사를 만족하는 거리 \(D_\mathrm{allowed}\)를 얻을 수 있다면

\[
v_\mathrm{budget}=\sqrt{(a\tau)^2+2aD_\mathrm{allowed}}-a\tau
\]

로 후보를 구하되 원 목표와 기존 상대 공백 목표 상한 0.10m/s를 넘지 않게 한다. 실제 측정속도 0–0.15m/s 범위도 그대로 유지한다. 실제속도 자체의 제동거리 \(D(|v_\mathrm{measured}|)\)가 허용거리보다 크거나, 최소 10mm 기하부터 통과하지 못하면 목표만 낮춰서는 허가할 수 없다. 예를 들어 측정속도 0.15m/s이면 목표를 0.10 아래로 낮춰도 마지막 입력의 식에서 53.625mm가 그대로 남는다. 감속상수 2m/s²는 기존 모델값이지 실차 측정 보증값이 아니다.

1. **현재 관측에 근거한 허용 거리 접두 구간 — 구조 정리의 우선안.** 한 현재 장면/원 경로의 재검사를 공유하고, 두 조향의 거리별 외곽·바퀴·unknown 교차에 거리 정보를 붙여 허용되는 연속 앞부분을 구한다. 그 거리에서 속도 하나를 역산하고 기존 전체 검사를 최종 한 번 수행한다. 기존 10mm·6°/5cm 공백·전방 제외·점유 padding·원시 시각·현재 측방·TTL은 그대로다. 현재 `command_arc`는 목표 거리마다 `linspace`를 다시 만들어 짧은 거리의 질의가 긴 거리의 부분집합이라고 보장되지 않는다. 따라서 지금의 통과/실패 함수에 단순 이분 탐색을 얹으면 안 된다. 중첩되는 거리 표본과 새 끝점/극점 검사를 명시하거나 기하 첫 교차를 보수적으로 처리해야 한다. 반복 wrapper의 경로/광선 계산을 공유할 여지는 있지만, 거리 메타데이터·끝 경계 검증 비용과 동등성 검증이 추가된다.
2. **보수적 기하 예산 제안 후 한 번의 전체 검사 — 작은 구현 대안.** 두 조향의 차체/바퀴 외곽 극값으로 옆방향·마스크 꼬리의 허용거리를 보수적으로 제안하고, 속도를 역산한 뒤 기존 현재 검사를 한 번만 수행한다. 기하 계산은 상대적으로 작지만 공백의 양끝 반환·전방/점유·시간 허가는 최종 검사에서만 얻는다. 네 차체 모서리만으로 바퀴나 얇은 unknown 셀을 대체할 수 없다. 이미 관측된 바깥 공간까지 모두 10mm로 제한하면 필요 이상 감속하므로 기존 정지 후보에만 적용하고, 정상 양수 경로에는 새 제한을 넣지 않는다. 허용속도를 과소평가하거나 유효 후보를 놓칠 수 있어 최속 해법으로 보장하지 않는다.

어느 안도 무경계 공백 6개가 속도 하향만으로 통과할 것이라고 가정하지 않는다. 반복 계산비는 [r380 준비](../current_cost_r380_v1/README.md)의 실제 계측과 함께 판단한다. 이번에는 원 실패 기록과 설계 경계까지만 남겼으며, 다른 P0 MCU 통합 작업과 병행 중인 상태에서 같은 코너를 새 상수로 반복 실행하지 않았다.
