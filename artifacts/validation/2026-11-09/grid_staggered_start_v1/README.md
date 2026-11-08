# 공식 5대 그리드 차례 출발 — 보충 대표 성립, 원 판정 실패 보존 (2026-11-09)

[실행 전 계획](PLAN.md)에 따라 r48 제어기와 r48 등록 인자를 바꾸지 않고, [11/8 그리드 대표](../../2026-11-08/grid_pack_newcourse_v1/README.md)의 **상대 출발 각본만** 바꿔 Gazebo를 **한 번** 실행했다. 자차는 grid3, 상대는 grid4(주 상대)·grid0·grid1·grid2다. 초록 뒤 grid2 0.20초, grid1 0.80초, grid0 1.40초, grid4 2.00초에 각각 0.50 m/s로 출발하며 창은 12초다. 상대 높이 180 mm는 시험 가정이다.

## 실행 전 점검

| 항목 | 결과 |
|---|---|
| 파생 각본 | [prepare.py](prepare.py)로 기준 설정·overlay에서 사례 이름·창·각본만 변경. 기준 SHA-256을 `derived_from`에 기록 |
| 기하 | [geometry.json](geometry.json): 공식 칸 일치, 12초 경로 도로/벽 안, 마지막 위치 s=48.76 m(첫 코너 1 m 전 한계 50.34 m), 상대 쌍 최소 30 mm(grid4–grid0/1 옆면)·겹침 0 |
| 관련 회귀 | `189 passed` ([regression.log](regression.log)) |
| r48 출처 | r48 등록 929소스 현재 0변경, r48 계약 승인 해시 일치. r48 이후 새 파일은 `scripts/run_new_course_solo.py`와 이 폴더 파일뿐 ([register.log](register.log)) |
| 실제 import/launch | 실행 직전 163항목(두 패키지 전체 `.py`, r48 모듈 목록, launch/설정, 실행 파일 진입점)이 등록 src 바이트와 일치 ([runtime_check.json](runtime_check.json)) |

## 원 실행과 집계 오류

원 실행 종료 기록은 `HARNESS_EXIT=1 SURFACE_EXIT=0 INTEGRITY_EXIT=0 LEFTOVER_PGREP=1 ASSESSMENT_EXIT=1 WALL_S=284`이다([run1.exit](run1.exit)). 원 `run1/report.json`의 평가는 `KeyError: 'minimum_progress_m'`로 끝났고, 그 결과 `harness_valid=false`, gate가 비었다. 원인은 주행이 아니라 **파생 설정 작성 오류**다. 원 평가기는 `acceptance[사례 이름]`으로 판정선을 찾는데, 사례 이름을 `five_vehicle_grid_staggered`로 바꾸면서 기존 `acceptance.five_vehicle_grid` 항목을 함께 옮기지 않았다. 원 report·보충 판정 오류 기록([assessment_run1.log](assessment_run1.log))은 그대로 보존한다.

**재평가는 새 주행이 아니다.** [reevaluate.py](reevaluate.py)는 저장된 trace·자세·접촉·명령 기록에 기존 `evaluate_run`/`finalize_verdict`를 다시 적용한다. 판정 항목은 값을 바꾸지 않고 새 이름에도 복사한 [수정 설정 사본](reeval/new_track_course_acceptance_fixed.json)을 쓴다. 같은 방법을 현재 소스로 실행한 r48 run1에 적용한 결과, evaluation·gates·integrity·stop_checks·유효성·통과 8항목이 하네스 내부 결과와 모두 같았다([control_r48.json](reeval/control_r48.json)). 보충 판정은 등록된 `assess.py`를 바꾸지 않고 재평가 report만 읽게 하여 다시 집계했다([reassess.py](reassess.py)).

## 결과

- **원 판정: 실패 보존.** 재평가 원 gate 11개는 모두 통과했다. 차체 완전 도로 안 기준도 도로 밖 표본 0으로 통과했고, 주 상대 최소 간격은 0.300 m(150 mm 기준 통과)다. 그러나 추가 상대 grid0의 초기 공식 옆면 간격 0.030 m가 80 mm 기준에 미달하여 `behavior_gates_passed=false`, `passed=false`다. 첫 미달 시각은 초록 시점(7.401초)이며 공식 배치에서 생긴 것이다. [재평가 report](reeval/report_reevaluated.json)
- **보충 판정 15개: 모두 통과.** [보충 판정](reeval/assessment_reevaluated.json)의 결과는 아래와 같다.

| 항목 | 값 |
|---|---|
| 상대 실제 출발(5 mm 이동, 초록 기준) | grid2 0.219 · grid1 0.829 · grid0 1.439 · grid4 2.029초. 등록 대비 +0.019~+0.039초, 순서 일치 |
| 상대 수신·추적 | 4대 모두 하네스 유효, 첫 기록 자세가 공식 칸과 0 mm, 추적 오차 최대 0.012 m |
| RGB 출발 | 출발 신호 판정 통과, 실제 초록 뒤 출발 허가 +0.281초, 거짓 출발 없음 |
| 자차 첫 움직임(≥0.10 m/s) | 초록 뒤 **2.806초**(grid4 출발 뒤 0.806초) |
| 진행·정체 | 12초 창 끝 **4.49 m**(trace 10 Hz, 원 평가 4.53 m), 움직인 뒤 0.05 m/s 미만 최장 0초 |
| 움직인 뒤 최소 간격(표본/이동 하한) | grid4 0.671/0.670 · grid0 0.677/0.676 · grid1 1.476/1.476 · grid2 2.276/2.275 m |
| 상대끼리 | 기록 자세 1,200 동시 표본, 최소 0.030 m(grid4–grid0, grid4–grid1 옆면), 겹침 0 |
| 접촉 | 자차–상대 접촉 메시지 0, 통합 접촉 사건 0, 자차 겹침 표본 0 |
| 노면 | 10,246표본, any/center/full 세 정의 모두 **잔디 바퀴 최대 0개**, 벽 바퀴 0 |
| STOP·종료 | 명시 STOP 정착 0.795초, 0.5초 조밀 증거 최대 0.0046 m/s, 변위 0.009 m, MCU STOP, 정상 종료, 잔여 프로세스 0 |
| 소스 | 실행 후 951개 입력·사본 불변 |

행동 기록은 FOLLOW 93·HOLD 23(`handoff_follow`)·CRUISE 0이다. 자차는 끝까지 주 상대 뒤에 머물렀고, 창 끝 간격은 0.760 m다. 자차가 늦게 움직인 것은 바로 앞 grid4가 2.0초에 출발한 각본과 맞는다. 다만 출발을 막은 직접 원인은 행동 기록으로만 추정한 것이며, 제어 내부를 따로 분해하지는 않았다.

## 한계

- 상대 접촉 센서는 평가기가 순간 이동시키는 정적 모델에 붙어 있다. 상대끼리의 접촉 사건은 생성되지 않을 수 있으므로, 상대끼리 무접촉의 근거는 기록 자세의 기하 간격이다. 30 mm 옆면 간격은 공식 배치에서 생긴 값이며 실제 차량 상호작용을 모사하지 않는다.
- **RGB 가림률은 측정하지 않았다.** 출발 허가 +0.281초는 11/8 대표(+0.278초)와 비슷하다. 180 mm 상대가 신호를 가렸다거나 가림 뒤 늦게 출발했다고 주장하지 않는다.
- 11/8 대표와는 제어기(r48)와 각본 두 가지가 다르므로, 잔디 0개나 첫 움직임 시각의 차이를 각본 효과만으로 해석하지 않는다.
- 단일 실행, 12초 직선 출발 구간만 다뤘다. 모든 그리드 조합, 경기 거리, 실제 독립 5대, 실차를 검증한 것은 아니다.

## 재현과 파일

- 실행 전: `bash checks.sh`(기하·회귀), `bash run.sh --check`(등록·런타임 출처 건식 점검, [precheck.log](precheck.log)). 평가기 시험은 `bash selftest.sh`로 11/8 실행을 읽기 전용으로 사용했다(각본이 달라 `scheduled_starts`만 실패하는 것이 기대값). [selftest_baseline_assessment.json](selftest_baseline_assessment.json)
- 실행: `bash run.sh`(이미 실행됨, 다시 실행하면 거절). 재평가: `bash reevaluate.sh`(기존 재평가 산출물이 있으면 거절).
- 원시 대용량 기록(`follow_decisions.jsonl`·`scans_received.jsonl`·자세 등)은 기존 Git 제외 규칙을 따른다.
