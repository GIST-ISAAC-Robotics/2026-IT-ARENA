# r503 r502 주시거리 후보의 오른쪽 옆차 대표 1회 — 실행 전 준비

r499와 **같은 `right_side_start` 12 sim초** 대표를 r502 공통 후보로 한 번 실행합니다. actor는 자차 앞 +0.50 m·오른쪽 −0.23 m, 0.50 m/s, `box` 200×150×220 mm(보이는 상대 가정)이고, RTF0.25·`g1_open_1p5`/`medium_v1`·시설3·원 STOP·원 판정·스캔 기하·현 CAD/C1/IMU는 r499와 같습니다. r499의 원본·실패·로그는 그대로 보존합니다.

## r499 대비 차이

- control 4파일만 바뀌었습니다([선택](source_selection.json)).
  - `moving_pass.py`: r502의 같은 관측 경로 24 cm 주시 후보(GPT 설계). 저장 입력126판단 중 seq98/99 두 명령만 바뀐 재생 근거가 있습니다([r502](../moving_pass_facility_r502_v1/RESULT.md)).
  - `traffic_solo_gap.py`·`local_follow.py`·`pass_release_trial.py`: r498 **출력 동등 최적화**입니다. r501 ARM 508/508 일치 근거가 있으며 판단 변경이 아닙니다([통합](../threat_view_reuse_r498_v1/integration_v1/RESULT.md)).
- 비제어 변경 2파일(`scripts/steering_zero_profile.py`, `race_ops_page.py`)은 sim launch에서 import되지 않습니다. 동결 시점 SHA로 기록합니다.
- 추가 시험 파일 `tests/test_moving_path_preview.py`·`tests/test_threat_view_transfer.py`를 동결에 등록합니다(실행하지 않음).
- config는 `scope`·`registered_at`·`launch.entry_g1`만, 인자는 출력 경로 이름만 다릅니다. case/actor/acceptance는 r499와 같습니다.
- 시설·actor·case·코스가 같으므로 r499의 601표본 정적 기하 결과를 [그대로 재사용](geometry_right_side.json)합니다.
- native `.so`는 r496/r499와 같고 재빌드하지 않습니다. 다만 r496 native166 production 목록에 `moving_pass.py`가 있어 새 `moving_pass.py`에 대한 native166은 재실행하지 않은 상태입니다.
- 설치는 `install`→`build/arena_autonomy/arena_autonomy`→`src/...` symlink이므로 실행 소스는 `src/`입니다.

## 절차

1. 동결 전 대조: [check_recipe.py](check_recipe.py) → [preparation_receipt.json](preparation_receipt.json).
2. root 검토와 `race_ops_page.py` 담당 완료 후 `source_selection.json`의 `freeze_authorized=true`로 바꾸고 `prepare_once.sh`(preflight import→freeze→check)를 실행합니다.
3. root가 prepared/ZIP SHA를 검토해 `root_execution_review.json`을 기록한 뒤 `execution_v1/execute_once.sh`를 **단1회** 실행합니다. 감시 `observe.py`, 필요할 때만 `stop_once.sh`, 종료 후 `finish_collection.bash`를 씁니다.
4. 실패해도 재실행하지 않습니다. 원 판정과 별도 목표를 분리해 기록하고, 실패 시 최초 정지 대표 입력을 회수합니다.

현재 동결·등록·실제 주행은 **0회**입니다.
