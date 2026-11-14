# r513 r512 연속 24cm 주시 후보의 오른쪽 옆차 대표 1회 — 실행 전 준비

r499/r503과 **같은 `right_side_start` 12 sim초** 대표를 r512 공통 후보로 한 번 실행합니다. actor +0.50 m·오른쪽 −0.23 m, 0.50 m/s, `box` 200×150×220 mm(보이는 상대 가정), RTF0.25, `g1_open_1p5`/`medium_v1`, 시설3, 원 STOP·원 판정·스캔 기하, 현 CAD/C1/IMU는 r503과 같습니다. r499·r503의 원본과 실패 기록에는 쓰지 않습니다. 잔디 우선순위 작업은 이 시험에 섞지 않습니다.

## r503 대비 차이

- control은 `moving_pass.py` 1파일입니다(`f567876f`→`bcd58132`, [선택](source_selection.json)). r502의 제동 뒤 전용 분기를 없애고, 같은 현재 경로의 24 cm 양수 후보를 횡이동 시작부터 비교합니다(GPT 설계). r503 저장 125계획 중 seq85–97의 13명령만 바뀐 재생 근거가 있습니다([r512](../continuous_pass_preview_r512_v1/RESULT.md)). 폐루프 효과는 이번 1회로 처음 확인합니다.
- `tests/test_moving_path_preview.py`(`68c16dab`→`e956d5ed`)는 r503 동결에 이미 있던 시험 파일의 변경이라 비제어로 기록합니다. sim에서 불러오지 않으며 이번에 실행하지 않습니다.
- r498 출력동등 최적화 3파일과 그 밖의 공통 입력은 r503 동결값과 같습니다.
- config는 `scope`·`registered_at`·`launch.entry_g1`만, 인자는 출력 경로 이름만 다릅니다. case/actor/acceptance는 r503(=r499)과 같습니다.
- 시설·actor·case·코스가 같으므로 601표본 정적 기하 결과를 [그대로 재사용](geometry_right_side.json)합니다.
- native `.so`는 같고 재빌드하지 않습니다. r496 native166 production 목록의 `moving_pass.py`가 바뀐 상태이며, native166은 다시 실행하지 않았습니다.

## 절차

1. 동결 전 대조: [check_recipe.py](check_recipe.py) → [preparation_receipt.json](preparation_receipt.json).
2. root 검토 뒤 `freeze_authorized=true`, `prepare_once.sh`(preflight import→freeze→check).
3. root가 `root_execution_review.json`을 쓴 뒤 `execution_v1/execute_once.sh`를 **단1회** 실행합니다. 감시는 `observe.py`(8 sim초 무진행·벽 접촉), 필요할 때만 `stop_once.sh`, 종료 후 `finish_collection.bash`를 씁니다.
4. 실패해도 재실행하지 않습니다. 실패하면 `extract_failure.py`·`attribute_once.sh`로 최초 정지 대표 입력을 회수합니다.

현재 동결·등록·실제 주행은 **0회**입니다.
