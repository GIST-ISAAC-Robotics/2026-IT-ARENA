# r534 대표 단독 전체랩 준비

**Windows 준비만 완료했습니다. WSL 진입·소스 동결·실제 주행은 0회입니다.** 먼저 r530 교통 12sim초 1회의 결과를 회수하고 root가 검토한 뒤, 최종 소스와 WSL 소유권을 별도로 인계받아야 단독 주행을 시작할 수 있습니다. 교통 `passed`를 임의로 바꾸거나 자동으로 단독을 시작하지 않습니다.

단독 조건은 r475 실제 완주의 기본 grid3 출발, actor 0·시설 3개, `g1_open_1p5`, `medium_v1`, RTF .25, 80sim초 상한, 8sim초 무진행 STOP입니다. r492의 +20mm/+3deg 출발 변형은 추가하지 않습니다. r475와 r492가 동일하게 사용한 단독 평가기·원 판정·STOP 절차를 재사용합니다. 새 시뮬레이션 결과는 아직 없습니다.

[field_world.sdf](field_world.sdf)는 r530 월드와 byte-identical이며 SHA256은 `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`입니다. 벽 collision/visual 475개, 시설 SDF·지주 STL·차량·센서·물성은 동일합니다. 변형은 첫 직선 두 벽 y3.0–11.5m의 진폭30mm·주기1m·두께5mm·20mm 접합 틈이라는 **미실측 가정**에 한정합니다. 전체랩을 달려도 전 코스의 모든 벽 굴곡을 검증한 것은 아닙니다. 복사한 기하/자원 JSON은 r530 당시 원 경로와 SHA를 유지한 근거이며, 현 실행 경로·일치는 [preparation_receipt.json](preparation_receipt.json)에 기록합니다.

준비한 수집 보완은 다음과 같습니다.

- 기존 `/safety/status` 구독의 QoS·상태 갱신을 유지하며 전체 JSON을 `safety_status.jsonl`에 기록합니다. `receipt_monotonic_s`는 평가기 수신 시각이며 guard 내부 callback 순서로 해석하지 않습니다. 이미 발행되는 current/retained memory 세부 내용은 필터링하지 않습니다. 같은 계측은 root 승인에 따라 r530 로컬 runner에도 적용했습니다.
- 실제 `source_snapshot.zip`에 파생 월드·시설·외부 자산·로컬 runner를 명시적으로 포함합니다. 실행용 전체 frozen ZIP과 실제 snapshot을 각각 검사합니다.
- r492의 runtime/evaluator 벽 polygon 대조를 재사용하고, r530 자원 존재 검사를 Gazebo 시작 전에 실행하도록 연결했습니다.
- [collect_evidence.py](collect_evidence.py)는 완주 성공이나 특정 Windows launcher 영수증을 전제하지 않습니다. 미완주·원 실패·누락 파일을 그대로 기록합니다. 정상 종료·STOP·PID0·입력 무결성·trace footer는 각각 구분하며 원 판정을 수정하지 않습니다.
- [execution_v1/finish_collection.bash](execution_v1/finish_collection.bash)는 PID 회수·입력 검사·분석을 순서대로 시도하고 각각의 종료 코드를 보존합니다. 분석 오류가 PID 회수 확인을 건너뛰게 하지 않습니다.

[PLAN.md](PLAN.md), [source_selection.json](source_selection.json), [traffic_review_template.json](traffic_review_template.json), [recipe_provenance.json](recipe_provenance.json)을 인계합니다. 최종 제어 소스 선택은 아직 pending이며 공통 소스·Jetson·원 r520 실패에는 변경이 없습니다.
