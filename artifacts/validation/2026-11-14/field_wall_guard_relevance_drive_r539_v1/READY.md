# r539 같은 교통 조건 비교 준비

**Windows 준비 완료, WSL 진입·동결·실행0회입니다.** r538 guard 후보의 root 최종 통합 SHA/관련 검사/독점 인계와 단1회 실행 승인을 기다립니다. 예상 생산 코드 변경은 `guard_memory_command.py` 하나이며 관련 tests는 최종 전달 뒤 선택합니다.

기준은 실제 r530의12sim초 원 실패입니다. r530은2.88m 진행·최장정지0.100초·접촉0·STOP6개/정상종료/PID0을 확인했지만 최종 앞차 간격3.4217m로 `final_gap=false`였습니다. 후반0.1m/s 제한의 동일조건 비교를 준비하며 원 실패를 덮어쓰지 않습니다.

[field_world.sdf](field_world.sdf)는 r530과 byte-identical이며 SHA256은 `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`입니다.475개 벽·같은 actor220mm/0.5m/s/+0.5m/−0.23m·시설3개·12sim초·RTF.25·기존 속도/STOP/원판정을 유지합니다. 자원 경로·지주·바닥·잔디·물성도 같습니다. 새 기하 감사나601 표본 계산은 반복하지 않고 기존 정적 근거와 동일 SHA를 재사용합니다. 벽 변형은 첫 직선 두 벽의 미실측 가정이며 전 코스 변형이 아닙니다.

`traffic_runner.py`와 전체 `/safety/status` 기록도 r530과 같습니다. 실제 snapshot에 파생 월드/자산/로컬 runner를 넣고, runtime 자원 존재·벽 collision digest를 확인하는 연결을 유지합니다. [same_scene_receipt.json](same_scene_receipt.json)과 [recipe_provenance.json](recipe_provenance.json)에 원 실행·새 파일 해시를 기록했습니다.

원 수집기의 `positive_normal_follow_plans`는 selector 문자열 계수이므로 handoff FOLLOW 복귀 지표로 사용하지 않습니다. r530과 같은 [analyze_controller_guard.py](analyze_controller_guard.py)를 종료 후 실행해 helper계획/실제발행/정상경로복귀/guard cap·점유정지를 별도로 집계합니다. 원 판정·원 수집 값은 그대로 보존합니다.

다음 순서는 root 최종 SHA/test 확인 → 실제 import·경쟁PID0 → frozen ZIP/입력무결성·root 인계 영수증 → 단1회 실행 → 기존 STOP/정상 회수/PID0 → 원 판정과 보충 집계입니다. 실패/기술 오류에도 자동 반복하지 않습니다. r534 전체랩은 이번 결과 검토 뒤 별도로 결정합니다. [source_selection.json](source_selection.json)의 승인 값은 모두false이며 [preparation_receipt.json](preparation_receipt.json)은 준비만 확인합니다.
