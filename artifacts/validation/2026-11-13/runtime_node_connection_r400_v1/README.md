# r400 명시 운영 노드 연결 — 공통 이관 완료

현재 G1G2Node는 별도 선택 없이는 기존 직렬 실행을 유지한다. 새 `g1_runtime_policy=cad_runtime_r377_v1`을 명시하면 r398 정책과 공통 두 계산 슬롯을 처음부터 생성한다. 직렬 자식을 만든 후 monkeypatch로 바꾸는 이전 시험 진입점을 쓰지 않는다. 기본off·native·fresh/current worker·retained reference 조건을 확인한다. hardware_shadow의 출력 격리는 보존하며 운영 환경은 별도명시다.

`RuntimeDualDelivery`의 후보를 기존 LocalPursuit 발행 경계에 통과시킨 뒤 실제 원시각/속도/각도 발행을 확인한다. STOP/RESET은 두 슬롯 세대를 끊는다. 시각/기하/점유 기준을 바꾸지 않는다. 명시 양쪽 정책 자식의 close를 finally에서 호출하고 실패해도pipe는 닫는다. r398 패키지의 이관 동등성 검사가 먼저 통과해야 실행한다.

실행준비: r400 staged가 먼저, r398 staged, common src 두패키지, common scripts/tests 순서로 PYTHONPATH를 두고 실제 native 설치 경로를 유지한다. source순수검사 `test_runtime_node_backend.py`, `tests/test_runtime_dual_spawn.py` 및 실제ROS `test_runtime_node_ros.py`를 한 묶음으로 실행한다. 기존 비선택 ROS 경계검사는 영향 범위만 추가한다. domain219, 실제port/Jetson/모터 없음, 외부180초 상한과 소스전후해시/자식회수. 상태생성/STOP/RESET만으로 센서→양수주행을 증명하지 않는다.

독립 정적 검토 후 실제 latest 제출 조건을 기록하도록 receipt만 분리했다. 기존 fresh-only의25ms조건을새동작에잘못표시하지않는다.

검증 결과: staged 21검사와 공통 이관19검사를 통과했다. 실제ROS 노드 두 계산 자식·STOP/RESET 세대 변경·정상 종료를 확인했다. [공통검사](verify_common_v1/)의19개에는 앞서 끝낸 spawn4개를 중복 포함하지 않았다. 실제 센서 연결·양수 주행·MCU 물리 출력은 아직 미실행이다.
