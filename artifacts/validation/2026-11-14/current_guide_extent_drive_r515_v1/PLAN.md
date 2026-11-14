# r515 현재 전방 외벽 범위 후보 — 같은 오른쪽 옆차 대표 1회 준비

r513과 같은 `right_side_start` 12 sim초 대표를 한 번 실행합니다. actor +0.50 m·오른쪽 −0.23 m, 0.50 m/s, `box` 200×150×220 mm, RTF0.25, `g1_open_1p5`/`medium_v1`, 시설3, STOP과 원 판정 및 별도 목표는 그대로 유지합니다. 기존 실패와 원시 입력은 보존합니다.

- 제어 변경은 root가 최종 확인할 `moving_pass.py`의 r514 현재 전방 외벽 범위 후보입니다. 최종 SHA와 시험 근거는 [source_selection.json](source_selection.json)에 기록한 뒤 동결합니다.
- 시험 파일 변경은 비제어 입력으로 구분하며 주행에서 실행하지 않습니다. 그 외 공통 생산 입력 차이는 자동 채택하지 않습니다.
- 시설·actor·case·코스가 같으므로 [기존 601표본 기하](geometry_right_side.json)와 native 바이너리를 재사용합니다. 새 빌드나 전체 회귀 반복은 하지 않습니다.
- WSL 소유권과 소스 인계 전에는 Windows 파일 준비만 합니다. 소유권 인계 후 실제 import·경쟁 프로세스·소스 해시를 확인하여 [prepare_once.sh](prepare_once.sh)로 동결합니다.
- 동결과 root 검토가 끝난 뒤 [execute_once.sh](execution_v1/execute_once.sh)를 단1회 실행합니다. 8 sim초 무진행 또는 벽 접촉은 기존 STOP 절차로 처리합니다. 실패 반복은 없습니다.
- 종료 후 원 평가, 별도 목표, STOP, 정상 종료, 실제 입력 무결성, 잔여 PID를 각각 기록합니다. 원 판정을 덮어쓰지 않습니다.

준비 시점에는 동결·등록·실제 주행 0회입니다. Jetson·센서·실물 출력·전체 교통 랩·실시간 10 Hz 검증과 구분합니다.
