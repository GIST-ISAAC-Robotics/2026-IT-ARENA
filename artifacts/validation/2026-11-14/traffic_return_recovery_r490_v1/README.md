# r490 동일 앞차 10sim초 대표 준비

r478의 실제 recipe를 재사용한다. 앞차는 처음4초 .30m/s,4.0~4.2초 .40,4.2~4.4초 .50,이후 .58m/s이며, 오른쪽 .12m·높이220mm·초기 중심 간격1.6m·기간10sim초·시설3개를 유지한다. 차량/센서/물성/속도·RTF .25·원 간격/노면/접촉 판정과 STOP/정상 회수 기준은 바꾸지 않는다. 원 r478 실패는 보존한다.

승인된 제어 변경은 r481 v2 두 파일, r482 `current_lane_preference.py`, r486 `moving_pass.py`와 `bypass_entry_trial.py` 총5개다. 새 관련 테스트2개를 별도 동결하고, 기존 manifest의 `jetson_access.py` 변경은 비실행 보조도구로 구분한다. **r489 독립 보호 수정은 포함하지 않는다.** 나머지 공통 입력과 native binary는 r478 기준과 같아야 한다. 실제 import/시설 연결 확인 후 동결하며 새 빌드·회귀 반복은 하지 않는다.

WSL 준비 명령(ROS init/노드/시뮬 실행 없음):

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_return_recovery_r490_v1/prepare_once.sh'
```

`prepared.json`/`sources.zip` SHA와 source delta를 root에게 전달한다. 실제 주행은 root가 `root_execution_review.json`에 `root_reviewed=true`, `execution_limit=1`, `repeat_on_failure=false`, `prepared_sha256`, `source_archive_sha256`을 기록하고 시작을 지시한 뒤에만 수행한다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_return_recovery_r490_v1/execution_v1/execute_once.sh'
```

등록/run1 gate가 중복 실행을 막는다. 담당자는 기존 `observe.py`로8sim초 무진행/접촉을 감시하고 필요 시 `stop_once.sh`의 owned STOP을 사용한다. 종료 후 `finish_collection.bash`가 원 평가 요약·동결 무결성·PID/partition 회수를 기록한다. 성공할 때까지 반복하지 않으며 원 실패도 그대로 남긴다. 이10sim초 대표는 전체랩이나 실물 증명이 아니다.
