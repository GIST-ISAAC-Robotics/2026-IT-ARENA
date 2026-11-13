# r492 단독 변형 guard 회복 확인 — 실행 전 초안

현재 상태: root의 r489 통합/180회귀 완료 뒤 준비 v2가 exit 0으로 끝났습니다. 1,375개 입력 동결·실제 import·기존 launch 선택 전달 확인까지 완료했고 실제 주행은 root 검토 대기 중입니다. `preflight_v2/freeze.json`, `runtime_preflight/result.json`이 최신 준비 근거이며 아래 내용은 최초 초안의 범위/절차 기록입니다. 준비 v1의 launch 순회 helper 오류는 `preflight_v1/import.log`에 보존했습니다. ROS constructor/pytest/build/Gazebo는 반복하지 않았습니다.

r485의 `start_left20_yaw_plus3`(+20 mm, +3 deg)와 actor 0·시설 3개·기하·센서·물성·속도·RTF .25·최대 80 sim초·8 sim초 무진행 owned STOP·접촉 및 노면 판정을 그대로 재사용합니다. 목표는 root가 통합할 r489 독립 보호 수정의 영향을 같은 변형 1회에서 확인하는 것입니다. 이전 실패를 성공할 때까지 반복하는 실행이 아닙니다.

Windows에서 실행 구조만 복사했고 제어 소스 복사·새 동결·빌드·WSL·ROS·Gazebo 실행은 하지 않았습니다. `draft_provenance.json`에 r485 원 prepared/archive/report와 파일별 SHA를 남겼습니다. `expected_header.json`, 변형 world 및 시설 SDF는 원 바이트 그대로입니다. `prepared/manifest.json`은 기존 geometry recipe 자료이며 새 실행 source 동결을 뜻하지 않습니다.

다음은 root의 r489 통합/검사 및 자원 배정 뒤에만 진행합니다.

1. 최종 공통 source delta와 native 재사용 가능 여부를 root 통합 영수증으로 확정합니다. r485 이후의 r486 2파일과 r489 승인 파일 범위가 포함되어야 합니다.
2. 복사한 `prepare_execution.py`·`runtime_preflight.py`의 예전 r477/3파일 전제와 `source_selection.json`을 새 승인 범위로 갱신합니다. 지금은 모든 통합/동결/WSL 승인 플래그가 false이며 자동 준비 불가 상태입니다.
3. 실제 import/입력 확인 → 새 archive/freeze → root 실행 검토 후 단일 실행합니다. 현재 실행 영수증·run1·새 archive는 없습니다.

예정 명령 위치는 `execution/prepare_once.sh`와 `execution/execute_once.sh start_left20_yaw_plus3`입니다. 아직 실행 가능한 완성본으로 판정하지 않습니다. r490 교통 실패의 별도 반복 실행은 포함하지 않습니다.
