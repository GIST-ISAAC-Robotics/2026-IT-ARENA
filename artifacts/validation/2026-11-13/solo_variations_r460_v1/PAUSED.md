# r460 일시 중지 인계

사용자의 FAST 설정 변경을 위한 즉시 중지 요청으로 현재 파일을 저장하고 중지했습니다. 공통 소스·r457 입력은 수정하지 않았으며 WSL/ROS/Gazebo/빌드/주행은 실행하지 않았습니다. 이 작업에서 실행 중인 소유 프로세스는 없습니다. root가 전달한 상태는 r457도 초기 WSL E_UNEXPECTED로 실제 주행 미시작입니다.

## 저장한 내용

- 기준 recipe의 runner/evaluator/finalize/시설/helper/header/하위 launch를 r460에 사본으로 보존했습니다. `baseline_copy.json`에 원 SHA가 있습니다.
- `variation_launch.py`, `launch/simulation.launch.py`: 원 XML의 벽 collision/visual 변형을 common runtime override보다 먼저 적용하며, 원 source.parent 기준 URI 절대화를 먼저 수행합니다. 기존 old610 중복 beam 제거와 RTF .25를 유지합니다. spawn은 공통 모듈의 artifact 사본 함수에서 slot 선택 직후 지역 변수만 변경합니다.
- `launch_adapter.py`, `run.py`: artifact 환경변수로 세 조건 중 하나를 선택합니다. 평가기의 사전 장애물은 같은 raw 변형에서 읽고, 종료 시 실제 runtime 월드의 벽 polygon과 대조하도록 연결했습니다. 시설 3개는 기준 STL slab을 유지합니다.
- `prepare_execution.py`: 기존 정적 준비와 분리된 connection manifest/최종 동결 준비 코드입니다. nominal passed/full_lap/정상 회수 및 주 담당 실행 검토를 요구합니다. 아직 이 스크립트를 실행하지 않았고 `connection_v1`, `execution_prepared.json`, 실행용 source ZIP은 생성하지 않았습니다.
- `runtime_preflight.py`, `stop_owned.py`, `execution/*`: 후속 import 확인 및 소유 실행/STOP/관측/회수 recipe 초안입니다. 실행하지 않았습니다. 주행 명령은 `python3 .../run.py <case>`이며 STOP argv 비교도 같은 세 인자입니다.

## 검사 상태와 남은 작업

아직 AST/연결/기하 검증을 수행하지 않았습니다. 현재 상태를 검증 완료 또는 실행 준비 완료로 취급하지 않습니다. Windows 기본 Python에서 shapely import가 없다는 환경 확인만 했으며 패키지는 설치하지 않았습니다. 새 정적 검사는 표준 라이브러리와 원 함수 AST를 사용하도록 구상했지만 파일은 아직 작성하지 않았습니다.

명시 재개 후: (1) 현재 초안의 제한 diff 검토, (2) Windows 정적 검사 구현·실행으로 common 원 XML 처리와 세 변형의 collision/visual batching·URI·spawn 위치·평가 collision 일치를 확인, (3) 연결 manifest 생성과 원본 SHA 불변 확인, (4) README/RESULT 마무리 및 주 담당 인계 순서입니다. r457 nominal 성공 전 r460 실행은 보류하며 WSL 자원 사용도 별도 배정 이후입니다. 최초 root 정적 `prepared/*`와 runtime probe 실패는 그대로 보존되어 있습니다.

전체 신규/복제 파일 SHA는 `paused_files.json`을 참조합니다. 공통 이관 대상은 없습니다.
