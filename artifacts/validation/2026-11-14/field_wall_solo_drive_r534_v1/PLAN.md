# r530 교통 이후 r534 단독 실행 연결

현재는 실행 대기 준비입니다. 아래 절차 자체가 실행 승인을 뜻하지 않습니다.

1. Root가 최종 r532 guard/r535 UI 및 이미 통합한 r523/r524/r526/r529를 포함한 선택 SHA·검사 근거를 전달하고 WSL을 반환하면 r530의 실제 import·소스 동결·입력 ZIP 검토 후 **교통 12sim초 단1회**를 실행합니다. 같은 실패 조건은 반복하지 않습니다.
2. r530의 원 보고서·STOP·정상 종료·잔여 PID0·입력 무결성을 회수합니다. 원 `passed`와 helper 계획/양수 발행/정상 FOLLOW·실제점유 정지는 별도로 분석합니다. 원문 `/safety/status`와 raw scan·follow trace·actor 평가용 위치를 사용하며 기록에 없는 guard 내부 순서를 재현 사실로 주장하지 않습니다.
3. 필요하면 이 폴더의 `collect_evidence.py --artifact artifacts/validation/2026-11-14/field_wall_resource_drive_r530_v1 --kind traffic`으로 원 보고서를 바꾸지 않는 수집 영수증을 작성합니다. PID 확인과 STOP 수집은 분석 전에 완료합니다. 분석 실패도 정상 종료 또는 제어 실패와 분리합니다.
4. Root가 실제 결과를 검토하고 단독 진행을 결정한 뒤에만 `traffic_review_template.json` 형식으로 별도 `traffic_review.json`을 작성합니다. 교통 원 `passed` 및 report/integrity/archive SHA를 기록합니다. 교통 실패만으로 단독을 자동 취소하거나, 교통 성공만으로 단독을 자동 실행하지 않습니다.
5. r534의 최종 `source_selection.json`은 **실제 r530 동결 소스**를 기준으로 정확한 추가 변경을 적습니다. native 재빌드·같은 회귀 반복은 하지 않습니다. root 소유권 인계와 별도 단1회 승인 뒤 `prepare_once.sh`로 실제 import/경쟁0/기하를 확인하고 동결합니다.
6. 별도 `root_execution_review.json`에 prepared/archive SHA, execution_limit=1, repeat_on_failure=false, wsl_exclusive_owner_confirmed=true를 확인하고 `execution_v1/execute_once.sh`를 한 번 실행합니다. 기본 출발에서 actor 없는 전체랩, 80sim초 상한과 기존 무진행 STOP을 사용합니다.
7. `execution_v1/observe.py`로 실제 진행·8sim초 무진행·접촉을 확인하며 필요하면 `stop_once.sh`의 기존 명시 STOP을 사용합니다. 종료 뒤 `finish_collection.bash`로 회수합니다. 실제 PID/로그·sim 시간·wall 시간·최장정지·완주·접촉/노면·원 STOP·14자식 정상종료 여부·강제회수·PID0·소스 무결성을 원 결과 그대로 보고합니다.

수집기 변경은 제어 명령·센서·속도·STOP·판정선을 바꾸지 않습니다. 버퍼 기록의 host 비용은 존재하므로 수집 시간을 Jetson 성능 또는 별도 순수 계산 벤치마크로 해석하지 않습니다. 실제 runtime 자원·ROS import/프로세스 검사는 아직 수행하지 않았습니다.
