# r530 후속 인계 절차

현재 단계는 자원 경로 수정과 Windows 준비 완료입니다. 실행 횟수 0, source freeze 없음, WSL은 root 소유입니다. 이전 r520 승인은 이 새 실행의 승인으로 사용하지 않습니다.

1. Root가 최종 공통 소스/추가 파일과 SHA, 관련 검사 결과를 확정하면 [source_selection.json](source_selection.json)에 r520 동결본 대비 허용 변경을 적습니다. r526/r528/r529뿐 아니라 root가 승인한 모든 최종 차이를 빠짐없이 대조합니다. 기존 native binary는 재빌드하지 않습니다.
2. 명시적 WSL 소유권/단 1회 실행 인계 뒤 `check_recipe.py --final`, [prepare_once.sh](prepare_once.sh)로 실제 import 경로·SHA·경쟁 프로세스 0을 확인하고 입력 ZIP을 동결합니다. 같은 기하의 601 표본 검사는 재실행하지 않습니다.
3. Root 검토 및 실행 담당 무결성 확인에 대한 `root_execution_review.json`을 기록한 뒤 [execution_v1/execute_once.sh](execution_v1/execute_once.sh)를 한 번만 사용합니다. PID/로그/실제 입력 해시를 회수하며 같은 실패 조건 재시도는 하지 않습니다.
4. 런타임 월드의 collision digest와 자원 존재 결과, 초기 actor/ego 간격을 확인합니다. 원래 12sim초와 기존 8sim초 무진행 STOP 조건을 유지합니다. [execution_v1/stop_once.sh](execution_v1/stop_once.sh)와 기존 STOP 절차를 사용하고 임의 강제 종료를 정상 종료로 취급하지 않습니다.
5. [execution_v1/finish_collection.bash](execution_v1/finish_collection.bash)로 원 판정과 별도 분석, 실제 STOP, 정상 종료, 입력 무결성, 소유 PID 0을 구분해 회수합니다. Guard/발행/수신·warmup·저속 재획득 증거를 저장 입력에서 분석하되 원 보고서를 수정하지 않습니다.

현재 [preparation_receipt.json](preparation_receipt.json)의 공통 소스 차이 목록은 준비 시점 참고치입니다. root가 공통 작업 중이므로 최종 선택 또는 해시 동결을 뜻하지 않습니다.
