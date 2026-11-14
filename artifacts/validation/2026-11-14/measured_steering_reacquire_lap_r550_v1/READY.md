# r550 Windows 준비 완료

**준비만 완료했으며 WSL 진입·빌드·소스 동결·실제 주행은0회다.** root의 r549 회귀 완료와 별도 최종 소스/WSL 인계 후에만 단독1회를 실행한다. 실패 후 같은 조건을 자동 반복하지 않는다.

기준은 r544 attempt2다. 같은 measured YAML·fieldworld·actor0·grid3·시설3개·80 sim초 상한·8 sim초 무진행 STOP·RTF .25·g1_open_1p5·medium_v1·원 판정을 유지한다. [명령 인수](arguments.json)는 artifact 경로만 새 폴더로 바꿨다. [runtime 조향 감사](steering_runtime_audit.py)의 실제 controller/guard/virtual MCU/actuator 및 SDF joint/plugin 7검사는 기존 성공한 코드와 경로 치환 외 동일하다. 중앙 명령 상한 .28475436568260193 rad·내륜 관절 상한 .30666234131856146 rad를 유지한다.

[준비 영수증](preparation_receipt.json)에 소스·명령·기대 헤더 SHA와 검사 범위를 기록했다. 월드 SHA는 `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`, 차량 YAML SHA는 `dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021`, expected header SHA는 `258630d5a24657d86c7a4a06cc22cc3174cdd3ab0801df78beb5ab9f09b0928d`다. 기하를 다시 생성하거나 전체 표본 감사를 반복하지 않았다. 기존 월드는 첫 직선 두 벽만 미실측 가정으로 변형한 것이며 전체 코스 벽 변형이나 실차 검증으로 확대하지 않는다.

[소스 선택 후보](source_selection.json)는 r549 `current_wall_reacquire.py` 반대벽 fallback, r543 `lidar_safety.py` 상태 옵션 연결과 신규 `guard_status_payload.py`를 포함한다. 상태 기록은 기본 `full`을 유지하며 compact 선택은 없다. 그 외 기존 실제 입력의 예상 밖 변경은 없었다. 예정 외부 필수 입력409개에는 이전 실제 snapshot의 참조 config/scene/centerline도 포함한다. 그 목록의 canonical JSON SHA는 `aa9f0541b245130a172a56bfb66a716a4c93d572ac24c2b7c116953204e9477b`다. 이는 **준비 후보 입력 해시**이며 아직 실제 실행 입력 해시는 없다.

기준 ZIP SHA `623c1a56451d4aae8e50315c6dae6b67dd5fdec07a1c6846f0ef62d638c99e96`와 원 보고서/무결성/PID 회수 근거의 보존을 확인했다. 새 ZIP은 만들지 않았다. 최종 동결 후에는 필수 입력 및 ZIP 불변을 검사하고, 다른 세션의 미사용 도구/전용 시험 변경은 관측 목록으로만 기록한다. 전체 저장소 불변을 주행 성공 조건으로 사용하지 않는다.

향후 승인된 실행 순서는 `prepare_once.sh`의 실제 import/경쟁 PID0/동결, 새 `root_execution_review.json` 작성, `execution_v1/execute_once.sh` 단1회, 등록 조건 감시 및 필요 STOP, `execution_v1/finish_collection.bash` 회수다. 정상 STOP·소프트웨어 조기 래치의 원 부가 실패·정상 종료·원 판정·실제 입력 SHA·RTF·전체 `/safety/status`를 구분한다. Jetson·ESP32·물리 센서에는 접근하지 않는다.
