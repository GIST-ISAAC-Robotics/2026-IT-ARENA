# r544 시작 전 계측 오류

**launch 시도 1회가 하네스 오류로 끝났으며 실제 주행은 시작하지 않았다.** 새로 추가한 [조향 전달 감사](steering_runtime_audit.py)가 controller executable을 `local_pursuit`로 잘못 고정했다. 이 레시피의 기존 실제 실행기는 `g1_g2_handoff_trial`이므로 launch action을 생성한 뒤 이름 검사에서 예외가 발생했다. 제어기나 새 조향 상한에 의한 코너 실패로 해석하지 않는다. 같은 조건의 자동 재실행은 하지 않았다.

[원 조향 영수증](steering_runtime_receipt.json)은 `passed=false`를 유지한다. 기록된 실제 `g1_g2_handoff_trial`/`lidar_safety` 상한은 .28475436568260193 rad, virtual MCU/actuator 상한은 .28475438456006436 rad, 양쪽 joint 및 plugin 상한은 .30666234131856146 rad로 기대값과 일치했다. 이름 검사 하나가 잘못된 것이며 원 실패 영수증을 성공으로 덮어쓰지 않는다. 실제 ROS 노드가 시작된 후의 파라미터 수신·물리 주행은 확인하지 못했다.

[원 보고서](run1/report.json)의 `passed`, `run_valid`, `harness_valid`, `shutdown_clean`은 모두 false이며 오류는 `RuntimeError: owned process exited early`다. launch가 시작한 자식 프로세스는 0개이고 GREEN·주행·안정 STOP 근거는 없다. [수집 결과](evidence_collection.json)의 누락 `follow_decisions.jsonl`, `stop_trace.json`도 이 시작 실패와 함께 보존한다. 정상 STOP/정상 종료를 확인했다고 주장하지 않는다.

실행 PID는 35535였으며 [실행 로그](execution_v1/runner.log)와 [launch 로그](run1/simulation.log)를 보존했다. [독립 회수 감사](execution_v1/owned_processes_after.json)의 잔여 소유 PID는 0이다. [무결성](integrity.json)은 실행 필수 440개·ZIP 불변이며, 관측 목록 983개에도 동결 이후 변경은 없었다. [수집 종료 코드](execution_v1/collection_exit.json)는 process0/integrity0/collection0이고 원 주행 종료 코드는 1이다. WSL 소유권은 회수 즉시 root에 반환했다.

동결 전 참고 목록의 motor probe 관련 변화로 중단된 두 사전검사 로그도 [첫 로그](preflight/import.log)와 [둘째 로그](preflight/import_final.log)에 보존한다. 이후 필수/관측 범위를 분리하여 [최종 사전검사](preflight/import_final2.log)를 통과한 뒤 이번 최초 launch를 시도했다. 제어 소스·설정·월드와 관련 없는 파일 변화는 이후 실행 중단 조건으로 사용하지 않는다.

월드·actor0·grid3·시설3개·80 sim초 상한·8 sim초 무진행 STOP·RTF .25·속도/물성/원 판정은 [준비 영수증](final_recipe_manifest.json)대로다. 선택 조향은 사용자 count 범위에서 CAD로 유도한 값이며 바퀴각 실측은 아니다. 이번 결과로 단독 완주나 코너 통과 여부는 판단할 수 없다. 새 시도를 하려면 이 실패를 보존한 별도 실행 단위에서 계측 이름을 고치고 최종 입력을 다시 동결해야 한다.
