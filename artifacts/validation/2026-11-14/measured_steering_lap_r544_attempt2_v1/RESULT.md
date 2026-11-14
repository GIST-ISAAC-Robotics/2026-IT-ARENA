# r544 attempt2 실측 count 파생 조향 단독 결과

**19.56 m에서 정체해 전체랩은 미완주이며 원 판정은 FAIL이다.** 첫 코너를 회전한 뒤 약0.8 m/s로 복귀했으나 이후 19.56 m 지점에서 `/drive`와 `/drive/safe`가 모두0이 됐다. 등록된 8 sim초 무진행 기준에 따라 STOP했고 실제 안정 정지·14개 자식 정상 종료·잔여 소유 PID0을 확인했다. 같은 조건의 추가 실행은 하지 않았다.

[원 보고서](run1/report.json)는 `passed=false`, `run_valid=true`, `harness_valid=true`다. 총 코스44.64 m 중19.56 m 진행, 최장정지9.256 sim초, GREEN 이후 기록40.716 sim초다. `full_lap`과 `no_permanent_stall`은 false이며 신호·자료 범위·바퀴 노면 기준은 true다. 조밀 pose20,612표본에서 벽 외곽 겹침0·도로 밖 바퀴 중심0을 기록했다. 이 표본 기반 외곽 검사는 연속 실물 접촉 검증이 아니다.

차량 설정 SHA는 `dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021`이다. [실제 launch 조향 영수증](steering_runtime_receipt.json)의 7개 검사는 모두 통과했다. `g1_g2_handoff_trial`/독립 guard 중앙 명령 상한은 .28475436568260193 rad, virtual MCU/actuator 물리 중앙 상한은 .28475438456006436 rad, SDF 내륜 joint/plugin 상한은 .30666234131856146 rad다. 사용자 count 범위를 기존 CAD 변환으로 반영했으며 실제 바퀴각 실측값은 아니다. [기록 집계](run_supplement.json)의 실제 경로 발행612개 중219개가 상한의1e-6 rad 이내였고 최대 절댓값은 명령 상한과 같았다. 이 결과만으로 정체의 단독 원인이 조향 상한이라고 단정하지 않는다.

월드·시설·속도·물성·원 판정은 첫 시도와 동일하다. actor0·grid3·시설3개·80 sim초 상한·RTF 설정.25·g1_open_1p5·medium_v1을 유지했다. 월드 SHA `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`이며 첫 직선 두 벽만 굴곡/틈의 미실측 가정으로 바뀐 월드다. 전 코스 모든 벽의 굴곡이나 실차 성능으로 확대하지 않는다.

실제 RTF는 GREEN 구간 `/safety/status`의 simulation reference 차이와 evaluator monotonic 수신 시각 차이로 계산해 약 **0.08588**이었다(해당 wall 구간473.697초). 이는 메시지 전달 시차를 포함한 기록 기반 비율이다. 원 보고서의 전체 wall 시간은578.673초이며 최고 지면 속도는1.5383 m/s였다. 설정 RTF.25와 실제 진행 속도를 구분한다.

[정체 발췌](stall_extract/transition.json)는 저장 기록만 추출했고 제어기 재생은 하지 않았다. 마지막 양수 call seq2215는 현재 벽 재획득0.1 m/s·left경로36점이며 실제 마지막 경로 발행은 t38.52, 조향−.16959877 rad다. 다음 seq2218은 `decision_previous_request_changed`로 전달 폐기됐고, 그 다음 seq2222는 정상 candidate로 받아들인0 계획이다. 두0 계획의 `normal.reason`은 `parallel_hold_baseline_unavailable`, `baseline_reason`은 `follow_boundary_unconfirmed`다. 각 경우의 current/compute 입력·call·원 plan·근접 guard JSON을 같은 폴더에 보존해 전달 폐기와 제어 계획을 분리했다.

[STOP 원문](early_stop.json)에서 `/autonomy/enable=false`, `/actuation/stop` 모두 성공했고 MCU `ESTOP_LATCHED`·출력0을 확인했다. 독립 감시가 t46.921에서 무진행8.25 sim초를 확인한 뒤 요청했다. 평가기 자체 STOP보다 외부 조기 STOP이 먼저였으므로 원 `abort=mcu_latched_before_stop: software_stop`, `no_mcu_latch_before_stop=false`를 그대로 둔다. [STOP 순서 기록](stop_sequence.json)에 실제 안정 정지와 이 원 부가 실패를 구분했다. `shutdown_clean=true`,14/14 자식 정상 종료·강제 정리 없음이며 [독립 PID 감사](execution_v1/owned_processes_after.json)의 잔여 소유 작업은0이다. WSL은 회수 즉시 root에 반환했다.

[무결성](integrity.json)은 필수442개·동결 ZIP 불변이다. 관측983개 중 다른 세션의 `scripts/probe_esp32_motor.py`, `tests/test_motor_probe.py` 변경은 별도 목록으로 보존했고 실행 실패와 섞지 않았다. 실제 runner snapshot454개도 원 `source_unchanged=true`·snapshot 누락/불일치0이다. 실제 snapshot에는 필수 동결 목록 바깥의 기본 참조 JSON/scene/centerline20개도 있으며, 이들은 runner의 실행 전후 검사와 snapshot으로 보존했다. 그 목록은 [집계](run_supplement.json)에 명시했다.

[전체 수집](evidence_collection.json)은 필수 근거 누락0·수집 오류0, trace 제출/완료404/404·입출력 불일치0·footer 정상 종료를 확인했다. 헤더의 조향 상한 한 키는 새값으로 등록했으며 나머지는 기존 정규화 규칙에서 동등하다. 실행 PID35883, [로그](execution_v1/runner.log), 원 실행 exit1·[수집 종료](execution_v1/collection_exit.json) process0/integrity0/collection0을 보존했다.

[첫 시도](../measured_steering_lap_r544_v1/RESULT.md)는 실행기 이름 감사 오류로 자식 시작0·실제 주행0인 하네스 실패다. root의 명시 승인 후 [이번 준비](attempt_preparation.json)에서 이름 검사 한 곳과 실행 경로만 고쳐 별도 시도했다. 첫 실패/ZIP/판정과 이번 실제 정체를 덮어쓰지 않았다. 추가 trial은 자동으로 시작하지 않는다.
