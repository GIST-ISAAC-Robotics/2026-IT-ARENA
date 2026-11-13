# r428 제한 주행 — 사용자 요청 일시 중단

기록: 2026-11-13 19:16 KST. 준비 당시 README와 동결 파일은 그대로 보존한다.

- 빌드·입력 동결·사전 검사16개를 마친 뒤 등록한 주행1회를 실행했다. 실행 담당의 마지막 중단 전 확인은20.763sim초·진행10.04m이다.
- **중단 원인은 사용자의 일시 중단 요청이다.** 8sim초 정체 기준에 도달해 중단한 것으로 취급하지 않는다. `early_stop.json.reason`은 기존 helper의 고정 문구이며 이번 실제 사유를 나타내지 않는다. 원 JSON을 수정하지 않는다.
- 소유 helper가 `/autonomy/enable false`와 `/actuation/stop`을 각각 한 번 호출해 성공했다. 출력0·`software_stop`을 확인했고, 원 평가기의22.845sim초 STOP 이후 차체 안정화·명시 MCU STOP 검사도 통과했다.
- 원 `report.json`의 `passed=false`, `harness_valid=false`, `abort=mcu_latched_before_stop: software_stop`, `no_mcu_latch_before_stop=false`는 보존한다. 이번 helper STOP이 원 평가기 STOP보다 앞서 있으므로 이 래치를 원래 주행 정체의 원인이라고 해석하지 않는다.
- 평가기 상위 자식5개 exit0, 강제 reader 회수없음, `remaining_test_pids=[]`, Gazebo server 종료 확인·잔존없음이다. 입력1,344개·준비 manifest·archive는 불변이다.

## 재개 지점

r426의 저장 입력 개선은 완료됐지만 이번 사용자 중단 주행으로 기존10.10m 정체 해소나 전체 랩을 판정하지 않는다. 재개 후 이 run1의 현재 경로 후보 수락·부모 발행·실제 진행을 먼저 대조한다. 원 실행을 덮거나 자동 재실행하지 않고, 결과에 따라 필요한 별도 제한 주행 또는 다음 교통 대표를 결정한다. 기본off/실물off와 기존 평가 기준을 유지한다. 현재 실행 중인 자기 시험은 없다.

## 원시 근거

- [평가 원문](run1/report.json), [소유 STOP](early_stop.json), [STOP 순서](stop_sequence.json)
- [입력 무결성](integrity.json), [현재 처리 감사](runtime_audit.json), [동결 입력](prepared.json)
- [준비 절차](README.md), [r426 저장 비교](../crop_current_reach_r426_v1/RESULT.md)

원시 상세 로그와 archive의 기존 Git 제외 정책은 유지한다. 새 실차·Jetson 시험, 커밋·푸시는 수행하지 않았다.
