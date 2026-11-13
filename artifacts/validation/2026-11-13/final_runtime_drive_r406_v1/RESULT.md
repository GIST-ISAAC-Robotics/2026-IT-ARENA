# r406 제한 주행 결과

최종128mm 축거·123.9mm 윤거·CAD조향·후방마커90mm/4가림과 공통 runtime을 사용한1회는 **완주하지 못했다.** 정체 감시 당시 진행10.11m에서8sim초 무진행으로 소유 STOP을 요청했다. 회수 trace 전체 최대 진행은10.13m다. 이전 r377의140mm/5가림과 구성을 구분한다.

- 원 `run1/report.json`: `passed=false`, `harness_valid=false`; 전체거리·정체·기간/자료범위 실패를 유지한다. `mcu_latched_before_stop: software_stop`은 외부 소유 STOP과 평가기 STOP 기록의 순서가 포함된 종료 사유이며, 그 문구만으로 주행 중 MCU가 먼저 멈췄다고 단정하지 않는다.
- 종료 STOP은 정착과 액추에이터 정지를 확인했다. 벽 접촉 표본0, actor 접촉/평가 겹침0, 두 바퀴 노면 기준 통과다. 정상 자식 종료·잔여PID0·강제회수0, 입력1324개와 준비 archive 불변을 확인했다.
- 원 runtime 감사는 header 불일치였다. [r409 후속 저장 감사](../runtime_config_audit_r409_v1/README.md)에서 차이는 자기구조 JSON의 공백뿐이며 의미가 같음을 확인했다. 원문/원판정은 보존한다.
- 새 거리 후보의 계산·부모 발행은 각1회다. 정체 후 현재 재검사가0.1m/s를 허용하는 표본도 있으므로 과거의 현재검사 거절만으로 이번 실패를 설명할 수 없다. 평가 trace25~34초89표본은 guard `unbounded_missing`77/`clear`12이며 대부분 최종 명령0이다. 간헐0.1m/s 발행·움직임도 있다. 표본별 시간차와 부모/독립 보호층의 적용을 분리해 분석한다.

이번 결과를 후보 효과나 전체 주행 성공으로 바꾸지 않는다. 동일 조건을 재실행하지 않고 저장 원인을 분리하며, B3 운영 연결·수동 개발 조작 준비는 병렬로 진행한다. Fable5.2/high 읽기 전용 자문은 세션 사용량 한도로 답변/도구 실행 없이 실패했다(도구 표시 초기화11/13 16:00 KST). 새 의견을 얻지 못했으므로 GPT가 분석을 계속한다. 같은 호출을 반복하거나 다른 Claude 모델로 대체하지 않았다.

근거: `run1/report.json`, `run1/trace.json`, `run1/follow_decisions.jsonl`, `runtime_audit.json`, `integrity.json`, `prepared.json` 및 [후속 감사](../runtime_config_audit_r409_v1/report.json). 대형/원시 자료의 Git 제외 정책은 유지한다.
