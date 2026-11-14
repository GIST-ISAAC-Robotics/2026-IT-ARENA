# r560 OFF 교통 대표 — 사용자 요청에 따른 중단·회수

2026-11-14. 사용자 “일단 지금 개선 작업은 여기까지해” 지시에 따라 진행 중인 r560에 STOP을 요청하고 회수했다. **등록12 sim초를 완료하지 않은 부분 결과**이며 원 FAIL을 보존했다. r558 단독 및 r559 trace native 후속 검사는 시작하지 않았고, 후속 GO는 취소되었다. 재개에는 새 사용자 지시가 필요하다.

## 종료 상태

- PID49045, 기존 disable/STOP 모두 수락, ESTOP_LATCHED/출력0 확인. 원 STOP의 model stable/dense pose/explicit MCU/deadline/wall5개 true다.
- 외부 STOP으로 원 `mcu_latched_before_stop: software_stop`, `no_mcu_latch_before_stop=false`가 남았다. `passed=false`, `run_valid=false`, `harness_valid=false`, data coverage=false도 그대로 보존했다.
- **14/14 child 정상 종료**, 강제 종료·미회수·잔존 owned PID0, `shutdown_clean=true`. WSL은 주 담당에게 반환했다.
- 필수 입력467개·ZIP 불변, source map432개와 실제 snapshot의 누락·SHA불일치0. process/integrity/summary/assessment/collection 종료코드 모두0이다.

[사용자 STOP 요청 영수증](user_stop_receipt.json), [종료 영수증](user_stop_final_receipt.json), [실제 STOP 응답](early_stop.json), [독립 PID 확인](execution_v1/owned_processes_after.json), [수집 종료코드](execution_v1/collection_exit.json)을 남겼다. 기존 STOP helper의 고정 reason은 정체/위험 일반 문구이지만 **이번 실제 원인은 사용자 중단**이다. 원 helper 파일/결과를 바꾸지 않고 이 차이를 별도 영수증에 명시했다.

## 부분 관측

GREEN7.401→종료창17.479, **10.078/12 sim초**다. 원 evaluator 진행 **4.69m**, 최소 표본 간격0.2964997m, 최종 앞차 간격0.6412021m였다. 원 longest stationary while lead moves는0초, sampled overlap0·swept nonpositive0·actor contact message0·도로/벽 기준 통과를 부분 구간에서 관측했다. 전체12초 통과·추월 성공·실차 무접촉으로 확대하지 않는다. 별도 보조 평가 진행4.6672m는 다른 투영 집계이며 원4.69m와 구분한다.

원 평가의 data coverage가 부족하므로 주행 유효성은 false다. 최초 움직임8.399sim초, 원 wall time227.338초였다. actual controller/guard/MCU/SDF7조향 상한 검사는 통과했으며, 기록된 candidate/발행/원 raw는 [evidence_collection.json](evidence_collection.json)에 보존했다. 원시 자료의 추가 원인 재생·actor 귀속 분석은 수행하지 않았다.

## 시작 전 확인과 기술 수정

r555 시작 실패 원본115개 파일을 보존했다. r559 마커 wrapper는 빈 OFF 파라미터만 생략하며 실제 native OFF/ON parser2case가4.217초에 통과·잔존0이었다. 이는 ON **주행**이 아니다. 원 r555 오류는 이번 실제 초기화에서 발생하지 않았다.

r553 기준4제어 변경(r554 bypass1 + r557 helper/current/retained3)을 정확한 SHA로 선택했고 실제 import12개·native 바이너리 불변·경쟁0을 확인했다. 기존 symlink build를 재사용했다. root의 r557 native215/stage8 검사 근거를 반복 실행하지 않았다.

새 사본의 assess는 선택한 실측 vehicle config와 원 report의 경로·SHA를 대조한다. 고정 final_cad YAML 선택 오류만 제거했고 임계값·판정선을 유지했다. 미출발 summary/assessment는 unavailable로 기록하도록 보완했고, r555 저장 사본 분기 확인을 통과했다. Windows Shapely 미설치 import 오류는 기록 후 기존 WSL 환경에서 확인했으며 설치0·공통 소스 수정0이다.

- [원 report](run1/report.json) SHA `755b71975cc16c17cef1bf902bb46b2eff96618b6252a87b1b1b6d76c1910541`.
- [prepared](prepared.json) SHA `3d8b7c9b16202714f3c6d7c045a68534fb1bf2f79f18f8f44ecf52e73ad037b5`.
- [sources.zip](sources.zip) SHA `0dd6be7d936109ed2d7d3cb2c71dc3eacc8c503ebdd9c58b86695032e969b9ad`.
- [실제 입력](actual_execution_inputs.json), [최종 위임 대조](root_execution_review.json), [native2case](native_smoke_v1/result.json), [저장 미출발 분기 검사](preparation_checks/result.json).

## 재개점

현재 활성 시험·빌드·WSL 실행은 없다. r558와 trace native4case는 **미실행·승인 취소** 상태다. 새 사용자 재개 후 당시 공통 소스/자원 소유권과 새 실행 승인을 다시 확인한다. 기존 r553/r555/r560 원 기록은 유지하며 현재 결과를 덮어쓰거나 자동 재실행하지 않는다. Jetson/ESP32/실물 접근0이다.
