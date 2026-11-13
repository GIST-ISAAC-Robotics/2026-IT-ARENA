# r457 최신 정책 actor0 단독 1회 — 23.35m 미완주, 정상 STOP·회수

2026-11-13, GPT. **실제 actor0 시험1회를 실행해 기존20.18m 정체 위치를 통과했으나23.35m에서 다시 정체했다.** 원8sim초 무진행 기준에 따라 sim47.273초의8.954sim초 무진행 관측 후 ownedSTOP1회를 호출했다.44.64m 한 랩은 완주하지 못했다. 재주행·제어 수정·판정선 변경은 하지 않았다.

## 조건과 원 판정

r452와 같은 최신 차체/센서/속도/시설에서 차량2개만 제거하고 승인된 r4595소스+1시험을 포함했다. old610시설3개는 그대로이며 실제 runtime world의 actor0 검사가 통과했다. r455는 미통합이다. r452의 교통2대 조건도 함께 달라졌으므로 이번 진행 증가 전체를 r459만의 인과 효과로 주장하지 않는다.

원 report는 `passed=false`, `harness_valid=true`, `run_valid=true`, `full_lap=false`, `no_permanent_stall=false`, runner exit1이다. owned helper STOP이 평가기 STOP보다 먼저 도착하여 `abort=mcu_latched_before_stop: software_stop`, `no_mcu_latch_before_stop=false`를 그대로 보존했다. 내부 STOP 요청은 sim48.329초다. 안정 정지·밀집 자세 정지·명시 MCU STOP·deadline진단 없음·벽 충돌 없음은 통과했다.

첫 실제 이동은 sim8.115초였다. 최종 누적 진행23.35m/목표44.64m, 원 평가기의 최장 .05m/s 미만 연속구간9.96sim초·합계10.968sim초다. 이 지표는 helper 판단의8.954sim초 무진행과 정의/종료시점이 다르다. 최초 지속0은sim38.119초의23.33m 표본이며 관성으로23.35m 부근까지 이동했다.

## 새 r459 이유의 실제 연결

`guard_curve_current_approach_observed_with_mask_budget`는 plan16건(ref32.22–34.52초)에 나타났다. 같은 generation/plan sequence와 current reference로 연결한 current16건 중13건은 `rebase_current_present_guard`,3건은 `guard_current_new_side` 거절이다. **통과13건은 부모 양수 발행까지 연결됐다.** 전체 publication/current reference 연결의 미대응·checked speed 불일치는0이다. 검사 수용을 적용으로 대체해 세지 않았다.

해당16 plan의 full-region 미관측0/frontmask0, masked tail은1.623–9.932mm였다. 원1cm 제한 안에서 통과한 기록이며 마스크를 지우거나 미관측을 채운 증거가 아니다. 각 plan/current/publication의 seq와 진단은 [r459 연결](analysis_v1/r459_mask_budget_connections.json), 부모 발행은 [reference 연결표](analysis_v1/publication_current_plan_join.json)에 남겼다.

20.18m 전후를 **진행19–21m(sim30.161–35.101초)**로 고정해 집계하면 plan49/current49/부모 양수42건이다. 이 중 새 이유는16/16/13건이다.20.18m 최초 초과 표본은sim32.485초다. 새 이유 마지막 입력 뒤 drive/safe/실제 속도가 모두 다음 수준을 넘었다.

| sim시각 | 진행 | drive/safe | 실제 속도 |
|---|---:|---:|---:|
|34.697s|20.75m|0.45m/s|0.367m/s|
|36.107s|21.57m|0.85m/s|0.770m/s|
|37.313s|22.55m|1.15m/s|1.098m/s|

## 새 정체의 최초0과 지속 거절 구분

최초0을 뒤의 current 거절로 소급하지 않는다. generation은 모두361이다.

| 단계 | 원 trace 위치 | 내용 |
|---|---|---|
| plan643, ref38.02s | 제출line1583/완료line1584 | `follow_boundary_unconfirmed` / `baseline_not_normal`, plan0, 기록된 current재검사0 |
| plan644, ref38.12s | 제출line1586/완료line1587 | `follow_release_wait`, plan0, 기록된 current재검사0 |
| plan645, ref38.22s | 완료line1590 | `local_follow` / `no_side_threat_seen`, plan+0.75m/s |
| current646, ref38.222s | call seq1590 / line1591 | `route_current_large_motion_masked_toward_right`, current0 |

이후 동일 right-mask 거절은100건(ref38.222–48.122초)이며 대응 plan100건은 모두 양수·`no_side_threat_seen`이다. 최초 지속0 이후 부모 양수 발행은0이다. actor0에서도 outer handoff는FOLLOW였고 selector는SOLO였다는 기록을 보존한다. 실제 다른 차량 때문이라는 설명은 사용할 수 없다.

첫 current646의 region distance는0.379875m, 미관측0/frontmask0, right-mask1284표본, tail19.174mm로1cm를 초과한다. 같은 입력의 기존 SOLO .15/.1 재검사도 모두 같은 이유로 거절됐다. 이 기록만으로 제한 완화의 타당성을 판단하거나 새 후보를 추가하지 않았다.

**후속 저장 분석 위치:** [최초 정체 anchor](analysis_v1/first_stall_anchor.json). 원 `run1/follow_decisions.jsonl` line1583의 `data.input`은 최초0 plan 입력, line1591의 `data.current_rebase.input`, `.compute.input`, `.compute.plan`은 지속 거절의 current/plan 입력이다. 초기0 두 plan의 제출/완료와 첫 right-mask plan/current 원 이벤트7개를 [선택 원 이벤트](analysis_v1/first_stall_raw_events.jsonl)에 별도로 보존했다. 전체 follow/plan pair 원본은 그대로다.

## 지연·관측·종료 근거

- 완전 plan 제출405/완료405, 고유키 각405, 미대응0·제출 wall시각 차이0. seq연속, footer accepted/written2027, dropped/failed/pending/capture_errors0, worker_alive=false.
- plan 계산 wall 중앙20.081/p95 31.583/max68.927ms; 고유 current396건 중앙16.793/p95 27.579/max78.889ms. 전체 wall565.877초. WSL 시뮬 통합 부하의 관측값이며 sim ms·Jetson10Hz 성능으로 바꾸어 해석하지 않는다.
- command_expired0, actuation 진단0. 벽 겹침·밀집 평면 벽 겹침·차체 road 이탈·3바퀴 이상 중심 이탈0. 제거된 actor 접촉센서는N/A이며 무발행을 물리 무접촉 증거로 삼지 않는다.
- 자식14/14 clean, process exit `[0,0]`, 원 shutdown clean. 별도 `/proc`에서 owner788와 `arena_autonomy_test_788` partition 잔여0.
- frozen1368입력 변경0, archive불변, wrapper 종료 뒤 check통과. prepared SHA `2737cba399450dcba44e09d1d6b960b11442e1b5d2acae05ab4464d4364d8376`, archive SHA `1e966fd9e27eae75dba8b11c56a5018a98c7971569daca39f8c248ec2da24b21`.

첫 WSL 서비스 진입 실패는 실제 주행 미시작으로 보존했다. 주 담당이 Ubuntu-24.04를 복구한 뒤 승인된 같은 wrapper로 실제 시험1회만 수행했다. 헤더 줄바꿈 초기 준비 실패·원 실패 report도 덮지 않았다. 이후 WSL/실행 중 공통소스 동결을 주 담당에게 반환했고 집계는 Windows에서 원 archive/로그만 사용했다.

근거: [집계](analysis_v1/summary.json), [원자료45파일 SHA](analysis_v1/original_evidence_sha256.json), [원 report](run1/report.json), [STOP 순서](stop_sequence.json), [입력 무결성](integrity.json), [소유 PID](execution_v1/owned_processes_after.json). 다음 판단은 주 담당의 저장 current 분석이며 r460변형·재주행은 보류한다.
