# r421 r418 정체의 저장 단계·시각 대조

**지속 정체의 주된 차단 단계는 current 재검사다.** 원 평가 trace의 마지막10sim초 **29.549–39.549초**에 해당하는 고유 계획99개는 모두 약0.5 m/s·`local_follow`·`bounded_side_missing`·crop 진단을 냈다. 연결된 current99개는 모두 `route_current_large_motion_region_unobserved`로0을 반환했고, 인접 부모 발행99기록도0이었다. 계획의 crop 허가가 current 승인·실제 구동까지 이어졌다는 근거는 없다.

이는 저장 필드의 대조이며 정책/상태 재생, 재시뮬레이션, 제품 수정은 하지 않았다. 원 [runtime 감사](../crop_edge_drive_r418_v1/runtime_audit.json)의 crop148/양수146·거리 후보 부모발행2는 그대로 맞지만, 그 숫자들은 지속 진행을 뜻하지 않는다. r420 코너 장면은 이 실패 상태에서 실행하지 않는다.

## 계보와 시간 경계

- `follow_decisions.jsonl`의 raw `call`319행은 `(generation,sequence)` 기준 **고유 계획311개+동일 input/output 중복8행**이다. 고유 양수 계획305개이며 crop148/그중 양수146은 고유 계획 기준으로도 같다.
- current는313개 모두 고유하다. 각 current의 중첩 `compute.plan` ID/input/output은 해당 outer plan과 일치하고 current compute input도 해당 current input과 일치한다. 충돌0이다. 전체 current 양수127·0반환186(large-motion185/guard-scope1)이며 부모 발행 기록313개의 양수도127이다.
- 발행에는 compute/command ID가 없다. 이번313연결은 **바로 앞 call event + current `reference_s` 정확일치**로만 대응했다. 이를 독립 guard·MCU까지 동일 ID로 추적한 것으로 확대하지 않는다.
- `trace.json`은 별도 약0.1초 표본이며 autonomy 상태 시각도 따로 있다. 아래 `trace[index]`는 **0부터 시작하는 배열 위치**, `seq`는 follow event의 원 번호다. 표본 사이 순간 동작이나 모든 폐기 계산의 이력을 복원하지 않는다.

## 최초 정체 경계: 계획 자체0과 이후 current0을 분리

| 저장 근거 | 계획 reference/목표 | current 및 부모 발행 | 의미 |
|---|---|---|---|
| call706 → publication707 | 24.320초/약0.5 | current24.360초·`current_solo_gap_retry`·0.15 → 24.440초 발행0.15 | `partial=fully_observed`인 직전 저속 재검사 성공 |
| call708 → publication709 | 24.420초/약0.5 | current24.460초·같은 retry·0.15 → 24.540초 발행0.15 | 진행을 유지한 마지막 대표 |
| call710 | 24.620초/0 | current/route-publication 항목 없음 | 계획 `follow_partial_transformed_gap_uncovered`, partial 자체는 fully_observed |
| call711 | 24.720초/0 | current/route-publication 항목 없음 | crop·bounded_side_missing 뒤에도 계획 `follow_partial_transformed_gap_uncovered` |
| call712 | 24.820초/0 | current/route-publication 항목 없음 | 계획 `follow_release_wait` |
| call713 → publication714 | 24.920초/0.5 | current24.922초·large-motion 거절0 → 24.924초 `not_published_as_checked`·0 | 계획이 다시 양수가 된 뒤에도 current에서 지속 차단 |

`trace[239]` 24.625초에는 `/drive=0`, `/drive/safe≈0.15`가 함께 보인다. 같은 주기의 쌍으로 해석하면 안 되는 실제 예다. `trace[240]` 24.725초는 처음 최대 진행10.10 m에 도달한 표본이며 ground speed는 아직0.0872 m/s다. 다음 `trace[241]` 24.827초는0.00540 m/s, 두 command는0이다. 양자화된 진행의 첫 최대 도달을 실제 정지 시각으로 부르지 않는다.

call713의 source-first는24.802초, current path age는0.120초, 질의 거리0.2225 m·unobserved920·masked-left1156·tail9.513 mm이고 `accepted=false`다. `trace[242]` 24.929초의 autonomy `sim_time_s=24.924`는 publication714 시각과 일치하며 `ROUTE_CURRENT_LARGE_MOTION_REGION_UNOBSERVED`, drive/safe0, guard `clear`, MCU `ARMED`를 기록한다. 이는 주된 상류 거절에 부합한다.

## 마지막10초와 원 scan 동일성

99개 계획/reference 범위는29.620–39.420초다. current는 계획보다2–4 ms 뒤의 다른 reference/조향/이력을 가진 별도 검사다. 계획은 전부 약0.5 m/s·bounded_side_missing, current 조회 거리는0.2205–0.2235 m·unobserved781–936이며 전부 거절했다. 이 구간에 `current_solo_gap_retry`, `current_relative_body_retry`, `current_solo_low_retry`, `current_corner_distance_budget` 진단은 없다. **진단 부재만으로 미진입과 내부 거절을 구분하지는 않는다.** 조건 분기의 원인은 별도 코드 검토 사항이다.

[scan_identity.json](scan_identity.json)은 outer 계획 input, current에 묶인 submitted plan input, current input의 **원 scan**을 canonical JSON SHA-256으로 대조한다. 배열 순서·float·비유한값 태그·metadata를 바꾸지 않았다.

- 전체 current313개 중 전체scan SHA 동일305개. 나머지8개는 ranges/angles/acquisition이 다르며, 이를 강제로 같은 입력으로 취급하지 않는다. frame/epoch/time_basis/range bounds는313개 모두 같다.
- 마지막10초는 **99/99 전체scan SHA 동일**이다. `ranges_m`, `angles_rad`, `acquired_at_s`, `source_frame_id`, `source_epoch`, `time_basis`, 두 range bounds가 모두 일치한다. 전체 history 동등성을 요구하거나 주장하지 않았다.
- 대표 call713: plan `(361,359)`→current `(361,360)`, scan512개, SHA `dab2a4a401f08be2b43f1a797fc12085bb97a919012a3a1172e3310e2bc165b7`.
- 마지막 call1003: plan `(361,649)` 39.420초/0.5→current `(361,650)` 39.422초/0, scan504개, SHA `9ad5a62a8dbf9683db8a82bb11fa3e8a5d4729676b75b7efe7152aae12bea1e3`. 거리0.2215 m·unobserved794·tail9.601 mm, publication1004는39.424초/0이다.

같은 scan이라는 사실은 과거 partial 허가를 현재 경로·조향·이동 영역의 proof로 재사용해도 된다는 뜻이 아니다. 이번 자료는 새 설계가 동일 원시 관측에서 어떤 조건 때문에 저속 재검사로 이어지지 않는지 분리하는 근거만 제공한다.

## 거리 후보2회·guard·MCU의 기록 범위

call680/689는 각각23.220/23.620초의 계획약0.45를 current0.0933267/0.0937574 m/s로 허가했다. publication681/690은23.227/23.627초에 같은 속도를 발행했다. 뒤의 별도 `trace[226]`23.311초와 `[230]`23.715초에는 해당 float32 속도가 drive/safe 양쪽에 보이고 guard `clear`, MCU `ACTIVE`, 작은 실제 ground speed0.0214/0.0183 m/s가 기록된다. autonomy 시각도 각각 발행 시각과 일치한다. 두 후보가 부모 계산에만 머물렀다고 볼 근거는 없지만, 이 표본의 근접 대응을 MCU 명령 ID 일치로 부르지는 않는다.

후반 대표의 drive/safe0·guard clear·MCU ARMED는 **양수 명령이 guard에서 일괄 차단된 상황보다 current/부모에서 이미0이 된 상황**을 지지한다. 다만 이 `trace.json`의 MCU 필드는 `mcu_state`뿐이다. 주행 중 실제 `target_speed_mps`/accepted sequence/제어출력은 직접 저장되지 않아 이 자료만으로 각 MCU 목표를 정확히 매칭할 수 없다. [early_stop.json](../crop_edge_drive_r418_v1/early_stop.json)의 target0·ESTOP_LATCHED·software_stop은 소유 STOP 뒤의 최종 확인이며, 앞선 정체 원인으로 소급하지 않는다.

## 소유 helper STOP 뒤 원 평가기 STOP

[stop_order.json](stop_order.json)에 원 [report](../crop_edge_drive_r418_v1/run1/report.json)·[stop_trace](../crop_edge_drive_r418_v1/run1/stop_trace.json)와 helper 결과의 필요한 필드를 따로 묶었다.

1. 소유 helper는 `/autonomy/enable` 사용자 정지, `/actuation/stop` software_stop 순서의 성공 응답을 기록했다. 정확한 helper 요청 sim 시각은 저장되지 않았다. 원 평가기의 첫 래치 표본은 MCU `now_us=39,570,000`, software_stop이다. 이는 **첫 래치 관측 시각**이지 helper의 정확한 요청 시각이 아니다.
2. helper 최종 확인은 같은 boot의 **generation2**, `now_us=39,834,000`, target0·ESTOP_LATCHED다. output도39.836초·0·blocked이며 `stop_confirmed=true`다.
3. 그 뒤 원 평가기는 `stop_request_sim_s=39.965`, 새 `/clock=39.967초`를 기록하고 자신의 STOP을 수행했다. 최종 MCU는 같은 boot의 **generation3**,40.464초·target0·ESTOP_LATCHED다. 정지창39.965–40.465초의 조밀251표본/250구간은 최대속도0.000751 m/s, 변위0.000230 m로 원 정착 검사를 통과했다. 14자식 정상 회수·잔여PID0도 원 report에 있다.

따라서 원 `abort=mcu_latched_before_stop: software_stop`과 `no_mcu_latch_before_stop=false`는 **평가기 자신의 STOP보다 먼저 들어온 소유 helper STOP**을 포함한 결과다. 원 실패를 지우지 않되, 이 문구를24.7초부터 시작된 주행 정체의 MCU 선행 고장으로 해석하지 않는다. `/clock` 콜백은 노드별 비동기이고 원 timing limits도 인과의 정밀 시각 증명을 부정하므로, microsecond 차이만으로 서비스 요청의 세밀한 순서를 더 추정하지 않는다.

## 산출물·재검토 경계

- [summary.json](summary.json): 고유 ID/대표 시각/4개 대표 call/종료 경계와 원 입력 SHA.
- [selected_timeline.json](selected_timeline.json): 첫 최대진행 전후1초·마지막10초·거리 후보2회의 필요한 필드만 남긴 제한 목록. 원 광선 배열/전체 경로 점군은 복제하지 않았다.
- [scan_identity.json](scan_identity.json): 전체/후반 원scan 동일 비율과5개 대표 SHA.
- [stop_order.json](stop_order.json): helper 세대2/원 평가기 세대3 STOP·정착·회수의 별도 시간 경계.
- [analyze_saved.py](analyze_saved.py), [scan_identity.py](scan_identity.py), [analyze_stop_order.py](analyze_stop_order.py): Windows Python stdlib 저장 읽기만 수행. 원 파일은 불변 확인했다. 출력은 새 파일로만 생성하며 현재 결과를 덮어쓰지 않는다.

새 위험/수치 완화/정책안은 이 묶음에서 제안하지 않는다. 다음 판단의 직접 근거는 **crop 뒤 계획 양수로 복구됐지만 current 저속 proof로 이어지지 않은 동일scan 대표**이며, 원 첫 transformed-gap 정지와 종료 STOP은 별도 원인으로 유지한다.
