# r412: 진행 거절과 관측 기억 갱신의 결합

진단·설계만 완료했습니다. r410은 단독 미채택이며 원 결과와 동결 파일을 보존했습니다. 새 정책 구현·정책 재생·비용 시험·WSL/ROS/장치 실행은 하지 않았습니다. `analyze_accounting.py`로 저장 JSON·시각·상태를 stdlib 분석했고 입력 SHA의 전후 일치를 확인했습니다.

## 첫 만료의 실제 계보

아래는 **r410의 동일 저장 순서 제한 재생**에서 확인한 사실입니다. r406 원 자식의 누락 계산을 복원한 결과가 아닙니다.

| trace / plan | reference | source first | 유효 반환 | partial 결과 | 기억의 reference |
| --- | ---: | ---: | ---: | --- | ---: |
| 684 / 332 | 23.82 | 23.703 | 471 | fully_observed | **23.82로 갱신**, 3m 이내450점 |
| 685 / 333 | 23.92 | 23.803 | 471 | unbounded_missing | 23.82 유지 |
| 686 / 334 | 24.02 | 23.903 | 471 | unbounded_missing | 23.82 유지 |
| 687 / 335 | 24.12 | 24.003 | 474 | unbounded_missing | 23.82 유지 |
| 688 / 336 | 24.22 | 24.103 | 476 | partial_history_expired | 23.82 유지, **0.400s** |

세 중간 실패의 원 gap은 각각7/7/6 rays이고 crop 밖 unknown은6/6/5개입니다. 즉 r410이 의도적으로 계속 거절한 실제 바깥 결측입니다. 마지막688만 전체 gap이 crop 안1 ray라 finite 이웃 복구 후 history 검사까지 도달했습니다.

이 구간 원 scan 취득 시각은 매번0.1s 증가하고 source age는 약0.117s로 기존0.3s 조건 안입니다. wheel/gyro 이력 최대 간격은 각각0.010/0.005s, reference까지 외삽은0.006/0.005s로 저장 MotionConfig의0.05/0.03s 조건 안입니다. epoch0·laser_frame·simulation_acquisition·timing_verified=true를 유지하고 측정속도는0입니다. 이 자료는 **입력 시각과 이력 연속성**을 지지하며, 각 반환의 물체 신원·실물 센서 신뢰도나 숨은 장애물 부재까지 증명하지 않습니다.

`partial_observation.py:238–242`의 unbounded 반환은 소실 위협 검사와 `_remember` 모두보다 앞섭니다. 따라서 `previous`뿐 아니라 `last_source=23.703`도 고정됐습니다. 688의 `now-previous[1]>.3`은 마지막 수신 후 경과가 아니라 **마지막 기억 갱신의 reference 후 경과**를 검사하여 만료 래치를 만들었습니다. 이전 `previous_distinct`도22.92/source22.803에 남아 있습니다.

후보 래치는688의24.22s에서 시작해780의33.42s에 fully_observed 확인2회로 풀렸습니다. 그 동안 reason은 `partial_history_expired`로 고정됩니다. baseline은 같은 제한 재생 전체에서 래치가 없었습니다. ≥25s의 양수2→1, 후보24 expiry/11 known_obstacle_latched는 이 상태 전파와 함께 해석해야 합니다. 이 창에는 timed_missing_diagnostic도 latch_evidence도 없으므로 **소실 위협이 없었다고 결론내릴 증거는 없습니다.**

## 최소 보완 설계 한 가지

**유효 partial 입력의 기존 소실 검사·기억 반영을 진행 허가의 geometry 거절과 분리**하는 방향을 권고합니다. 새 TTL/기억 저장소/속도 규칙을 만들지 않고 기존 위협 검사와 `_remember` 순서를 한 함수로 추출해 재사용합니다. 첫 구현 범위는 이 창의 `unbounded_missing` 분기만으로 제한하는 편이 작습니다. 모든 early return를 일괄적으로 뒤로 옮기는 것은 권고하지 않습니다.

1. 스캔·source age·clock 역행/reset·epoch·motion·point-order 무결성 실패는 현재처럼 먼저 반환합니다. 실제 새 스캔이라도 previous가 이미0.3s를 넘었으면 기존 expiry/latch를 유지하며 현재 수신 사실로 그 나이를 덮지 않습니다. 같은 source 재호출은 `previous_distinct`/fresh 의미를 그대로 사용하고 확인 횟수를 늘리지 않습니다.
2. geometry의 허가 결과는 별도로 보존합니다. `unbounded_missing`은 끝까지 `allowed=false`와 원 reason을 반환합니다. **진행 허가용 ±100°/폭/거리/속도 판정은 바꾸지 않습니다.**
3. 이 거절 프레임에서 기억을 바꾸려면, 기존 transport→near/timed missing sweep→실제 재관측/표면 대응→미해결 위협 latch·전체 capture를 먼저 끝내야 합니다. crop 안1 ray만 검사하면 바깥100–104°의 같은 gap에 가려진 근거리점을 잃을 수 있습니다. 따라서 accounting 입력에는 해당 **원 연속 gap 전체**와 원 인덱스·취득시각을 사용해야 합니다. 이것은 소실 증거 검사 범위이며 바깥 unknown의 진행 허가나 free 변환이 아닙니다. 다른 전방 결측도 빠뜨리지 않고, known-structure의 기존 별도 기억 위임은 보존합니다.
4. 검사 완료 후에만 기존 `_remember`로 현재 유효점/원천 시각을 갱신합니다. 위협이 있으면 `previous_point_in_missing_sweep`와 `_capture_latched_threat`가 **교체 전** 옛 점·전체 배경·source index·시각을 기존 anchor/clearance에 고정해야 합니다. capture/transport/시각 metadata가 불완전하면 갱신하지 않습니다. 이미 활성인 latch·anchor를 새 현재점으로 초기화하지 않습니다.

이 방향은 현재 유효점 배열과 지속해야 할 소실 증거의 역할을 기존 구조 안에서 분리합니다. 무조건 `_remember`를 삽입하는 방법은 첫 가림 프레임에서 옛 근거리점을 없애고, 다음 프레임에는 비교할 점 자체가 없게 만듭니다. 그 방식은 제외합니다. 또한 별도의 `last_observed_time`만 갱신해 오래된 `previous`의0.3s 검사를 통과시키는 방식도 시각·점 나이 불일치를 만들므로 제외합니다.

## 이동 범위·추가 비용·reason 주의

- 추출 대상은 `PartialObservationPolicy.evaluate`의 previous 선택/만료부터 transport·위협·재관측·capture·commit까지 기존 한 블록(대략280–390행)입니다. `_refine_threat`의 동적 호출과 `TimedPolicy._remember`의 point_times/point_ids 정합을 유지해야 합니다. 메모리를 base `_remember`로 직접 우회하면 안 됩니다.
- `RecoveryPolicy._capture_latched_threat`는 전체 위협점과 원 배경을 저장합니다. 반면 짧은 `latch_evidence.witnesses`만으로 옛 위협을 재구성하는 것은 부족합니다. `_evaluate`는 top-level reason이 known_obstacle_latched일 때만 기존 해제 경로를 실행하므로, geometry 거절을 새 래치 이유로 덮어쓰거나 기존 래치를 더 늦은 expiry로 덮어쓰지 않아야 합니다. geometry는 원 이유, 내부 latch는 최초 원인과 시각을 보존하는 역할입니다.
- 기존 `joint_missing_sweep`에는 points≤1024, half-angle≤3°, age<0.3s 등의 계산 계약이 있습니다. directions≤16 검사는 **그 함수의 한 호출** 기준이며 `timed_missing_sweep`는 ray별1개씩 호출하므로 전체 gap16개 상한이라고 볼 수 없습니다. 이를 새 자르기 기준으로 도입하면 안 됩니다. 잘못된 metadata/미완결 원 gap을 성공한 검사로 취급하거나 처리할 ray를 임의 절단하지 않습니다. 계산 계약이 성립하지 않으면 **원 거절과 옛 기억을 그대로 유지**합니다. metric·넓은 결측의 다른 early return까지 한 번에 일반화하면 계산량·입력 계약 경계가 커집니다.
- 현재 이 창은 조기반환으로 위협 비용이0이지만 보완 후에는 기억 약450점 × missing6–7 rays × 조향별 기존 차체 sweep 계산 및 재관측 비용이 추가됩니다. 현재 단일 crop ray보다 커집니다. 현재 점/투영은 이미 전달되므로 새 투영은 필요하지 않지만, timed geometry 및 surface correspondence 비용은 실제 추가됩니다. ms/실시간 통과는 계산하지 않았습니다. r411 비용 작업과 합치거나 새 벤치마크를 실행하지 않았습니다.

## 가장 작은 수용 검사

구현 전에 아래 네 묶음의 기대 결과를 고정하는 것으로 충분합니다. 새 전수 replay는 제안하지 않습니다.

| 입력/반례 | 반드시 유지할 결과 |
| --- | --- |
| 저장684→685→686→687→688, 원 수치·기하 유지 | 685–687은 계속 geometry 거절. 각 새 scan에서 실제 소실 검사가 완료됐는지와 capture/commit을 기록. 위협이 없다면 기억 나이는0.1s로 이어질 수 있고, 위협이 있으면 기존 threat latch가 맞는 결과. 단순히 expiry가 사라진 것만 성공으로 판정하지 않음 |
| 옛 가까운 물체가 unbounded gap에 사라짐; 하나는crop 안, 하나는동일 gap의100° 바깥 | 그 프레임의 geometry 거절과 별개로 옛 위협을 latch/전체 capture. 다음 프레임에 finite 이웃이 복구되어도 재관측 없이 양수 허가/증거 소멸 금지. 먼 안정된 벽의 실제 재관측 사례와 구분 |
| 동일 source 반복, source/reference 역행·epoch reset, 실제 수신/운동 공백 | 중복으로 새 기억/확인2회를 만들지 않음. 기존 reset/invalid/진짜 expiry를 유지하고 갱신 금지. 실제 공백 뒤 새 스캔 하나로 기존 소실 증거 초기화 금지 |
| timed metadata 누락·gap이 kernel 범위 밖·기존 활성 근거리 latch | old snapshot/원 취득시각/anchor를 보존하고 원 거절 유지. 일부 광선만 선택하거나 현재점으로 옛 위협을 대체하지 않음 |

이 설계는 추출 범위와 증거 보존 순서가 작아 보이는 **정적 판단**이며 통과 구현이 아닙니다. 저장 창에서도 기존 위협 계산을 실제로 수행하면 다른 near-threat 래치가 생길 가능성이 남습니다. 그 경우 정상 expiry 회계 문제와 실제/보수적 소실 위협을 다시 구분하고, 통과를 위해 기존 위험 수치나 기억을 약화하지 않아야 합니다.

근거: `evidence_v1.json`의 원 입력 SHA·5행 시각/상태/광선 목록, 동결 r410 `verify_native_v1/report.json`·candidate/baseline rows, 공통 partial/traffic_recovery/known_structure/recovery_geometry 코드. 이번 준비물 외의 파일은 수정하지 않았습니다.
