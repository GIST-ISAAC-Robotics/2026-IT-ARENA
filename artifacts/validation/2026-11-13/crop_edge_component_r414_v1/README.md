# r414 제한 crop 경계 성분 prototype

**준비·동결, 미실행·미채택.** r413 `ADVICE_2.md`와 root `REVIEW.md`의 결정에 따른 기본 OFF 후보입니다. 현행 common partial source에서 시작했으며 미채택 r410 helper를 기본 동작에 넣지 않았습니다. Windows stdlib 저장 계보·AST·SHA만 확인했습니다. 정책/pytest/native/WSL/ROS/Gazebo/장치는 실행하지 않았고 common과 과거 artifact는 수정하지 않았습니다.

## 한 곳의 선택형 변경

`staged/arena_autonomy/partial_observation.py` 생성자 마지막 인자는 `crop_edge_component_trial=False`입니다. 엄격한 bool만 받습니다. False이면 새 인스턴스 상태도 추가하지 않으며 원 판정·기억 흐름을 유지합니다. True는 해당 인스턴스에만 적용되고 reset은 명시 선택을 보존합니다. 실제 노드·FollowConfig·guard 설정에는 연결하지 않았습니다. 비교 하네스만 새 planner 인스턴스를 만든 직후 명시적으로 True를 지정합니다.

True여도 `MeasurementScan`, `time_basis=simulation_acquisition`, history의 `verified_only` 정책이어야 후보를 적용합니다. 실제 Recovery 경로의 기존 투영·시각·점 순서 검사도 통과해야 합니다. 이것은 verified simulation 준비 범위이며 실물 취득시각 보증이나 host-estimated 허가가 아닙니다.

후보는 한 호출에서 **경계 run 하나가 crop 한쪽 끝에만 닿고, 기존 in-crop 판정이 비정면·비metric(폭≤3°)**일 때 원 배열을 검사합니다. 다른 내부 gap은 기존대로 계산합니다.

- 원 angular order에서 실제 유효 finite 반환 두 개까지 같은 missing 성분을 찾습니다. chain의 raw index가 ±1로 연속이어야 하며 raw 끝/처음 seam이나 정렬로 만든 이웃을 인정하지 않습니다.
- 성분/양 flank가 알려진 structure sector나 excused ray를 통과하면 거절합니다. 구조로 설명되지 않은 sector도 제외합니다. 음의 무한대·range_min 미만·유한 범위 밖 값은 건너뛰지 않습니다. 실제 finite flank는 원 range 범위 안이어야 합니다.
- 성분 전체와 다른 일반 missing의 합이 기존 **6°/front 표본수의5%** 안이어야 합니다. 두 실제 flank 거리는 기존 near boundary 검사에 모두 넣습니다.
- 취득/현재 방향이 정면이면 후보를 거절합니다. 기존 속도·0.5m/s cap·진짜 history 만료·소실 위협·래치는 그대로입니다. 큰 history 회계 refactor, 새 기억 구조·TTL은 없습니다.
- 허가용 `unknown`과 별도로 **전체 원 성분을 포함한 `threat_unknown`**의 점별 취득시각·방향·origin을 기존 near/timed sweep, 재관측, `_capture_latched_threat`에 전달합니다. ±100° 밖의 옛 점이 기존 여유로 잡힐 것이라고 추정하여 생략하지 않습니다. 현재 raw/점군을 수정하거나 unknown을 free로 채우지 않습니다.

`crop_edge_component` 진단에는 성분·in-crop·양 flank 원 인덱스, 원 거리, 전체/합산 폭, 합산 표본 비율과 검사용 전체 인덱스를 남깁니다. 이 진단의 존재는 최종 허가가 아닙니다. 이후 near/history/threat/guard 조건이 거절할 수 있습니다.

## 기존보다 넓어지는 허가와 위험

이것은 단순 crop 버그 수정이 아닙니다. 예를 들어 전체5°·양끝0.57m인 결측은 전부94–99° 안에 있으면 기존 metric 조건에서 거절되지만, 경계에 걸쳐 in-crop 부분만≤3°이면 이 후보에서 다음 검사로 갈 수 있습니다. 전체6°와 실제 양끝을 확인해도 원 정책보다 넓은 허가입니다. 처음부터 보이지 않은 물체, 실제 취득시각 오류, 기존2m/s 상대 운동 가정 밖은 해결하지 않습니다. planner와 guard가 나중에 같은 후보를 쓴다면 해당 허용 자체의 독립 안전 증명으로 볼 수 없습니다.

원22.62초의 `follow_partial_transformed_gap_uncovered`, 경로 선택·현재 재검사·명령 적용·차체 응답도 이 한 변경으로 해결했다고 주장하지 않습니다. 양수 호출 수 증가만으로 성공/지속주행 개선을 판정하지 않습니다.

## 준비한 핵심 검사 24개

`test_edge_component.py`는 실제 `structure_policy(RecoveryPolicy, 최종4 sector)` 구성을 사용합니다. motion/body 수치는 저장 r406 header를 복사한 `fixture_header.json`에서 가져옵니다. 인위적인 geometry/범위 반례만 해당 입력을 명시 변경합니다.

- **F1:** 원101.0/101.7/102.4°의 연속3점0.35m가 다음99.4–104° +Inf 성분에 사라집니다. 나머지 반환은0.57m이며 옛3점4cm 안에 현재 반환이 없습니다. `(speed, steering, previous)=(0,0,0),(.5,.25,0)`을 Python/native 각각 검사합니다. 기존 threat latch와 **capture unique_points=3**, 원 anchor 보존 및 현재 기억 갱신을 요구합니다. speed는 관측 정책의 sweep 인자이고 정지된 합성 motion 이력과 구분합니다.
- **F2:** 약105° 외곽1점. 기존 `timed_missing_sweep`의 직접 광선 위협 한계를 고정 구간에서24회 이분하여 그보다1cm 안쪽에 점을 둡니다. 여유를 튜닝하지 않고 전체 성분 경로가 이1점을 latch/capture하는지 검사합니다. native 결과의 한계 구간과 시험 거리는 pytest 로그에 남깁니다. 이 탐색은 모델 반례 구성이지 성능 시험이 아닙니다.
- in-crop 소실, 실제0.4s 공백, 같은 source의 나이 증가에 따른 `same_scan_missing_wait`, CW, 합산6°/5% 초과, seam·구조·범위 밖·가까운 먼 flank, 비시뮬레이션 시각·legacy array·불완전 native metadata를 검사합니다. 기존 metric·정면 경로와 OFF base의 전체 인스턴스 값도 대조합니다.

F1/F2 또는 기존 중단 조건이 실패하면 원 실패 로그를 보존하고 중단합니다. 예상한 capture 수에 맞추려고 소실 여유·폭·거리·fixture 점 위치를 실행 중 조정하지 않습니다.

## 제한 저장 대조

원본/OFF/ON을 **각각 새 process**에서 고유 plan254개로 순차 실행합니다. r410의 검증된 계보 로직을 복사했으며 원 raw265행의 중복11개는 input/output 동일성을 다시 검사합니다. current153과 공유 sequence의 누락30개를 복원하지 않습니다. 첫37계획(seq1..73)의 baseline 원출력은 exact oracle이며 이후217개는 같은 저장 순서의 open-loop 대조입니다.

원본→OFF는254개 전체 출력과 기록 상태가 exact여야 ON을 실행합니다. 기록 상태는 base7필드 전체 값, point_times/point_ids, 전체 anchor 상태·요약, structure previous/hidden/source입니다. planner 전체 객체의 상태 동등성과는 구분합니다. 원 출력도 보존하고 기존 `comparable_decision`의 표면 진단2곳 `processing_ms`만 비교에서 정규화합니다.

ON은 직접 성분 진단이 처음 생기기 전의 출력·기록 상태 동일성을 요구하고, 직접 대상/바깥 성분/후속 상태 전파를 행별 저장합니다. 이후 원출력 불일치와 양수 수·reason 변화는 결과로 보고할 뿐 누락 이력이나 폐루프 궤적을 복원한 것으로 표현하지 않습니다. guard의 실제 호출 순서·운영 연결은 이후 별도입니다.

## 고정 실행 명령과 회수

지정 실행 담당자만 기존 native 환경에서 다음 한 명령을 실행합니다. manifest의 source/native/input SHA가 다르거나 출력 폴더가 존재하면 중단하며 덮어쓰지 않습니다.

```bash
python -X utf8 -B artifacts/validation/2026-11-13/crop_edge_component_r414_v1/compare_saved.py \
  --output artifacts/validation/2026-11-13/crop_edge_component_r414_v1/verify_native_v1
```

순서: 단위24개(300초 상한) → 원본254 → OFF254 → OFF exact 확인 → ON254 → 비교(arm당600초). 실패/timeout/입력 변경이면 다음 단계로 가지 않습니다. 테스트·arm별 log/JSONL/receipt, `off_report.json`, `report.json`, `comparison_rows.jsonl`, `input_ledger.json`을 회수합니다. 추가 native build·비용 반복·새 live 주행을 이 명령에 포함하지 않았습니다.

원 source는 `cffd6898945ae5f8052f0dbd5467291a765d65d28cf620b32ec1ef239189a827`입니다. `proposed.patch`와 manifest가 변경·입력 경계를 고정합니다. 이 문서의24는 준비한 검사 수이며 실행 통과 수가 아닙니다.
