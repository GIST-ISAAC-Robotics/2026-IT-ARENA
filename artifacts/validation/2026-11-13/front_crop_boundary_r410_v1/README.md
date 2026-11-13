# r410 crop 경계의 실제 이웃 복구 후보

준비 상태입니다. Windows stdlib 저장 분석·AST·SHA 확인만 수행했습니다. 정책/pytest/native/ROS/WSL/시뮬레이션은 실행하지 않았고, common·원 trace·기존 결과는 수정하지 않았습니다. 실제 주행에 채택하지 않습니다.

## 확인한 원인과 적용 범위

`partial_observation.py`는 원 측정 배열을 ±100°로 crop한 뒤 결측 run이 crop 첫/끝에 닿으면 이웃을 찾기 전에 `unbounded_missing`을 반환합니다. 그때 `_remember`도 수행하지 않으므로 단일 호출의 기하 복구와 이후 history/래치 상태 변화는 구분해야 합니다.

저장 r406 run1의 reference ≥25초 raw90행에서 계획0은86행이며 원 normal reason은 `follow_unbounded_missing`77 / `follow_release_wait`9 / `local_follow`4입니다. 77행 모두 +100° 경계의 원 +Inf이며 4개 known-structure sector와 겹치지 않습니다.

| 원 경계 실패 | 행 수 | 원 배열에서 확인한 내용 | 후보 범위 |
| --- | ---: | --- | --- |
| finite 이웃만 crop 밖 | 27 | 전체 결측은 crop 안 1 ray, 폭 0.684–0.720°. 즉시 바깥 유효 이웃은 약 100.296–100.368°. 이웃 거리 약 0.5696m / 7.117m | 실제 이웃만 다시 참조하고 기존 나머지 판정으로 진행 |
| 결측 자체가 crop 밖으로 계속됨 | 50 | 전체 결측 폭 4.248–5.040°, 약 6–7 rays. 원 유효 이웃까지 여러 unknown이 있음 | 기존 `unbounded_missing` 유지 |

대표 trace696은 원 idx117(99.504°)에서 idx111(103.824°)까지 실제 결측이며 전체 폭5.004°입니다. trace697은 idx118(99.576°) 한 ray만 결측이고 바로 다음 idx117(100.296°, 7.117m)이 유효 반환입니다. 상세 원 인덱스·각도·거리·취득 센서 좌표는 `analysis_v1/report.json` 및 rows에 있습니다. 좌표는 취득 센서 좌표이며 현재 차체 자유공간의 증거로 승격하지 않습니다.

원 r377 계획225행과 r396 입력 계보의 current223행에는 crop edge 결측 자체가 0행입니다. 이 결과는 원 입력의 분기 비대상 분석이며 새 정책 전체 출력 동일성을 실행 확인했다는 뜻은 아닙니다.

## 변경 한 곳

`staged/arena_autonomy/partial_observation.py`와 `proposed.patch`는 기존 경계 분기에서만 `_cropped_gap_flanks`를 호출합니다. 원 전체 angular order에서 run 앞뒤가 **즉시 유효 finite 반환**이고, 사이에 다른 원소가 없으며, raw source index가 ±1로 연속일 때만 두 실제 반환을 사용합니다. CW/비균일 측정도 같은 조건을 적용하고 취득 배열 끝/처음 seam은 잇지 않습니다.

바깥 unknown을 건너뛰거나 crop을 넓히지 않습니다. unknown 값/개수/전체 run 폭, 3°/6°·50mm·1m·근거리·속도·시간·소실 위협·래치 기준을 그대로 둡니다. 새 반환/점군/자유공간을 만들지 않습니다. 이 base policy를 쓰는 planner와 독립 guard에 공통으로 적용할 수 있는 한 곳의 수정이며 별도 guard 예외는 없습니다. guard의 실제 호출 순서·주행 효과는 이 저장 준비물로 검증하지 않습니다.

원 50행은 바깥 unknown을 포함하므로 후보에서도 해당 경계 분기를 통과하지 못합니다. 일부는 기존 metric 1m 이웃 조건과 ±100° 현재방향 조건에도 맞지 않지만 이를 바꾸는 후보가 아닙니다. 27행이 최종 양수 계획으로 바뀔지는 history·소실 위협·selector/보호 결과까지 계산한 뒤에만 알 수 있습니다.

## 제한 저장 대조와 계보 한계

`input_ledger_v1.json`은 stdlib로 확인한 계보입니다. raw265행은 고유 plan254 + input/output 완전동일 중복11행입니다. current153개와 sequence를 공유합니다. 합집합의 누락30개 중5개는 input/output 없는 current 폐기 기록이며 나머지25개는 단계/입력 미복원입니다. 최초 누락 sequence74 전의 plan37개(seq1..73)만 원 자식 입력 이력이 온전한 prefix입니다. 그 뒤217개는 동일 저장 입력 순서의 open-loop 비교이며 원 자식 이력 재현이 아닙니다.

중복 제거 후 reference ≥25초는88개입니다. unbounded77(직접27/바깥50)과 전체 직접대상31개는 중복이 없습니다. `analysis_v1`은 raw행 분석 그대로 보존하고 고유 개수는 ledger에서 구분합니다.

`compare_saved.py`는 별도 새 process의 baseline/candidate evaluator에서 같은 고유254개를 순서대로 계산합니다. generation 변경 때만 기록 reset 이유를 적용합니다. 입력·원 header의 수치/기하/시간은 보존하고 기존 common `cad_runtime_r377_v1`·`native_v1` 및 기록된 r396 환경 계약만 명시합니다. candidate 자식의 첫 import 전에 단일 staged partial 모듈 경로를 추가하며 전역 캐시/운영 monkeypatch는 도입하지 않습니다.

각 arm은 전체 판단 출력과 `previous`, `previous_distinct`, `last_source`, `clear_sources`, `latched`, `latch_reason`, `latch_time`의 전체 값을 행별 저장합니다. 전체 planner/structure 객체 상태 동등성 주장과 구분합니다. 기존 `comparable_decision`이 지정한 표면 정합의 두 `processing_ms` 진단만 비교에서 정규화하며 원 output도 별도 남깁니다. 직접대상 전의 예상 밖 차이, 첫37개 baseline 원출력 불일치는 중단합니다. 이후 원출력 불일치는 누락 이력 한계와 함께 기록하고 판정선을 맞춰 없애지 않습니다. 직접대상 이후 바깥50행의 출력/상태가 바뀌면 경계 허용과 이전 상태 변화 전파를 구분해 검토합니다.

## 지정 실행 담당자 인계

`manifest.json`은 준비물·common Python/native 파일·저장 입력의 SHA를 고정합니다. common/원자료 변경 시 실행을 중단하며 자동 재생성하지 않습니다. 출력 폴더가 이미 있으면 덮어쓰지 않습니다. 한 명의 실행 담당자가 기존 numpy/pytest/native 환경에서 다음 한 명령을 수행합니다.

```bash
python -X utf8 -B artifacts/validation/2026-11-13/front_crop_boundary_r410_v1/compare_saved.py \
  --output artifacts/validation/2026-11-13/front_crop_boundary_r410_v1/verify_native_v1
```

명령은 기존25개 단위 반례 → baseline254 → candidate254 → 비교 순서입니다. 단위검사180초, 각 arm600초 상한이며 실패/timeout/SHA 변경 시 다음 계산을 하지 않습니다. 단위 범위는 array/native CCW/CW/nonuniform, 실제 유효 이웃, 바깥 결측, NaN/Inf/범위 밖 이웃, raw seam·비인접, history warmup/만료·래치·옛 근거리 소실·현재 근거리 및 비대상 상태 보존입니다. 정책 실행 결과가 아직 없으므로 통과 수나 허가 회복 수를 미리 적지 않습니다.

`report.json`, arm별 `*.jsonl`·receipt·log, `comparison_rows.jsonl`·`input_ledger.json`을 회수합니다. candidate에 양수 출력이 생겨도 연속진행·실제 보호층 통과·차체 응답·완주 개선으로 부르지 않습니다. 기대 효과가 없으면 이 한 곳의 수정과 원인 분석을 분리하여 미채택하고 새 속도/각도/거리 완화로 이어가지 않습니다.
