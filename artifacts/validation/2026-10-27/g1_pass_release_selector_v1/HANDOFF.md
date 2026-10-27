# 인계 — 10/27 옆차 통과 뒤 G1 복귀 상태 선택 시제품 (Claude → GPT)

사용량 경고로 중단했다. 이 문서는 **10/27 두 번째 묶음(상태 선택 시제품)**의 인계다. 10/26 M3 후방 영역 인계(`2026-10-26/m3_rear_residual_v1/HANDOFF.md`)와 다른 작업이다.

- **실행 상태:** 실행 중인 Claude·Haiku 작업은 없다. WSL에서 관련 python·Gazebo 프로세스 0개를 확인했다.
- **변경 범위:** 운영 소스·노드·launch·기본값은 바꾸지 않았다. 새 Gazebo/ROS 주행, 커밋·푸시, Jetson/전원 변경도 없다.

## 1. 끝낸 일

- **run2 문구 정정:** 원시 증거는 수정하지 않았다. 고친 파일은 `g1_parallel_lane_v1/README.md`, `docs/PROJECT_CONTEXT.md`, `docs/DEVELOPMENT_PLAN.md`, `docs/activity/2026-10-27.md`다. 모두 줄바꿈을 보존했다.
  - 원래 판정: 주 평가 `gates`의 실패는 `final_gap`뿐이다. 다만 추가 actor 평가의 옆차 `sampled_gap`·`swept_gap`(150 mm)도 좌우 모두 실패다.
  - 정지 원인: `|y|>폭/2` 분류에는 벽도 들어가므로 "옆면 밖 반환이 첫 간섭을 지배"로 좁혔다.
  - 보완 감사: `g1_parallel_lane_v1/run2_first_interference_audit.py` → `.json`/`.log`(exit 0). 첫 간섭 반환을 격리했다. 간격 대기 82/82(왼쪽)·93/93(오른쪽) 모두 간섭 반환 전부가 평가 전용 정답의 옆차 상자 3 cm 안이었다. 제어기가 반환을 actor로 식별했다는 증명은 아니다.
  - 반환 나이: 첫 간섭 반환은 왼쪽 0.031–0.141 s, 오른쪽 0.075–0.190 s다. 0.025–0.19 s는 옆면 밖 반환 전체(벽 포함)의 범위로 구분해 적었다.
- **시제품 작성(미연결, 이 폴더만):**
  - `pass_release_selector.py`(sha256 `0d083106…`): 상태 SOLO/GUARD/RELEASED. 설계 규칙은 파일 머리말에 있다.
  - `replay_selector.py`(`234a323a…`): 저장 입력 4개를 재생하고 기록 출력 동일성과 불변식을 검사한다.
  - `synthetic_world.py`: 10/26 판본의 동일 사본(sha256 `ec89bac9…`).
- **재사용 판단:**
  - M3 `side_threats`·`side_return_masks`(벽 ±3 cm 제외·압축 3점), `body_queries`·`command_arc`·`masked_only`·`envelope_distance`는 순수 함수라 그대로 재사용했다.
  - M3 해제 관문·장부는 쓰지 않았다. 엄격판은 run17/21형에서 해제가 0이고, 10/26 합집합 시제품은 캐시 결함이 있고 재실행되지 않았다.
  - 물체 동일성은 가정하지 않는다. 통과 판정은 (1) 마지막 위협 장면의 최후단 반환이 앞면+0.10 m 앞이면 `benign`, 아니면 `rear`이고, (2) 그 뒤 서로 다른 fresh 2회 동안 옆면 밖 띠가 마스크 외 양의 자유 관측이어야 한다. `rear`는 세션 동안 다른 물체의 통과로 지워지지 않는다.

## 2. 실제로 실행한 검사

`run1_left`만 메모리 스모크로 확인했다. **산출물 파일은 만들지 않았다.**

| 항목 | 결과 |
|---|---|
| 기록 출력 동일 | 648/648 |
| 모드 분포 | SOLO 4 · GUARD 140 · RELEASED 504 |
| 상태 전이 | GUARD 3.92 s(사후 옆차 중심 0.46 m 뒤), RELEASED 6.72 s(옆차 0.53 m 앞, `benign`) |
| 해제 뒤 선택 사유 | `released_large_motion_observed` 467 · `small` 5 · `hold` 28 |
| 0.35 m/s 초과 선택 명령 | 448행(개루프, 실제 주행 아님) |
| 불변식 위반 | SOLO≠G1·GUARD≠평행·고장/비정상 사유의 양수 모두 0 |
| 비용 | 선택기 p95 18 ms·최대 26 ms(PC) |

## 3. 미완료·미검증

1. 정식 저장 입력 대조 run1을 실행하지 않았다(아래 명령). `run2_right`·`stop_restart`·`run11_lap`의 결과는 없다. `run11_lap`에서 벽·마커가 GUARD를 오촉발하는지가 핵심 확인 항목이다.
2. 합성 반례 하네스는 작성 전이다. 필요한 사례:
   - 벽만, 순수 단독 초기, 옆차 잔류
   - 뒤로 빠져 마스크로 사라짐(`rear`), 앞으로 통과
   - 같은 측면 2물체 중 하나만 통과, 앞차 정지, 재접근
   - 시계 되감기·세대 변경·장면 나이 초과, 이전 조향 크게 남음, 비정상 사유의 양수
   - 고정 출력은 `(speed, steer, {'reason':'local_follow','path_points':...})` 형태로 만들고, `synthetic_world.World`를 장면 생성에 쓴다.
3. 이 폴더 README와 현황·계획·활동 문서에는 시제품 결과를 반영하지 않았다.

## 4. 정확한 다음 명령

WSL 저장소 루트에서 실행한다. 순수 numpy라 ROS가 필요 없다.

```bash
/usr/bin/python3 -I -B artifacts/validation/2026-10-27/g1_pass_release_selector_v1/replay_selector.py \
  --output artifacts/validation/2026-10-27/g1_pass_release_selector_v1/run1_replay --workers 4
```

- 출력은 `summary.json`과 입력별 `*_rows.jsonl`이다(.jsonl은 10/27 Git 제외 규칙에 해당).
- exit 1이면 기록 불일치 또는 불변식 위반이므로 원인을 보존한 뒤 조사한다.

## 5. 남은 설계 판단(GPT)

- **RELEASED 뒤 큰 횡이동:** 현재 판단 원호의 현재 차체 밖 표본이 마스크 외 전부 관측 자유면 허용한다. 마스크 표본이 있는 쪽이 `rear`/`unknown`이면 거부한다.
  - 운동학상 왼쪽 복귀 때 왼쪽 뒤 모서리는 처음에 안쪽으로 움직인다. 꼬리 띠에 들어가는 것은 반대쪽 모서리의 약 1.5 mm 바깥 흔들림이다.
  - 그래서 run1에서 마스크 표본 0으로 큰 중앙 복귀가 허용됐다. 2 cm 외곽 표본이 mm 흔들림을 놓칠 수 있다는 한계가 남는다.
- **잔여 위험(명시):** 후면 평면 뒤 숨은 물체의 접근(뒤차 등급), 꼬리 띠 미세 물체(E3 §8.27 D7), 0.5 m/s 상계를 넘는 물체. ADR 0027의 정상 전진·작은 이동 재검토 범위로 다루며, 미관측을 자유로 바꾸지 않았다.
- **비교 근거:** 통과 판정의 0.10 m·두 fresh 2회·띠 폭 0.15 m는 시험값이다. 저장 4입력과 반례 결과를 본 뒤 연결 가능 여부와 제한 실행(run1/run2 같은 일정에서 해제 시각·앞차 최종 간격 비교)을 결정한다.
