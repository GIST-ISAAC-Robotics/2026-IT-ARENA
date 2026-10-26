# M3 유한 실행·제동 전이 후보 — run17 분해와 저장 입력 대조 (10/24 밤)

기준 문서: [E3 §8.7](../../../../docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md#87-1024-밤--run17-분해와-유한-실행제동-전이-후보).
모든 결과는 저장 센서 입력의 개루프 재호출이다. ROS/launch/Gazebo 주행·노드 연결·실물 변경은 없다.
GPT 주 에이전트가 설계·검토, Claude Opus 5.5가 조사·구현·문서, Claude Haiku 5.5 보조(자체 보고
`claude-haiku-5-5`)가 run2/run3 고정 실행·회수를 맡았다.

| 경로 | 내용 | 판정 |
|---|---|---|
| `analyze_run17.py` → `run1_run17_breakdown/` | run17 동결 소스 재호출(출력 불일치 0/2,083) 뒤 촉발·후보·발행각·재개를 호출별로 분해. 정답은 계산 뒤 사후 표시만 | 원인 분리 근거. 해시 전후 동일 |
| `run2_execution_replay/` | 첫 고정 실행 시도 | **무효(시작 전 셸 오류, 산출물 없음)** — `FAILED.md` |
| `run3_execution_replay/` | 동결 소스 관련 회귀 + 저장 4기록 대조(기록 조향 / 자기 조향 반사실), 해시·잔존 확인 | 아래 결과 |
| `replay_execution.py` | 대조 하네스. 변형 `full`·`no_history`·`no_straight`·`strict`·`frozen_run17` | — |
| `summarize_blockers.py` | run3 행 기록의 남은 막힘 분류(읽기 전용 사후 분석) → `run3_execution_replay/analysis/` | — |
| `posthoc_positive_gaps.py` | 측방 선택 뒤 양수 판단의 사후 정답 간격·방위(읽기 전용) | 제어 입력 아님 |
| `hash_manifest.py` | `src/arena_autonomy/arena_autonomy/*.py`·관련 시험 8개·도구·저장 입력/사후 정답 파일(총 67개) SHA-256 | — |
| `dev/` | 동결 전 탐색 산출물 | 판정 근거 아님(`dev/README.md`) |
| `HANDOFF.md` | 사용량 경고 뒤 GPT 인계(실행 상태·해시·다음 작업) | — |

## 재현

`run3_execution_replay/commands.txt`의 네 단계(`pre`, `recorded`, `self`, `post`)를 순서대로 실행한다.
`pre`는 해시·소스 사본·관련 회귀, `recorded`/`self`는 대조, `post`는 해시 재확인·잔존 프로세스 기록이다.

## 해석 한계

- `recorded`: 직전 조향은 원래 기록값이다. 후보가 낸 조향과 다르므로 정지 중 조향 정렬·조향 전이는 평가할 수 없다.
- `self_steering`: 센서·자세·측정 속도는 기록 그대로 두고, 각 제어기(기준 G1 포함)가 자신의 직전 출력을 직전
  조향으로 받는 반사실 진단이다. 후보가 정지를 내도 기록 차량은 계속 움직였으므로 폐루프 결과가 아니다.
- 사후 정답 간격은 원래 기록 자세의 값이며 후보 명령을 따른 결과 간격이 아니다.

## run3 결과 요약

- `pre`: 관련 206/206 회귀 통과(ROS Jazzy 환경, `regression.xml`). `recorded`/`self` 재생 exit 0. `post`: 67파일 해시 전후 동일(`hash_check.txt`), 잔존 프로세스 0. 시작/종료 UTC는 `started_utc.txt`/`finished_utc.txt`.
- 측방 선택 뒤 양수(`full`): 기록 조향 G1 46·R0+ 100·정지·재개 0, 자기 조향 반사실 51·83·10. `frozen_run17`은 모두 0(run17 재현). 단독 2,837/2,837 동일.
- 남은 막힘: `analysis/remaining_blockers.json`(`summarize_blockers.py`). 사후 정답 간격: `analysis/posthoc_positive_gaps.json`(`posthoc_positive_gaps.py`) — R0+ 19판단은 후방 마스크 안 옆차 5.5–9.7 cm.
- 행 기록 `recorded/`·`self_steering/`의 `rows_*.jsonl`은 판단별 요약이며 원시 센서는 원래 E3/G1 기록 폴더에 있다.
