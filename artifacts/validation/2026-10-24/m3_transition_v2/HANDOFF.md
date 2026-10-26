# GPT 인계 — M3 유한 실행 후보 (10/25 00:2x KST, 사용량 경고)

## 실행 상태
- 진행 중 실행 없음. WSL의 `replay_execution`/`analyze_run17`/`pytest`/사후 분석 프로세스 0개 확인.
- Haiku 5.5 보조 2개(run2, run3 실행)는 모두 완료·유휴. 둘 다 자체 보고 `claude-haiku-5-5`.
- 현재 소스 = run3 snapshot: `m3_execution_trial.py` sha256 `c57ed998af97223d…`, `tests/test_m3_execution_trial.py` `da90904f4d7db360…`
  (`run3_execution_replay/source_snapshot/`와 동일). 동결 `m3_observed_follow.py` `24073567e7b17597…`는 run17 사본과 동일(수정 안 함).
- ROS/launch/Gazebo·노드 연결·기본값 변경·커밋/푸시 없음.

## 수정/추가 파일 (Claude 작성분만)
- 새 소스: `src/arena_autonomy/arena_autonomy/m3_execution_trial.py` (명시 선택형 `M3ExecutionFollow`, 미연결)
- 새 시험: `tests/test_m3_execution_trial.py` (29개)
- 문서: `docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md` §8.7 추가, `docs/PROJECT_CONTEXT.md` 3·5·17행 교체,
  `docs/reports/DEVELOPMENT_PLAN_2026_10_23.md` §9.6 "10/24 밤 후속" 단락, `docs/activity/2026-10-24.md` 마지막 블록
- 이 폴더: `analyze_run17.py`, `replay_execution.py`, `hash_manifest.py`, `summarize_blockers.py`, `posthoc_positive_gaps.py`,
  `run1_run17_breakdown/`, `run2_execution_replay/`(무효), `run3_execution_replay/`, `dev/`(판정 근거 아님), `README.md`
- 메모리(작업 외): `ros-setup-nounset.md`
- 다른 세션의 기존 워킹트리 변경(wall_follow.py, .gitignore 등)은 건드리지 않음.

## 확인한 원인 (run1, 동결 run17 재호출 불일치 0/2,083)
1. 후보 첫 자세의 경로 좌표 왕복 오차(μm~mm) → 현재 차체 옆 후방 마스크 칸이 '새 미관측' → HOLD_STOP 154/82/70.
2. 0이 아닌 조향의 꼬리 흔들림(≤k·r²/2, 최대 약 1.5 mm)이 마스크 안 → 발행각 원호 진행 0.01 m에서 거절.
3. 실제 HOLD(조향 유지·정지)와 다른 0.5 s 상수각 확약(0.276 m) + fresh에서만 늘어나는 재개 계수 → 재개 교착.
4. 앞차 정지·재개 추가 정지 70은 등록된 잔디 쪽 실제 옆차 촉발(앞차 오인 아님).

## 실제 검사·결과 (run3, `run3_execution_replay/commands.txt` 4단계)
- `pre` 관련 206/206 통과(ROS Jazzy source), `recorded`/`self` exit 0, `post` 67파일 해시 불변·잔존 0.
- 측방 선택 뒤 양수 `full`: 기록 조향 G1 46·R0+ 100·정지재개 0 / 자기 조향 반사실 51·83·10. `frozen_run17` 전부 0(재현).
  단독 2,837/2,837 동일. `strict`(이력·직진 끔) 자기 조향 0.
- 실패 보존: run2 = run.sh가 `set -u`로 ROS setup source → 시작 전 exit 1, 산출물 없음(`run2_execution_replay/FAILED.md`).

## 한계·미해결 (숨기지 말 것)
- 전부 개루프(기록 센서/자세). 자기 조향도 반사실이며 폐루프 아님.
- R0+ 양수 19판단: 사후 정답상 후방 마스크 안 뒤따르는 옆차 5.5–9.7 cm(방위 ~158°). 후보가 못 봄.
- 이미 옆차가 97 mm 예산 안: 전 후보 거절→HOLD(G1 29·R0+ 15). 정지 중 마스크 띠 미지지(G1 28 등, 직진은 잔디 경계 미확인).
- 측방 활성 p95 약 57–75 ms(50 Hz 초과). 연속 외곽·실차 제동(2 m/s²)·차체 정지 미확인·0.5 m/s 상계 초과 숨은 차.
- 산출물 약 45 MB(`dev/` 16 MB). Git 포함 범위 미결정.

## 다음 정확한 작업 (GPT 결정 후)
1. 독립 검토 대상: `m3_execution_trial.py` 전체(특히 `anchored`, `envelope_distance`, `masked_only`/`support`/`SceneHistory`,
   `in_path_free_progress`, presteer 분기), 시험 29개, `replay_execution.py --self-steering` 반사실 정의.
2. E3 §8.7.6 D1–D6 결정(이력 지지·직진 출발·한 주기 확약·앞 물체 분담 채택, 현재 자세 예산 미달 처리, 후방 숨은 차 전제).
3. 승인 시에만: `local_pursuit.py` 기본 false 파라미터→`M3ExecutionFollow`, `follow_trace` 헤더 모드 추가와 재생 일치,
   §8.7.6의 사전 등록 사례 각 1회. 같은 사례 반복·간격 완화·미관측 면제 금지.
