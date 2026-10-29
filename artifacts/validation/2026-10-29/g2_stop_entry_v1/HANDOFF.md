# 인계 — 이동하다 멈추는 앞차의 진입 공간 묶음 (Claude → GPT)

2026-10-29 05:28 UTC. 5시간 사용량 경고(five_hour 0.90)에 따라 Claude Opus 5.5 워커가 GPT에 인계한다. 상세 결과는 [README](README.md)를 기준으로 한다.

## 상태

- **할당 미완료:** 목표였던 "멈추는 앞차의 진입 후 정체 해소"는 이루지 못했다. 원인 분해와 시험용 정책 1종의 비교까지 마쳤다.
- **진행 중 실행 없음:** 05:28 UTC에 WSL 프로세스 목록에서 probe/replay/pytest가 없음을 확인했다. 모든 `.exit`가 0이고 Haiku 보조 호출도 모두 완료됐다. 중복 실행이나 유실 위험은 없다.
- 기본 정책은 `front_patch_v0` 그대로다. 보호 판정·차량/센서 물성·G1·launch는 바꾸지 않았다. 커밋·푸시·게시·Gazebo·`colcon build`는 하지 않았다.

## 발견한 원인 (확인)

1. 진입한 경로를 잘못 유지하거나 다시 고른 것이 아니다. 진입 경로 (편이 0.16 m, 전이 0.6 m)는 진입 시 관측 반환과 끝까지 간섭이 없었고, 차량은 그 경로를 따라갔다.
2. 가까운 진입의 짧은 전이 때문에 차체가 기운 채(0.24–0.28 rad) 표적 모서리를 지나 차체 간격이 0.129 m로 줄었다. 같은 스캔을 다시 쓰는 동안 표적 여유(0.5 m/s × 나이)가 0.101 → 0.141 m로 커져, 나이 0.14 s 이상인 호출마다 `known_occupied_command_hold`가 났다. 이 정지/진행 교대로 정체했다.
3. 진입 공간은 두 단계에서 줄었다. (a) 앞차가 멈춘 순간의 추종 간격(v1 0.819 m, v0 1.095 m). (b) 준비 완료까지 G1이 계속 다가간 거리. 기존 정책은 (a)에서 성공 사례와 같은 여유가 있었으나 (b)에서 잃었다.
4. 성공한 정지 가장자리 사례도 모서리 여유가 0–2 cm로 얇다. 중앙 표적 정체와 같은 기전이다.

## 가설 (미확인)

- `observed_entry_space_v1`에서 원거리 진입이 1 s 늦었다. 위치 유지로 급정지한 직후, 같은 정지 표적에 '조각 이동 증거'가 45호출 잡혀 준비 누적이 다시 시작된 것이 직접 원인이다. 그 뒤에 정지 중 운동 보정·엔코더 양자화 오차가 있다고 추정하나, 원인은 분해하지 않았다.
- 표적 여유 모델과 0.16 m 최소 편이의 조합이 성공/실패를 cm 단위로 가르는 것으로 보인다. 정적 기하 추정이며 완전 추종을 가정했다.

## 수정 파일

| 파일 | 변경 |
|---|---|
| `src/arena_autonomy/arena_autonomy/bypass_entry_trial.py` (CRLF 유지) | `_compare_entries(..., padding_growth_s=0.)`, `current_outer_wall_padding(..., growth_s=0.)`. 기본 0이면 기존과 동일하다(18기록 재생으로 확인) |
| `src/arena_autonomy/arena_autonomy/bypass_handoff.py` | 정책 `observed_entry_space_v1` 추가(`_execution_entries`, 위치 유지, 진입 거절), `_preparation(require_above_cap)` 추가. v0/v1 동작은 그대로다 |
| `tests/test_bypass_handoff.py` | 새 검사 5개(여유 증가0 동일·입력 거절, 위치 유지, 진입 거절/허용, 노드 파라미터) |
| `.gitignore` (CRLF) | g2_stop_entry_v1 대용량 기록 제외 규칙 |
| `docs/PROJECT_CONTEXT.md`, `docs/DEVELOPMENT_PLAN.md`, `docs/activity/2026-10-29.md` (CRLF) | 현재 G2 항목·현행 계획·활동 기록에 결과와 다음 판단 반영 |

이 묶음 직전(같은 날) 완료·검토된 준비 감속 묶음의 변경(`g1_g2_node.py` 파라미터, `observed_entry_relevance_v1`)은 [g2_prepare_cap_v1](../g2_prepare_cap_v1/README.md)를 따른다.

## 실행한 검사 (모두 exit 0)

| 검사 | 산출 | 결과 |
|---|---|---|
| 관련 회귀 | `../g2_prepare_cap_v1/tests3_stop_entry.log` | 166 통과 |
| 판단 기록 재생 | `trace_replay.json` | 18기록 각 600/600 |
| 기존 정책 멈춤 조건(판단 기록 포함) | `legacy_stop1/` | closed1과 같은 1.6646 m |
| 새 정책 폐루프 6조건 | `closed_space1/` | 표는 README §6 |
| 재출발 반례의 기존 두 정책 | `closed_restart1/` | 4.063/4.188 m |
| 저장 교통 1랩 7,801호출 | `shadow_space1/` | 위치 유지 0·감속 26·제안 0·G1 상세 일치 |
| 오프라인 분해 | `stop_entry.jsonl`, `stop_after.jsonl`, `corner_passage.json`, `entry_space_timeline.json` | — |

## 실행하지 않은 검사

- 새 정책 기준 v0/v1의 7,801 재비교. 두 정책의 코드 경로는 바뀌지 않았다.
- 이전 schema5 Gazebo 재생 재실행. 직전 묶음 결과는 601/641·603/656이며 원인은 run3의 음의 속도 처리다.
- Gazebo·ROS 노드 실행·`colcon build`. 실물 C1 시각에서의 여유 주기 확인.
- 급정지 직후 이동 증거 오검출 분해. 합성 각 조건의 반복 실행.

## 실패 원본 (보존)

- v1 정체: `../g2_prepare_cap_v1/closed1/stopping_combined__observed_entry_relevance_v1.jsonl`(.records.json)
- v0 정체: `legacy_stop1/stopping_combined__front_patch_v0.jsonl`
- 새 정책 정체/지연: `closed_space1/stopping_combined__…`, `closed_space1/right_far_combined__…`
- 재출발: `closed_space1/stop_restart_combined__…`, `closed_restart1/report.json`

대용량 `.jsonl`/`.records.json`/소스 ZIP은 로컬 보존이며 Git에서 제외된다.

## 후속 최소 명령 (WSL, 저장소 루트 기준)

```bash
# 회귀 재확인(로그 이름은 새 이름으로; 기존 이름은 덮어쓰기 거절)
bash artifacts/validation/2026-10-29/g2_prepare_cap_v1/run_tests.sh <새이름> tests/test_bypass_*.py tests/test_follow_trace*.py tests/test_wheel_contact_analysis.py
# 판단 기록 재생 재확인
bash artifacts/validation/2026-10-29/g2_stop_entry_v1/run_py.sh <새이름> replay_traces.py
# 오검출 분해 출발점: 원거리 새 정책 기록의 2.6–3.75 s(이동 증거 45호출)
python3 -I -B artifacts/validation/2026-10-29/g2_stop_entry_v1/decompose_trace.py  # 사례 인수 형식은 파일 docstring 참조
```

`decompose_trace.py`는 g2_prepare_cap_v1/closed1 기록을 읽도록 작성돼 있다. closed_space1을 분해하려면 경로 상수를 바꾼 새 사본이 필요하다.

## 다음 판단 (GPT)

1. 급정지 직후 이동 증거 오검출 분해(선행).
2. 이동 앞차 추종 간격 확대 여부. 준비 감속 성과(3,073 → 26 제안)와 충돌한다.
3. 표적 여유 모델(0.5 m/s × 나이)의 변경 여부. 보호 가정 변경이므로 결정이 필요하다.
4. `observed_entry_space_v1`은 시험용으로만 보존한다. 채택·폐기를 결정해야 한다.
