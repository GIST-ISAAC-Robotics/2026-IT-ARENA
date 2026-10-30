# G2 통로 후보의 명시 기본 OFF 연결

2026-10-30. GPT 주 에이전트의 범위 결정에 따른 시험 옵션 연결이다. 운영 기본값·경기 기본값·실차 위험 수용 변경이 아니다. Claude Opus 5.5가 구현을, Claude Haiku 5.5가 Gazebo 실행·회수를 맡았다. 후보의 근거와 한계는 [후보 보고서](../g2_inner_boundary_v1/README.md)에 있다.

## 바꾼 것

| 파일 | 변경 | 기본 동작 |
|---|---|---|
| `bypass_corridor_trial.py`(새 파일) | 오프라인 후보를 전역 치환 없이 옮김. 등록 정책 `corridor_v1` | 명시 선택 때만 import |
| `bypass_entry_trial.py` | `_compare_entries`의 선택 인자 `padding_provider`, `EntryGuide.padding_provider=None` | None이면 기존 `current_outer_wall_padding` |
| `bypass_entry_follow.py` | `BypassExecutionConfig.corridor_policy`(`off`/`corridor_v1`) | `off`면 기존 `EntryGuide` |
| `g2_entry_node.py` | 파라미터 `g2_corridor_policy` | `off` |

기본 launch는 이 값을 고르지 않는다. 자동 G1→G2 전환(`g1_g2_node`)에는 연결하지 않았다. `/drive`→독립 보호→`/drive/safe`, 원시 장애물, 단일 모터축 28카운트/15:1/65 mm, 차량 물성, 명령 상한은 그대로다. 정답값은 제어기에 들어가지 않는다.

**등록값:** 관측 조건 항 0.225 m/s(0.03+0.06 m / 0.4 s), 시험 가속 3 m/s², 정렬 상한 30 mm. 0.225 m/s는 평균 변화량의 오차를 고려한 값이고 순간·미래 속도의 물리 상계가 아니다. 3 m/s²는 시험 가정이다. 30 mm 정렬은 결정론적 여유 보존이 아닌 탐색 가정이다.

schema4 판단 기록 헤더의 `execution_config`에 `corridor_policy`가 추가된다. 이 키가 없는 과거 기록은 `off`로 재생한다.

## 확인 결과

| 검사 | 결과 |
|---|---|
| 관련 회귀 [tests_c3](tests_c3.log) + [출처](provenance.log) | 277 통과, 새 출처 시험 포함 2 통과 |
| 오프라인↔런타임 같은 입력 [offline_runtime.json](offline_runtime.json) | 3사례 각 1031호출 일치(0.225 등록값, 저장 합성 입력의 개루프 재생) |
| 기존 G2 schema4 재생 [replay_old.json](replay_old.json) | 640/640, 639/639 일치 |
| 기존 G1/G2 schema5 재생 | 601/641, 603/656. 연결 전 소스도 [같은 값](replay_before.json)이라 이번 변경 영향이 아니다 |
| 전역 함수 치환 없음 | `test_module_function_is_not_replaced_while_candidate_runs` |
| source 갱신/epoch/reset/STOP 경계 | 근거 창 반례, reset 초기화, `reset_follow('SENSOR_STOP')` 포함 기록 재생 |
| 빌드 [build.log](build.log) | exit 0, 설치 경로 모듈 해시 대조 [installed2.log](installed2.log) |

처음 실행한 c1의 pytest 실패는 호출 옵션 오류이고, c2의 1건 실패는 후속 계약 전 출처 시험이었다. 후속 계약 [g2_corridor_r1](../../../../config/tests/g2_corridor_r1_sha256.json)을 추가해 c3에서 통과했다. 원시 로그를 보존한다. schema5 재생 불일치는 현황에 기록된 과거 음의 속도 처리 변경의 영향으로 보이나 이번에 원인을 다시 분해하지는 않았다.

저장소 전체 회귀는 실행하지 않았다. 관련 범위만 검사했으며 G2 시험 경로 밖의 소스는 바꾸지 않았다.
