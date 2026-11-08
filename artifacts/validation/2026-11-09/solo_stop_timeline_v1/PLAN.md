# r25 단독 주행 명시 STOP 시간선 분해 계획

2026-11-09, 저장 로그 분석 전용. 새 시뮬레이션·Jetson 실행·소스/설정/판정 변경은 하지 않는다.

## 대상

- 주 대상: [r25 실제 단독 주행](../../2026-11-08/solo_gap_envelope_connected_v3/README.md)의 `run1/`. 보고값 STOP 대기1.446s(시뮬)·변위0.945m.
- 비교(1개만): [같은 날 교통 경주거리 시험](../../2026-11-08/traffic_race_distance_v1/README.md)의 `two_actor_race_distance/run1/report.json`·`stop_trace.json`. 다른 평가기(`g1_traffic_eval`)이므로 정지 직전 속도·변위만 참고한다. 직전 `solo_corner_approach_connected_v1`은 42.66m 미완주로 STOP 판정 기록이 없어 쓰지 않는다.

## 읽는 입력과 시계 영역

| 입력 | 내용 | 시계 |
| --- | --- | --- |
| `report.json` | `stop_wait_sim_s`, `stop_displacement_m`, `dense_pose_stop_evidence`, 최종 MCU/출력 | Gazebo pose 헤더 시뮬 시각 |
| `stop_trace.json` | 평가기 STOP 루프의 pose와 평가용 차체속도 | Gazebo pose 헤더 시뮬 시각 |
| `ego_poses.jsonl` | 조밀 pose(변위 재계산) | Gazebo pose 헤더 시뮬 시각 |
| `actuation_trace.json` | 가상 MCU `/actuation/status`: 상태·목표속도·모터축 누적 카운트·캡처 시각 | MCU 노드 ROS 시계 µs(`/clock`) |
| `drivetrain_gz.jsonl` | 구동 플러그인 적용 목표·모터/좌우 바퀴 각속도·토크·평가용 차체속도(약21ms 간격 표본) | Gazebo 플러그인 `sim_time_s` |
| `follow_decisions.jsonl` | 대회 제어기 결정 단계 출력(속도 명령) | 결정 입력 `reference_s`(센서 기준 시각) |
| `wheel_contacts.jsonl` | 바퀴-노면 접촉 메시지(정지 창만 순차 읽기) | Gazebo 접촉 헤더 시뮬 시각 |

시계 영역은 표에서 열을 나누어 기록하며 서로 더하거나 빼서 지연을 만들지 않는다. 같은 시뮬 시간축을 공유한다고 알려진 Gazebo 계열 내에서만 구간 길이를 계산하고, MCU ROS 시계와의 차이는 "같은 `/clock` 유도로 가정" 표시를 붙여 참고값으로만 둔다. 평가 wall 시각은 STOP 구간에 기록이 없으므로 미기록으로 둔다.

## 산출

- `analyze.py`: 입력 해시 확인 후 위 키만 선택 창(37.3–39.1s)에서 읽어 `summary.json`을 만든다. 입력 파일은 읽기 전용.
- `summary.json`: 시간순 사건표, 단계 분해(요청→MCU 래치→플러그인 목표0→첫 감속→0 교차→역회전/안정 시작→0.5s 판정 창), 미기록 필드, 비교 1건.
- `input_sha256.json`: 읽은 입력 해시.
- `README.md`: 결론·한계·대안 해석.

## 판정 원칙

- 평가용 정답(차체속도·바퀴 각속도·접촉)은 원인 분리용이며 제어 입력이 아니다.
- 잠김·떠있음·음의 축 회전은 해당 로그 값이 직접 보일 때만 분류한다.
- 시뮬레이션 제동 결과를 실제 회로 재현이나 실차 제동거리로 확대하지 않는다.
