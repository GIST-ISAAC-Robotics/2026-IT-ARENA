# B3 선택형 경기 운영 도구

`scripts/race_operator.py`는 **이미 실행 중인 B3 후보**를 사람이 조회·조작하는 도구다. 기본 제어 모드를 바꾸거나 시뮬레이션을 시작하지 않는다. 신호를 대신 켜 주거나 앞차의 움직임으로 출발을 추측하지도 않는다.

## 1. 적용 범위와 주의

- 대상 노드는 `local_pursuit`의 `race_control_mode=b3_candidate`, `vision_execution=process`로 명시 선택한 경우다. 공식 출발 검출은 별도 `start_signal_mode=official_4red_candidate` 매개변수이며 두 선택은 서로를 자동 활성화하지 않는다. 현재 launch 기본값은 유지한다.
- **상위 정책 정지 ≠ MCU STOP ≠ 실제 차량 정지.** 이 도구는 `/race/command`만 발행한다. `/actuation/command`, `/drive`, `/drive/safe`에 발행하지 않는다. 통신이나 제어기가 고장 난 상황의 비상 정지 수단으로 쓰지 않는다.
- 사람의 `acknowledge`는 상위 래치만 푼다. 별도 MCU ARM·신호·센서·독립 보호 조건이 필요하지만, **그 조건이 이미 모두 갖춰져 있다면 해제 직후 움직일 수 있다.** 실제 위험 제거와 정지 확인 없이 확인 표시만 넣지 않는다.
- ROS domain 격리는 혼동 방지이며 인증이 아니다. 신뢰되지 않은 네트워크의 명령/상태 위조를 막는 도구가 아니다.
- 실물 MCU 시계 변환·전기 정지·공식 그리드 시야·실물 주행은 미검증이다. [B3 계약·현재 한계](RACE_START_CONTRACT_2026_10_13.md#8-1024--새-경기같은-경기-재개의-선택형-운영-연결), [M5 그리드 제약](GRID_START_M5_2026_10_24.md)을 먼저 확인한다.

## 2. 준비와 조회

기존 후보 실행과 같은 ROS 환경·domain에서 실행한다. 다음은 저장소 루트의 **WSL bash** 예시이며 후보 실행 자체를 시작하는 명령은 아니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 scripts/race_operator.py status
```

`status`는 발행자를 만들지 않는다. B3의 `/autonomy/status`가 하나여야 하고 새로 받은 상태의 `race`를 출력한다. session·race_epoch·intervention·latch·출발 사실·재ARM 필요·MCU 상태를 확인한다. `policy_allowed_if_sensors_ready`는 이름 그대로 조건부 값이지 실제 움직임 허가나 정지 증명이 아니다.

도구의 수신 신선도 0.5초는 **방금 받은 상태인지**만 본다. MCU/센서의 원천 시각·신선도는 제어기가 별도로 판정한다. 기본 응답 대기 5초는 도구의 실제 시간 제한이고 물리 제동 시간이나 제어기 TTL이 아니다.

## 3. 명령은 먼저 미전송으로 확인

`--send`가 없으면 ROS에 연결하지 않고 보낼 JSON만 출력한다. 아래 `확인한_SESSION`, 경기/개입 번호는 예시이며 **조회 후 사람이 확인한 현재 값으로 직접 대체**한다. 상태를 읽어 자동으로 번호를 복사하고 곧바로 해제하는 스크립트를 만들지 않는다.

### 상위 정책 STOP

```bash
python3 scripts/race_operator.py stop
python3 scripts/race_operator.py stop --send
```

활성 개입을 만들고 제어기의 0 요청을 유도한다. session/경기 번호가 틀려 정지가 막히지 않도록 STOP에는 해제용 번호를 요구하지 않는다. 다만 명령 수신자와 응답 발행자가 각각 하나이며 같은 노드인지 확인한다. 제어기 응답이 없으면 실제 적용 여부는 미확인이다. MCU STOP/물리 정지는 별도 경로로 확인해야 한다.

### 새 경기

```bash
python3 scripts/race_operator.py new-race --session 확인한_SESSION --race-epoch 0
# JSON과 대상 확인 뒤 같은 인자에 --send를 추가한다.
```

현재 경기 번호를 제시한다. 수락되면 경기·영상 세대가 함께 바뀌고 `new_race` 래치가 생긴다. 새 출발 시퀀스·사람 확인·래치 이후 별도 MCU ARM이 필요하다. 이전 경기의 신호 사실은 재사용하지 않는다.

### 같은 경기의 개입 해제

```bash
python3 scripts/race_operator.py acknowledge --session 확인한_SESSION --race-epoch 0 --intervention 1 --hazards-cleared --operator-reviewed
# 실제 위험 제거·검토·대상 확인을 마친 뒤 같은 인자에 --send를 추가한다.
```

두 확인 표시는 자동 교정이나 차체 정지 측정이 아니라 **운영자의 확인**이다. 미지정이면 도구가 거절한다. 같은 경기에서는 이미 확인한 출발 사실을 유지하지만 MCU 재ARM은 별도다. HOLD처럼 고정 개입이 없는 일시 대기는 이 명령 없이 기존 계약대로 회복한다.

## 4. 전송·응답과 오류

- `--send`의 새 경기/해제는 새 상태와 명시 번호를 대조하고, 상태·명령·결과 endpoint가 하나의 노드에 속하는지 확인한다. 서로 다른 노드나 중복이 있으면 전송하지 않는다.
- 확인과 배송 사이에 새 STOP/경기가 발생할 수 있다. 최종 번호 검사는 서버가 한다. 도구가 최신 번호로 자동 바꾸지 않는다.
- 한 번만 전송한다. 응답 누락·시간 초과·중단 시 재전송하지 않고 `unknown_after_send`로 남긴다. 이때는 먼저 `status`로 실제 상태를 확인한다. 명령 결과가 없다는 이유로 적용되지 않았다고 단정하지 않는다.
- 응답은 요청 UUID와 op가 같은 것만 채택한다. `accepted_policy_only`도 MCU 배송·차체 정지·주행 허가의 증명이 아니다.

| exit | 출력 outcome | 의미 |
|---|---|---|
| 0 | `status` / `dry_run_no_ROS_connection` | 조회 완료 / 미전송 JSON 생성 |
| 0 | `accepted_policy_only` | 상위 정책에서 수락한 응답 확인 |
| 2 | `not_sent` | 입력·대상·상태 검사 실패 또는 전송 전 시간 초과 |
| 3 | `rejected` | 서버가 해당 명령을 거절 |
| 4 | `unknown_after_send` | 전송 시도 뒤 적용 여부 불확실. 자동 재시도 금지 |

## 5. 검증 범위

[v3 검증 묶음](../../artifacts/validation/2026-10-24/b3_operator_v3/README.md)은 관련 84회귀(새 도구 32·기존 B3 52)·정상 종료·소스 불변·잔존 0을 확인했다. 순수 검사와 격리 ROS의 실제 `LocalPursuit` 연결로 상태·해제·새 경기·STOP·옛 번호 거절·응답 유실(서버 수신 정확히 1회)을 검사했다. 센서·MCU·Gazebo를 실행하지 않으므로 물리 정지/재출발 증명은 아니다. 현재 출발 주행을 막는 간격·시야 문제도 그대로다.

v1의 실패 2개(단위 시험 전제 누락·ROS 전용 context와 전역 executor 혼용)와 v2의 응답 유실 시험 전송 전 timeout은 각각 보존한다. v2의 정확한 discovery 지연 원인은 미확정이며 v3에서는 기존 도구 기본 대기 5초로만 비교했다. 이 짧은 결과가 장시간 운용·무선 네트워크·실물 인증/비상 정지를 보장하지 않는다.
