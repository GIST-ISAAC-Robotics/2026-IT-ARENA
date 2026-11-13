# B3 선택형 경기 운영 도구

> **11/3 후속:** 일반 명시 STOP 뒤에는 별도 RESET 버튼 없이 START로 재개 준비할 수 있도록 변경했다([검사·시뮬 재개 증거와 평가기 실패](../../artifacts/validation/2026-11-03/ordinary_resume_v1/README.md)). RESET은 정지를 유지하며 상태만 먼저 초기화할 때 사용한다. START 자체도 새 입력/경로와 MCU 재ARM을 확인한다.

> **11/2 후속:** 웹 화면과 같은 경기 복구가 [기본 OFF 후보로 연결됐다](../../artifacts/validation/2026-11-02/official_ops_v1/README.md). 운영은 아래 0절을 우선한다. `new-race`/`legacy_reset`은 출발 사실을 지우므로 세이브포인트 복구용으로 혼용하지 않는다. 옛 수동 번호 입력 지시가 복구 계획과 충돌하면 계획을 따른다.

## 0. 웹 화면으로 운영

**11/7–8 선택형 늦은 출발 후보:** `official_marker_late_candidate`를 쓸 때는 주최 측의 사전 녹색 점등 시험이 끝나고 등이 꺼진 뒤 **최초 출발 준비**를 누른다. 신호 옆 ID45·0과 지속 녹색을 확인하여 빨강이 앞차에 가렸어도 늦게 출발하는 후보이므로, 준비 이후의 시험 녹색과 진짜 출발 녹색을 영상만으로 구별할 수는 없다. 준비 전 영상은 새 영상 세대로 차단하지만 이 운영 순서를 대신하지 않는다. 대표 광학 가림 해제 주행은 확인했고 실제 경기·실차 사용은 미검증이다. 일반 `official_4red_local_candidate`는 기존 빨강 이력 확인을 유지한다. [후보·실험 한계](../../artifacts/validation/2026-11-07/late_start_marker_v1/README.md).

차량에서 `race_ops`를 실행하고 노트북 브라우저로 연다. 공용 AP 접속·보안·실제 정지는 현장 미검증이다.

**실물 runtime 시작:** 이 웹 프로세스만 실행해도 planner/guard가 B3로 전환되는 것은 아니다. [최종 profile·양쪽 B3 노드·UART host의 진입 템플릿](../../artifacts/validation/2026-11-13/runtime_entry_audit_r415_v1/REVIEW.md#운영-진입-명령-3개)을 먼저 확인한다. r415는 당시 기본 진입 템플릿이며, 이후 실장 조향 상한은 아래 `runtime_installed_limits.py`의 생성값으로 연결한다. r444의 추정시각 관측 보완은 [별도 명시 선택](../../artifacts/validation/2026-11-13/estimated_observation_r444_v1/RESULT.md)이고 기본 활성화·실물 주행을 확인한 상태는 아니다. host에는 clock policy·motor feedback 설정이 필요하며 ESP 포트는 host 하나만 소유한다. 기존 센서 preview에 별도 host를 중복 연결하지 않는다. 실장 영점과 출력 프로필의 빌드 연결/장착 확인은 [실물 이관](../hardware/PHYSICAL_HANDOVER_APPROVED_LAYOUT.md#6-esp32-프로그램-이관)에 남아 있다.

```bash
ros2 run arena_autonomy race_ops --ros-args -p vehicle_id:=car1 -p host:=0.0.0.0 -p port:=8765 \
  -p token_path:=/path/token.txt -p record_path:=/path/race_record.json -p log_path:=/path/ops_log.jsonl
```

토큰 파일이 없으면 새로 만든다. 읽기 전용 접속은 `view_token_path`로 별도 토큰을 준다. `host` 기본값은 `127.0.0.1`이다. 패널은 실행 중인 HTTP 서버의 주소로 연다. 예를 들어 위 포트 설정이면 `http://Jetson주소:8765/`이며, SSH 포워딩을 사용하면 그 로컬 포트를 연다. 저장한 HTML 파일을 직접 여는 방식은 `/api/state`와 명령 API가 제공되는 운영 패널 실행을 대신하지 않는다.

| 상황 | 조작 |
|---|---|
| 프로세스 시작·새 경기 뒤 | **최초 출발 준비**: 위험 제거 확인 뒤 해제·ARM. 최초 출발은 영상 신호 확인 또는 `WAIT_GREEN`에서 운영자의 명시적 신호 확인을 사용한다. MCU·센서·경로·보호 조건은 유지한다 |
| 제자리 잠깐 정지 | **STOP → 재개 START**. 일반 명시 STOP일 때 별도 RESET 버튼은 생략 가능. 고장·물리 정지 복구와 구별 |
| 상태만 먼저 초기화 | **STOP → 복구 RESET**은 정지를 유지한다. 이후 최종 배치 확인 뒤 **START**. 일반 명시 STOP 뒤에는 재배치했더라도 START 자체가 새 입력으로 준비하므로 별도 RESET 버튼은 필수가 아니다 |
| 제어 프로세스 재시작 뒤 | 화면의 저장 기록이 현재 경기임을 확인하고 **같은 경기 복구** → **재개 START** |

- 번호를 입력하지 않는다. 화면이 보여 준 경기·개입 번호가 버튼과 함께 묶이고, 달라졌으면 거절된다.
- START 수락은 주행 허가가 아니다. 화면이 `복구 준비`에서 `주행`으로 바뀌어야 허가다. 취소되면 이유가 표시되고 자동 재실행되지 않는다.
- 결과 미확인이면 다시 누르기 전에 상태를 확인한다. 화면은 자동 재전송하지 않는다. STOP은 언제든 다시 누를 수 있다.
- Wi-Fi STOP은 HW 정지를 대체하지 않는다. 화면의 실제 차체 정지는 항상 `미확인`이다. 모니터 연결이 끊겨도 정상 주행 차량은 멈추지 않는다. 다만 START 준비 중 끊기면 그 START는 취소된다.
- ID30을 보지 못한 오른쪽 구간 재배치는 [대표 1회](../../artifacts/validation/2026-11-02/official_ops_recovery_v2/README.md)에서 확인했다. 모든 세이브포인트·가림 조건을 검증한 것은 아니다.
- 사건 시간축: 화면의 CSV/HTML, 또는 `scripts/race_event_export.py --log ops_log.jsonl --request 요청번호 --output 출력경로`.

**일반 시험에는 웹 창이 필요 없다.** 자동 평가기는 같은 명령 인터페이스를 사용하고 기록을 회수한다. 브라우저는 화면/버튼 연결을 검증하거나 사람이 현황을 볼 때만 연다.

아래는 CLI 사용법이다. `recovery-reset`·`resume-start`도 같은 번호 규칙으로 쓸 수 있다.

`scripts/race_operator.py`는 **이미 실행 중인 B3 후보**를 사람이 조회·조작하는 도구다. 기본 제어 모드를 바꾸거나 시뮬레이션을 시작하지 않는다. 신호를 대신 켜 주거나 앞차의 움직임으로 출발을 추측하지도 않는다.

## 0.1 기본 속도 상한과 수동 단발 시험

B3 운영 패널에는 별도 개발 모드 없이 **속도 상한**이 표시된다. m/s와 km/h 중 한쪽을 편집하면 다른 쪽도 변환한다(1 m/s = 3.6 km/h). 편집값과 서버 적용값을 구분하고, 정지 및 MCU 비ARM 상태에서 적용한 뒤 기존 출발 준비/START를 사용한다. 상한을 적용하는 것만으로 ARM하거나 출발하지 않는다. 주행 중 변경은 현재 범위에 없다.

기본 초기 상한은 현재 차량의 `max_speed_mps`다. 이전 `development_speed_limit_enabled:=true development_speed_limit_mps:=0.3`을 명시한 실행은 그 초기값을 유지한다. STOP·경기 초기화·브라우저 종료 때문에 상한을 자동 해제하지 않는다. 이는 목표속도 제한이며 실제 속도·제동거리 보증이 아니다.

수동 시험은 자율 제어기를 먼저 종료한 **별도 `operation_mode:=manual` 실행 구성**에서 사용한다. 기존 guard와 actuation host가 필요하며 `/drive`의 다른 발행자가 있으면 준비/실행을 거절한다. 기존 프로세스를 자동으로 끄거나 보호층을 우회하지 않는다. 일반 자율 구성은 수동 자식이나 graph 주기 검사를 실행하지 않는다.

```bash
ros2 run arena_autonomy race_ops --ros-args -p operation_mode:=manual \
  -p vehicle_id:=isaac -p host:=0.0.0.0 -p port:=8765 \
  -p token_path:=/확인한/운영토큰경로 \
  -p manual_steering_limit_rad:="${MANUAL_STEERING_LIMIT:?설치 상한에서 계산한 min(0.2,L)}" \
  -p log_path:="${RUN_OUTPUT:?이번 실행의 새 결과 폴더}/ops_manual.jsonl" \
  -p manual_output_root:="$RUN_OUTPUT/manual_jogs"
```

`RUN_OUTPUT`은 미리 만든 이번 실행의 절대 경로다. 같은 폴더 아래 host·패널·단발 결과를 모으되 host가 생성할 하위 `host` 폴더는 미리 만들지 않는다. 수동 구성은 자율 status의 시각이 없으므로 사건 내보내기에 `--request`를 지정하지 않고 `python3 scripts/race_event_export.py --log "$RUN_OUTPUT/ops_manual.jsonl" --output "$RUN_OUTPUT/events_manual"`로 전체 기록을 내보낸다. `manual_jogs` 원 request/result/stdout과 host 원 UART/사건 기록도 함께 회수한다. `log_path`를 생략하면 패널 디스크 로그는 남지 않는다.

1. 차량 이름/토큰과 정지 상태를 확인하고 필요하면 상한을 적용한다.
2. **수동 준비**를 눌러 명시 CLEAR→ARM을 확인한다. 준비만으로 주행하지 않는다.
3. 속도·실행 시간·중앙 등가 바퀴 조향각(+좌/−우)을 입력하고 단발 실행한다. 서보축 각도가 아니다.
4. 실행이 끝나면0명령·MCU STOP·상태 확인 뒤 자식을 회수한다. 다시 실행하려면 다시 준비한다.

현재 단발은 최대0.2m/s·0.05–2초, 기본 조향±0.2rad다. 실장 한계보다 크게 설정하지 않는다. `v×t` 표시는 등속 가정 예상 주행거리이며 실제 이동/정지거리나 곡선의 직선 변위가 아니다. 입력 초과값을 몰래 줄여 실행하지 않고 거절한다. WASD는 요청 범위에서 제외했다. 수동 실행 중 브라우저 연결이 끊기면 취소/STOP을 요청하며, 정상 자율주행은 브라우저 창을 필수로 요구하지 않는다.

실제 설치 bundle에서 운영 상한을 만들 때는 [`runtime_installed_limits.py`](../../scripts/runtime_installed_limits.py)를 사용한다. 생성한 같은 YAML을 planner와 guard에 선택하고, `host-arguments`의 값은 반올림하지 않고 host에 전달한다. 별도 수동 구성의 `manual_steering_limit_rad`도 **기본0.2rad와 생성 상한 중 작은 값**으로 맞춘다. 이 값은 중앙 등가 바퀴각이며 서보축 각도가 아니다. [실행 예시](../../artifacts/validation/2026-11-13/installed_limit_chain_r439_v1/RESULT.md#이관-후-사용할-최소-흐름). 설정 생성만으로 펌웨어가 바뀌거나 차량이 출발하지 않는다.

[r429](../../artifacts/validation/2026-11-13/operator_panel_r429_v1/RESULT.md)에서 기존140·신규19검사, 후속56영향검사·DOM 검사와 실제 HTTP/ROS/가짜MCU 대표를 확인했다. 실제 차량·무선 AP·브라우저 시각 검수는 이 대표에 포함하지 않는다. 기존 CLI `race_operator.py set-speed-limit`와 `development_jog.py`도 유지한다.

## 0.2 선택형 라이다·카메라 보기

기존 패널의 센서 미리보기에서 필요한 센서와 **실제 발행 중인 토픽/메시지 형식**을 고르고 적용한다. 기본은 모두 OFF다. 같은 operator/view 토큰을 사용하며 별도 프로그램이나 카메라 장치 재개방은 필요 없다.

- 시뮬 기본: native JSON `/scan/native` 또는 LaserScan `/scan`, RGB `/camera/color/image_raw`.
- 현재 실물 runtime 입력을 쓰면 native `/hardware_runtime/c1/native`, RGB `/hardware_runtime/camera/image_raw`처럼 실제 실행에 지정한 이름을 넣는다. 센서 발행기는 기존 실행이 담당하며 패널이 새로 시작하지 않는다.
- 라이다는 센서 좌표의 유효 반환 점·거리·수신 나이를 보여준다. 기본3m 표시 반경을 조절하고 점에 포인터를 올려 거리를 읽는다. 최대720점·5Hz이며 무반환은 자유 공간이 아니다. 벽/차량 분류 결과나 운동 보정 결과가 아니다.
- 카메라는 최신 한 장을 최대 가로640px·2Hz JPEG로 전송한다. 수신1초 초과는 오래된 영상으로 표시하고 새 JPEG를 제공하지 않는다. 원영상을 저장하지 않는다.
- **모두 끄기** 또는 마지막 설정 탭의 상태 조회가3초간 끊기면 구독·캐시를 해제한다. 미리보기는 구동 상태·ARM·STOP을 바꾸지 않는다. 주행 중에도 선택할 수 있으나 Jetson 실주행의 추가 부하는 아직 측정하지 않았다.

[r434 대표](../../artifacts/validation/2026-11-13/operator_preview_r434_v1/RESULT.md)는 WSL의 실제 ROS/HTTP에서 native10Hz·RGB20Hz 합성 입력, JPEG 약2Hz, 종류별 대기1개, OFF/만료 구독0과 인증 경계를 확인했다. 실물 광학/거리 정확도나 브라우저 시각 검증을 대신하지 않는다.

## 1. 적용 범위와 주의

- 대상 노드는 `local_pursuit`의 `race_control_mode=b3_candidate`, `vision_execution=process`로 명시 선택한 경우다. 공식 출발 검출은 별도 `start_signal_mode=official_4red_candidate` 매개변수이며 두 선택은 서로를 자동 활성화하지 않는다. 현재 launch 기본값은 유지한다.
- **상위 정책 정지 ≠ MCU STOP ≠ 실제 차량 정지.** CLI `race_operator.py`는 `/race/command`만 발행한다. 웹 RaceOps는 경기 요청과 MCU Trigger 서비스 호출을 연결한다. 두 도구 모두 `/drive`·`/drive/safe`에 직접 발행하지 않는다. 통신이나 제어기가 고장 난 상황의 독립 비상 정지 수단으로 쓰지 않는다.
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


## 개발 전용 짧은 조작 준비 — 11/12

[조작 도구 r169](../../artifacts/validation/2026-11-12/development_jog_r169_v1/README.md)는 자율 제어기를 끈 별도 시험용이다. `python3 scripts/development_jog.py forward --speed .15 --seconds .5 --speed-limit .2`는 미리보기이고 `--send`를 붙여야 발행한다. `left/right`는 전진하며±0.2rad 조향한다. 자동 ARM/CLEAR/RESET·후진·WASD는 없으며 기존 독립 보호층을 거친다. r408 보완 뒤에는 출발 전 기존 virtual 또는 runtime host 형식의 새 ARMED·0목표·유효 feedback과 단독 발행 조건을 확인한다. 종료 때0목표는 요청 조향을 유지하고, 별도 명시 STOP과 같은 boot/시계의 새 `ESTOP_LATCHED` 세대를 확인한다. 이는 실제 차체 정지의 측정은 아니다. 별도 스크립트라 자율 모드의 입력 루프에 추가하지 않았다.

노트북의 실제 ROS/보호층과 가짜 MCU endpoint6사례까지 확인했다. 실제 차량 조작·방향·제동·전체MCU전송은 미검증이다. r162 자율 시험 상한과 수동 도구의 상한은 각 모드에서 명시하며 하나의 화면에서 공유하는 전환은 아직 없다. 당시 모터 시험 중의 Jetson 접근 중단 요청은 이후 사용자 재허용으로 해제됐다. 현재 장비 소유권과 실행 범위는 [현재 현황](../PROJECT_CONTEXT.md)의 최신 작업 지시를 따르며, 접속 허용을 실물 구동 승인으로 확대하지 않는다.
