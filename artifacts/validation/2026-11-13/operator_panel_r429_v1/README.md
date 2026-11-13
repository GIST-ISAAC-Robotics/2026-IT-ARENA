# r429 운영 패널 stage

2026-11-13. 공통 원본을 바꾸지 않는 구현 사본이다. `stage/`의 저장소 상대 경로 8개만 이관 대상이며 `changes.json`·`changes.patch`와 원본 SHA를 대조한다. `test_overlay/`, `pydeps/`, preview·시험 산출물은 이관 대상이 아니다.

## 구현 범위

- B3에서 속도 상한을 항상 구성한다. 기본 초기 상한은 기존 `max_speed_mps`이고, 옛 `development_speed_limit_enabled=true`의 명시 초기값은 유지한다. 기본 초기 상한은 가속/출발 목표가 아니다. 정지 및 MCU 비ARM 상태 변경 조건은 유지한다.
- m/s와 km/h를 양방향 편집하고, 주기 상태 수신은 미적용 편집값을 덮지 않는다. 서버 적용값과 편집값, 단발 유효 상한을 구분한다. 네 속도 읽기 행도 두 단위로 표시한다.
- 수동 조작은 `operation_mode:=manual`의 별도 실행 구성이다. 자율 구성에서는 수동 준비·실행을 거절하고 수동 graph 폴링·자식·`/drive` publisher를 생성하지 않는다.
- 명시 수동 준비 버튼은 독립 guard/host·단독 graph·현재 MCU를 확인한 뒤 CLEAR/ARM한다. 기동만으로 ARM하지 않는다. 이미 있는 자율 노드를 자동 종료하지 않으며, 자율 노드가 0명령만 내더라도 publisher가 있으면 수동 준비/실행을 거절한다.
- 실행은 기존 `development_jog` 구현을 패키지 모듈로 옮겨 재사용한다. CLI는 얇은 호환 진입점이다. 목표속도·0.05–2초·중앙 등가 바퀴각을 조정한다. +좌/−우이며 서보축 각도가 아니다. 기존 단발 속도0.2 m/s와 기본 조향±0.2 rad는 유지한다. 설정 상한보다 큰 입력은 거절하고 조용히 줄여 실행하지 않는다.
- `v×t`는 등속 가정 예상 주행거리다. 실제 이동·정지거리나 곡선 순변위가 아니다.
- 자식은 `/drive`에만 발행하고 기존 guard를 거친다. 종료/STOP/parent stdin EOF·heartbeat 공백/MCU 변경/graph 상실은 기존 0목표+STOP 회수를 사용한다. parent도 STOP을 독립 요청하며 자식 종료를 회수한다. 정상 종료 뒤 다시 준비해야 다음 단발을 실행할 수 있다. MCU ACK/상태는 실제 차체 정지 증명이 아니다.

## 운영 전환

자율→수동은 UI의 허가 전환 버튼이 아니다. 운영자가 STOP·정지 확인 후 기존 planner를 정상 종료하고 **guard와 host만 유지**하여 별도 수동 패널을 실행한다. 기존 두 `/drive` publisher를 공존시키거나 planner PID를 강제로 종료하는 도구는 추가하지 않았다. 다시 자율 구성으로 바꿀 때는 수동 STOP·자식 회수·수동 패널 종료 뒤 기존 B3 진입 템플릿을 따른다. 이 절차는 실제 차량으로 검증하지 않았다.

설치된 ROS overlay와 공통 소스가 준비된 뒤 아래처럼 실행한다. guard/host는 [r415 진입 템플릿](../runtime_entry_audit_r415_v1/REVIEW.md#운영-진입-명령-3개)의 해당 두 명령을 재사용하고 **계획 노드 명령은 실행하지 않는다.** C1/IMU/엔코더 등 guard가 요구하는 입력은 같은 계약으로 제공해야 한다.

```bash
ros2 run arena_autonomy race_ops --ros-args \
  -p operation_mode:=manual -p vehicle_id:=car1 \
  -p host:=0.0.0.0 -p port:=8765 \
  -p token_path:=/path/token.txt -p log_path:=/path/manual_ops.jsonl \
  -p manual_output_root:=/path/manual_pulses \
  -p manual_maximum_mps:=1.5 -p manual_steering_limit_rad:=0.2
```

예제1.5 m/s는 r401 host/차량 운영 설정의 상한이다. 설치 프로필이 더 낮으면 동일하게 낮춘다. 실제 단발은 항상 `min(서버 적용 상한, 0.2 m/s)` 이내다. ±0.2 rad는 기존 jog의 제한 범위를 유지한 값이며 장착 확인값이 아니다. `manual_steering_limit_rad`는 설치된 차량/host/guard 중 가장 낮은 중앙 등가각 한계 이하여야 하고 이 구현의 최종 CAD 상한0.4578436017036438 rad를 넘길 수 없다. 자동 장착 보정·자동 펌웨어 변경은 없다.

기존 host의 출력 wire 차단은 패널이 해제하지 않는다. 사람이 설치/출력 허가를 완료한 뒤 사용하는 host 옵션과 실제 물리 STOP 확인은 기존 실물 이관 절차를 따른다.

## 검증 진입점

Windows 순수검사는 stage 수정 파일을 공통 패키지 사본 `test_overlay/arena_autonomy/` 위에 덮은 환경에서 수행했다. 기존 jog·운영·B3/수동출발140검사와 새 경계19검사를 실행했다. `check_page.js panel_preview.html`은 DOM 합성 UI 검사이며 실제 브라우저 픽셀/네트워크 검증이 아니다.

`ros_panel_probe.py --output <새 결과 경로>`는 실제 ROS·HTTP·LidarSafety와 가짜 MCU 서비스/상태, 합성 scan의 제한 대표다. 실제 장치·모터·서보·Gazebo·flash를 쓰지 않는다. 공유 WSL은 root가 자원을 배정한 뒤 한 번 실행한다. 결과는 `RESULT.md`를 따른다.
