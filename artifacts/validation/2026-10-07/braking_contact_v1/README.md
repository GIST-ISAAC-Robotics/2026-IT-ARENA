# 단일 모터축 설정 확인·제동 접촉 분리 시험

2026-10-07. 기술적 해석과 남은 범위는 [G1-b 보고서](../../../../docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md#107-후속--단일-엔코더-재확인과-접촉-계측)에 모은다. 이 폴더는 재현 입력·요약·실패 근거를 보존한다.

- `encoder_runtime_audit.json`: 이전 실행의 설정 ZIP/실제 모델/제어기 구독/3,483개 MCU 상태 대조. 28카운트·15:1·65 mm·모터축 단일 채널 통과.
- `r0_s0/`: 첫 접촉 계측 R0. 실제 STOP/정상 종료는 통과했지만 actor 위치의 종료 coverage가 부족해 실행 무효. 원본 판정을 유지한다.
- `fixed_official/`: 고정 명령 시험기 v1. 명령 타임스탬프 누락으로 보호층이 출발을 막았다. 중단 후 정상 종료했으며 제동 성능 결과에서 제외한다.
- `fixed_official_v2/`, `fixed_flat_v2/`: 타임스탬프를 고친 동일 시험기/소스로 분할 노면과 단일 판 대조. 요청은 0→1 m/s 1초, 1초 유지, 0.5초 감속, 이후 0. `/drive`→LiDAR 보호층→`/drive/safe`→가상 MCU 유지. 두 실행 모두 기능·STOP·정상 종료 통과.
- `r0_s0_capture_v2/`: 종료 자료 직렬화 전에 모든 transport 관측을 함께 닫도록 수정한 후속 R0. 원시 행을 사후 잘라 판정을 바꾸는 방식이 아니다.
- `*_analysis.json`: 바퀴별 접촉 하중과 제동 초반 0.5초의 속도 차이. 힘은 각 바퀴에 작용하는 world 좌표 값이며 누락/오래된 관측은 null로 남긴다. 사건형 메시지 미수신을 하중 0으로 바꾸지 않는다.
- `final_audit.json`: 초기 네 실행의 소스 ZIP/원시 SHA/평가 센서 외 차량 동일성 감사. `final_audit_v2.json`은 후속 R0까지 포함한 마감 감사다. 감사 통과는 실패한 주행 판정의 변경이 아니다.

## 실행

WSL에서 ROS Jazzy와 `install/local_setup.bash`를 불러온 뒤, `src/arena_autonomy`, `src/arena_vehicle_interface`, `scripts`를 기존 `PYTHONPATH` 앞에 추가한다. 출력은 반드시 새 경로를 사용한다.

```bash
python3 -B scripts/validate_g1_traffic.py --case stopped_center --seed 0 --mode r0 --record-braking-diagnostics --record-wheel-contacts --output artifacts/validation/<새 경로>
python3 -B scripts/probe_braking_contact.py --surface official --output artifacts/validation/<새 경로>
python3 -B scripts/probe_braking_contact.py --surface flat --output artifacts/validation/<새 경로>
python3 -B scripts/analyze_wheel_contacts.py artifacts/validation/<실행 경로> --output artifacts/validation/<새 분석 파일>.json
```

R0 분석은 `--onset`에 `analyze_g1b_braking.py`가 찾은 첫 제동 시각을 명시한다. 고정 명령 시험의 시작 시각은 보고서에서 읽는다. 공식/팀 기본 월드 파일은 수정하지 않고, `flat`은 평가 사본의 도로/잔디/지면 충돌체만 높이 3 mm 단일 판으로 교체한다. 기본 트랙 대체안이 아니다.

고빈도 접촉·센서·자세·MCU 기록은 로컬/Git 제외. 보고서와 소스 ZIP, 실행 모델/월드, 원시 SHA는 보존한다. 이번 환경의 패키지는 `ros-jazzy-gz-sim-vendor 0.0.10-1noble.20260604.111001`, `ros-jazzy-gz-physics-vendor 0.0.7-1noble.20260604.105412`다.

## 접촉력 해석 근거

Gazebo의 [ExtraContactData 규약](https://github.com/gazebosim/gz-physics/blob/gz-physics7/include/gz/physics/GetContacts.hh)은 첫 충돌체에 작용하는 world 좌표 힘을 제공한다. [접촉 메시지 연결 변경](https://github.com/gazebosim/gz-sim/pull/2050)은 두 번째 충돌체에 반대 힘을 기록한다. 2026-10-07 원문과 실제 메시지의 `body1Wrench`/`body2Wrench`를 대조했다. 코드에서는 항상 body1을 취하지 않고 바퀴가 어느 충돌체인지 확인한다. 초기 낙하 충격은 제동 분석 구간에 포함하지 않는다.
