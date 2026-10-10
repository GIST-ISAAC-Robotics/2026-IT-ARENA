# 동일 요청·초기 자세의 제동/노면 대조

2026-10-08 실행 전 등록. [왼쪽 가림 실패](../../2026-10-07/g1b_side_v1/README.md)의 감속 중 휠–차체 차이를 분리한다. 제어기·차량 물성·기본/공식 월드를 수정하지 않는다. 주행 성공 시험이 아니라 **원인 분리용 고정 요청 시험**이다.

## 조건과 중단 범위

- `command_profile.json`: 이전 trace의 /drive 속도 표본과 마지막 autonomy 조향 상태를 고정한 32점·3.5초 입력이다. `make_profile.py`가 이전 감사의 원시 trace SHA를 대조하고 만들었다. 원래 50 Hz 요청 전체가 아니라 약 10 Hz 속도/약 5 Hz 상태 조향을 유지형으로 재구성한 입력이며 **실패 주행의 정확한 재생으로 부르지 않는다**.
- 두 조건 모두 동일 프로필·슬롯 3 시작 자세, 기존 2 kg·65 mm·15:1·모터축 28카운트·같은 PI/토크·순차 C1 10 Hz·물리 step을 유지한다. 상대차는 제외하고 경로 제어기 대신 고정 요청만 발행한다. 시각 외의 차체 정답/센서 값으로 요청을 바꾸지 않는다.
- A는 현행 분할 노면. B는 실행 사본에서만 바닥/도로/잔디 충돌체를 상단 z=3 mm·두께 100 mm의 단일 판으로 바꾼다. 표시 형상과 모든 비노면 충돌체를 유지한다. B를 기본 코스로 채택하는 작업이 아니다.
- `/drive` → 독립 LiDAR 보호층 → `/drive/safe` → 가상 MCU를 유지한다. 요청·safe 명령·MCU가 보고한 목표와 구동 진단을 보존해 보호 개입/전달 차이를 확인한다. **같은 요청이 실제 적용 명령까지 완전히 같다는 뜻은 아니다.**
- 3.5초 프로필 종료 후 1초 동안 0명령, 명시 STOP 후 1초 관측한다. 발행/구동 진단 공백 50 ms 이하·처음/끝 coverage·0.45초 이상 최종 정지 관측·MCU STOP·정상 종료·소스 불변을 유효성 조건으로 한다. 큰 속도 불일치 여부와 시험 유효성은 별도로 보고한다. 자료가 무효이거나 STOP/종료가 실패하면 다음 실행을 보류한다.
- 기본 A→B 각 1회. 유효한 두 결과의 차이가 크면 같은 소스/입력으로 B→A 각 1회 추가해 실행 순서/위상에 따른 재현성을 확인할 수 있다. 이번 범위는 최대 네 유효 실행이며, 차이가 없거나 원인 단정이 불가능하더라도 판정선을 바꾸지 않는다. 오른쪽 가림·G2·고속화·물성 튜닝은 이번 범위에 넣지 않는다.

## 평가

프로필 상대 시각 **1.75~2.85초**에서 최대 평균 휠–차체 차이·좌우 휠 차이·역회전·원천 토크·접촉 하중을 비교한다. 전체 구간 최대도 함께 기록한다. 실제 적용 명령 차이와 초기 위치 차이가 있으면 이를 노면 차이와 분리해서 해석한다. 0.35 m/s는 기존 조사 전환 기준이지 실물 허용오차가 아니다. 접촉 메시지 미수신은 하중 0이 아니다. 기존 실패 기록을 새 결과로 덮지 않는다.

## 실행

WSL 저장소 루트에서 `bash artifacts/validation/2026-10-08/braking_ab_v1/run_case.sh official`과 `.../run_case.sh flat`을 순차 실행한다. 선택적 역순 반복 이름은 각각 `flat_repeat`, `official_repeat`다. 스크립트는 기존 경로/로그에 덮어쓰지 않는다. 실행 소스 ZIP/프로필/모델/월드/판정/원시 SHA는 보존하고 고빈도 원시 파일은 로컬/Git 제외한다.

사전 `run_checks.sh` 결과는 `pytest_preflight.log`의 **74개 통과**다. 실행 결과 및 사후 검증과 구분한다.

## 마감 결과

A→B→B→A 네 번 모두 입력·명시 STOP·안정 정지·정상 종료·소스 불변을 통과했다. 두 쌍 모두 동일 차량·요청 프로필·시작 설정·비노면 월드가 확인됐으며 20 ms 정렬 275표본에서 요청/safe/MCU 목표의 속도·조향 차이가 0이었다. 분할 노면은 평균 바퀴–차체 차이 0.774775 m/s, 단일 평면은 0.707519/0.707520 m/s였다. 단일 평면에서는 좌우 바퀴 표면 속도 차이가 1.537450에서 약 0.03417 m/s로 감소했다. **두 노면 모두 큰 불일치가 재현됐으므로 시험 유효성 통과를 슬립 해결로 해석하지 않는다.**

| 근거 | 내용 |
|---|---|
| [첫 A/B 분석](ab_analysis.json), [역순 반복 분석](repeat_analysis.json) | 입력·소스·노면 외 동일성, 명령 표본, 운동/접촉 수치, 원시 SHA |
| [마감 감사](final_audit.json), [생성기](finalize_analysis.py) | 네 실행 현재 소스·기록 불변, 0목표에서의 모터 기계 출력 표본, 구동 매개변수 |
| [식 특성 검사](zero_crossing_probe.cpp), [출력](zero_crossing_probe.json) | 강제 부호 전환에서 PI/40 ms 응답의 일시 양의 출력. 차량 동역학/실물 시험 아님 |
| [마감 회귀](pytest_final.log), [회귀 명령](run_final_checks.sh) | 608개 통과; 사전 74개와 합산하지 않음 |
| [상세 해석·다음 작업](../../../../docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md#108-후속--동일-명령의-제동노면-대조) | 노면 영향과 공통 구동 특성 분리, 0명령 전 잠김·실물 미검증 한계 |

각 실행의 `report.json`·`source_snapshot.zip`·`runtime_vehicle.sdf`·`runtime_world.sdf`·시각 메시를 보존한다. `braking_diagnostics.jsonl`·`wheel_contacts.jsonl`·`requested_commands.jsonl`·`safe_commands.jsonl`은 고빈도 원시 자료로 로컬/Git 제외다. 추가 실행이나 자료 덮어쓰기를 하지 않는다. 커밋/푸시·Jetson·전원 변경은 하지 않았다.

사후 분석 재현은 WSL 저장소 루트에서 아래와 같다. 출력은 새 경로를 사용하며 기존 JSON을 덮지 않는다. `finalize_analysis.py` 역시 최초 마감 JSON이 존재하면 쓰기를 거부한다.

```bash
python3 -B scripts/analyze_braking_ab.py \
  artifacts/validation/2026-10-08/braking_ab_v1/official \
  artifacts/validation/2026-10-08/braking_ab_v1/flat --output /tmp/braking_ab_new_analysis.json
g++ -std=c++17 -I src/arena_gazebo/include \
  artifacts/validation/2026-10-08/braking_ab_v1/zero_crossing_probe.cpp \
  -o /tmp/arena_zero_crossing_20261008
/tmp/arena_zero_crossing_20261008
```
