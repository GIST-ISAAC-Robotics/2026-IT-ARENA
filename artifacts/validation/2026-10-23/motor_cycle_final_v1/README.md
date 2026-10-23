# 출력 없는 제어 주기 결합 최종본 — 전체 PC 회귀와 변이 검사

**4,122통과·실패/건너뜀 0**, 149.51 s, 검사 exit 0이다. 실행 전후 `src/scripts/tests/config` 해시 차이 0 B다. 이어 같은 실행 스크립트에서 제어 주기 변이 검사를 수행했으며 정상 대조 통과·결함 변이 **8/8 검출**이다.

이 소스는 v1/v2 뒤 최종 검토에서 **중간 ARMED를 제어 주기가 직접 관측해야 한다는 조건을 제거한 판본**이다. 같은 소유 작업의 신뢰된 receiver가 ARM과 DRIVE를 두 tick 사이에 모두 처리해도, 후보는 세대 변경에서 적분을 초기화한 뒤 현재 목표를 계산한다. 첫 tick부터 ACTIVE인 수신기에 새 계산기를 붙이는 경우·고장 자동 재개·오래된 입력은 계속 거절한다. Gazebo `AppliedGuard`의 비신뢰 메시지 관문은 바꾸지 않았다.

- 변이체 8종: 출력 준비 검사 제거·receiver tick 제거·고장 고정 제거·세대 초기화 제거·고정 dt·피드백 무시·고장 적분 초기화 제거·0목표 초기화 제거. 모두 컴파일 성공 뒤 제어 주기 사례에서만 실패했다(코어/수신 연결부/TX/벤치/PI 공유 검사는 통과). [요약](mutants/summary.json)
- v2의 `no_arm_observation`(새 ARM 관측 누락) 변이는 그 조건 자체를 제거했으므로 `no_generation_reset`으로 대체했다. v2의 8/8 결과를 이 판본의 검증으로 소급하지 않는다.
- 여섯 C++ 대조 바이너리를 ASan/UBSan으로 새로 빌드해 명시 환경 변수로 pytest에 제공했다. 컴파일 로그는 비어 있다.

PC 합성 사건 검사다. 실제 UART/타이머/ISR·출력 차단·독립 watchdog·MCU 실행 시간·실물 제동은 검증하지 않았다. 운영 ROS/Gazebo 모터 루프·센서/물성/보호 한도는 바꾸지 않았다. `firmware_verified=false`, `ready_to_drive=false`.

[실행 스크립트](run_checks.sh), [전체 로그](full_tests.log), [변이 로그](mutants.log), [종료값](checks_exit.txt), [해시 차이](source_hash_diff.txt), 소스/바이너리 해시와 원시 스냅샷은 같은 폴더에 보관한다. 이 README는 실행 뒤 GPT 주관 작업에서 원시 산출물을 대조해 작성했으며, 실행은 GPT-6 Luna가 맡았다.
