# M3 유한 실행 후보 — GPT 인계 검토

Claude 사용량 경고 후 GPT가 설계·수정·검토를, GPT-6 Luna가 고정 실행과 결과 회수를 맡았다. 원본은 [10/24 run3](../../2026-10-24/m3_transition_v2/run3_execution_replay/)에 보존했다. 첫 제한 Gazebo 주행은 진행·측방 간격 실패다. 실차 시험은 없다.

| 실행 | 목적 | 확인 결과 |
|---|---|---|
| `run1_reproduction` | 수정 전 두 독립 반례 | 2개 재현. 안전성 통과가 아님 |
| `run2_guarded_tests` | 수정 후 관련 회귀 | `PYTHONPATH`의 scripts 누락으로 수집 오류 2건, exit 2. 실패 보존 |
| `run3_guarded_tests` | 실행 경로를 수정한 관련 회귀 | 94통과, 소스 불변, 잔존 0 |
| `run4_guarded_replay` | 동일 저장 입력·기록/자기 출력 조향 대조 | 두 실행 exit 0, 소스 불변, 잔존 0. 모두 개루프 |
| `run5_profile` | 고정 G1 입력의 비용 분해 | 반복 경로 투영·기하 탈락 뒤 추가 관측 비용 확인. 시간 통과 판정 아님 |
| `run6_optimized` | 판단 내 동일 투영 재사용·기하 조기 거절 | 170회귀·저장 4,920명령/선택/이유 동일, 활성 p95 약 54–63 ms |
| `run7_integration_tests` | 선택형 노드·기록·평가 연결 | 181통과·1실패. 빈 CLI 문자열이 ROS 파서에서 거절된 시험 오류 보존 |
| `run8_integration_build` | 시험 인자 수정·설치본 확인 | 9통과·2패키지 빌드 성공·새 셸 import/설치 launch 해시 일치 |
| `run9_preflight_contracts` | 기존 출처 사슬과 새 변경 연결 | 68통과·소스/입력 불변·잔존 0 |
| `run9_closed_loop_left` | 사전 등록한 왼쪽 옆차 제한 주행 | 0.240 m·옆차 최소 약 69.9 mm, 진행·간격 실패. STOP·정상 종료·소스 불변. 판단 636/636 동일 재생 산출물. 중복 재생 호출 거절 이력 보존 |
| `run10_command_selection` | 실제 명령 검사 뒤 후보 재선택 회귀 | 47통과·1실패. 새 테스트의 선행 세대/시각 구성 오류, 비교 미실행 |
| `run11_command_selection` | 테스트 수정·OFF/ON 저장 입력 비교 | 48통과·OFF 636/636 동일·ON 양수 161→369. 복원+장면+판단 p95 약 77–79 ms |
| `run12_selection_integration` | 선택형 연결·평가 표식 수정·출처 검사·빌드 | 185통과·2패키지 빌드·설치본/소스 일치·전후 불변·잔존 0 |
| `run13_selection_closed_loop` | [사전 등록](COMMAND_SELECTION_DRIVE_PLAN.md) fallback ON 왼쪽 옆차 1회(Claude Haiku 5.5 보조 실행) | 0.720 m·옆차 최소 69.8 mm로 진행·최종 앞차 간격 실패. `run_valid=true`, STOP·밀집 정지·정상 종료·노면 이탈 0·94파일 불변·잔존 0·재생 637/637 |
| `run13_analysis` | 읽기 전용 분해와 최소 반례 | 전 후보 거절 70판단의 미지지 질의가 모두 후방 마스크 경계 꼬리 띠(차체 밖 ≤0.37 mm). 빈 직선로도 0.01 rad 이상 거절. [분해](run13_analysis/README.md) |
| `run14_baseline_release` | 기본 false `release_to_baseline` 연결·새 출처 계약 검사 | 201통과·2패키지 빌드·설치본 일치·불변·잔존 0. OFF 637/636판단·4기록 4,920행 동일, ON 반사실 해제 6.32/6.12 s 1회씩 |
| `run15_release_gate` | 부재 기반 해제 반례와 Claude 인계 | run14안 반려. 탈락/가림/부분 반환 반례와 미완성 수정 상태 보존 |
| `run16_release_gate_review` | GPT 관측 관문 검토·수정, Luna 고정 검사 | 빈 기록·세대/시각 단절 반례 3건 재현 후 수정. 관련 139검사·OFF 637/636 동일, 새 ON 해제 7.02/6.92 s. 고정 입력이며 주행 개선 증거는 아님 |
| `run17_observed_release_closed_loop` | [관측 기반 복귀](OBSERVED_RELEASE_DRIVE_PLAN.md) 왼쪽 1회 | 3.860 m로 진행 통과, 최소 측방 69.8 mm로 전체 실패. STOP·정상 종료·노면 이탈 0·소스 불변·잔존 0·재생 648/648. 복귀 후 sampled fresh 양수 86판단 모두 옆차가 앞쪽에서 간격 상한을 결정 |
| `run18_front_target_review` | 원래 지정 앞차와 실제 직선 전방 actor 분리 | 77검사. run17 전방 actor 간격 0.720495 m, run13 3.871783 m. 사후 보조 평가만 추가하며 원본 실패·입력 해시 보존 |
| `run19_stop_restart` | 기존 정지·재개와 잔디 쪽 옆차 결합 1회 | 정지 3.237 s·재개 0.540 s·재개 후 1.220 m 통과, 최종 간격 1.255 m로 실패. STOP·정상 종료·소스 불변·잔존 0·노면 이탈 0·재생 791/791 |
| `run20_followup_audit` | run19 읽기 전용 분해·전방 대상·오른쪽 기하 사전 검사 | 앞차 간격 실패 확인, 오른쪽 명목 간격 70.023 mm·actor 도로 경로 검사 통과. 새 주행 판정 아님 |
| `run21_right_encounter` | 오른쪽 도로 내 피추월 1회 | 진행 3.860 m·복귀 7.12 s, 최소 측방 69.8 mm로 실패. STOP·정상 종료·노면 이탈 0·재생 649/649. 실제 전방 actor 간격 0.717079 m |
| `run22_release_region_audit` | run19 미확인 칸의 관측 원인 | 출력 791/791 동일·입력 불변. 미확인 칸 대부분 후방 마스크로 유지 |
| `run23_body_hypothesis` | 한 강체라는 조건부 가능 영역 비교 | 출력 791/791 동일. 가능 칸 295→156, 마스크 전용 61→2. 서로 다른 물체를 묶는 반례 때문에 제어 미채택 |
| `run24_first_curve` | 첫 U자 코너 앞차 추종 1회 | 13.018 m·180.506°·최소 간격 0.531 m로 등록 기준 통과. 측방 촉발 0, 재생 1,790/1,790·STOP·정상 종료·노면 이탈 0·소스 불변·잔존 0 |
| `run25_center_hold_compare` | 10/25 오전 Claude 재개: 출발 상태 해시, run19 원인 분해, 작은 중앙 보정 생략 저장 입력 비교 | 기록 OFF 4기록 전체 동일. run19 자기 조향 반사실 전 후보 거절 4→1, run17/21/24 출력 차이 0, 관문·위협 차이 0. [요약](run25_center_hold_compare/README.md) |
| `run26_small_center_integration` | 기본 false 옵션 연결·새 출처 계약·관련 회귀·빌드 | 281통과·2패키지 빌드·설치본 일치·run13/run9 OFF 재생 동일·불변·잔존 0. [요약](run26_small_center_integration/README.md) |
| `run27_small_center_stop_restart` | [사전 등록](SMALL_CENTER_DRIVE_PLAN.md) 정지·재개 1회(Claude Haiku 5.5 보조 실행) | **등록 기준 통과**: 최종 간격 0.623 m·재개 후 1.820 m. STOP·밀집 정지·정상 종료·소스 불변·잔존 0·재생 797/797·노면 이탈 0 |
| `run28_small_center_analysis` | run19/run27 읽기 전용 대조 | 위협 소멸 뒤 전 후보 거절 4→0, 앞차 이동 중 정지 1.69→0.49 s. 복귀 관문 미해제는 같음. [요약](run28_small_center_analysis/README.md) |
| `run29_corner_unreleased` | run24 실제 코너 입력에 해제되지 않은 M3 상태 주입(개루프) | 이동 중 코너는 대부분 통과(양수 576/585 of 625), 진입 전이에서 1.6–2.6 s 재개 대기·정지 중 곡선 후보 거의 미지지. [요약](run29_corner_unreleased/README.md) |
| `run30_drift_transition` | 합성 직선 22 mm 초과 전이 | 중앙·(방향 오차 시)유지 후보 계획 미지지, 직진만 남아 편차 증가 |
| `run31_curve_stop_probe` | 합성 곡선 정지/이동 | 이동 사례도 실패해 대표성 낮음, 보조 기록 |
| `run32_corner_after_threat` | 정지·재개 뒤 첫 코너 연결 1회, GPT-6 Luna 실행 | **실행 유효·주행 실패**: 초기 15 s 정지·재개 통과, 전체 7.279 m·코너 직전 11.572 s 정지·최종 간격 2.798 m. STOP·정상 종료·노면 이탈 0·소스 불변·잔존 0·재생 2,036/2,036. [결과](run32_corner_after_threat/README.md), [당시 인계](run32_corner_after_threat/HANDOFF.md) |
| `run33`–`run34` | 직진 확약 구간 노면 검사 연결·회귀/저장 입력 대조 | run33 새 테스트 경계 1실패 보존, run34 300회귀·OFF 5,920판단 동일. ON run32 양수 1,009→1,645(개루프) |
| `run35_committed_surface_corner` | 같은 코너 조건에서 새 노면 범위 옵션 1회 | 7.659 m 진행, 초기 정지/재개 통과·코너 실패. STOP/종료/노면/출처·재생 2,041/2,041 확인 |
| `run36`–`run37` | 같은 측면 두 차량의 기록 교체 반례 | 첫 이력 오류 보존. 유효 합성은 숨은 A가 남아도 새 B 때문에 4.7 s에 잘못 해제 |
| `run38`·`run40`–`run42` | 미연결 기록/합집합/양의 갱신 비교 | 숨은 A 보존과 정상 단독 복귀·계산 비용을 비교. 그대로 채택하지 않음 |
| `run39_corner_progress` | run32/35 저장 주행 구간 분석 | run35는 짧은 주행/대기를 반복하나 코너 회전은 못 함 |
| `run43`–`run45` | 위협 기록 보존 구현의 전후 회귀 | 전 좌우 반례 2실패, run44 ROS 경로 오류, 수정 후 관련 307통과 |
| `run46_retained_threats_replay` | 기록 보존 전후 다섯 저장 주행 대조 | 이전판 4,772판단 재현. 좌우 복귀 소실·최대 약 1.3 s 비용, 입력/소스 불변 |
| `run47`–`run49` | 비용 분해·같은 거리/광선 계산 배치화 | 315회귀·4,772판단 전체 동일. 최대 약 1.34→0.56 s, 여전히 실시간 미달 |
| `run50_curve_stop_preflight` | 다음 M2 코너 내부 정지 일정/경로 검사 | actor 정지 방향 84.46°, 경로 1,226표본 통과. HOLD 평가의 직선/자세누락 거절 확인 |
| `run51_curve_stop_restart` | 코너 내부 정지·재개 1회 | HOLD 3.640 s·재개 0.546 s·진행 13.018 m·최소 간격 0.348 m 통과. 측방 촉발 0, STOP/종료/노면/불변·재생 2,046/2,046. [결과](run51_curve_stop_restart/README.md) |
| `run52_queue_preflight` | 두 앞차 일정/평가기 | 94회귀·최소 actor 간 표본 간격 약 0.450 m·직선 끝 위치 제한 확인. 최초 잘못된 WSL 배포판명 시도 보존 |
| `run53_queue_staggered` | 두 앞차의 서로 다른 재출발 | 먼 차만 출발한 동안 자차 대기, 재개 0.633 s·최소 간격 0.363 m·797/797. 외곽 해시 CRLF 실패와 별도 74소스 ZIP/전후 일치 구분. [결과](run53_queue_staggered/README.md) |
| `run54_faster_lead_preflight` | 앞차 초기 0.50 m/s 조건 | 설정 차이 한 항목·411개 외곽 도로/벽 표본 통과. Python 성공과 bash 마지막 CRLF 종료 오류를 분리 보존 |
| `run55_faster_lead_stop` | 초기 0.50 m/s 뒤 정지 | HOLD 2.930 s·재개 0.744 s·최소 간격 0.361 m·789/789·115파일 불변. [결과](run55_faster_lead_stop/README.md) |
| `run56_m3_sensor_stress_preflight` | 기존 주입을 M3 명시 옵션에 연결 | CLI 수용/거절·주입/출처 관련 74회귀 통과. 주행 제어기·기본값 미변경 |
| `run57_faster_lead_sensor_stress` | 초기 0.50 m/s+오차/지연 | 최소 간격 0.412 m·재개 0.708 s·794/794·120파일 불변. 196취득/수신·±3 cm·추가32–70 ms 주입 확인. [결과](run57_faster_lead_sensor_stress/README.md) |
| `run58_narrow_lead_preflight` | 좁고 중심에서 벗어난 앞차 | SDF 20×10×12 cm·횡위치 +8 cm·측정면 교차·412개 경로 표본 확인 |
| `run59_narrow_offset_lead` | 좁은 앞차+오차/지연 | 최소 간격 0.407 m·재개 0.718 s·789/789·124파일 불변. [결과](run59_narrow_offset_lead/README.md) |
| `run60_m4_preflight` / `_v2` | B3 운영 연결·빌드 | 첫 출처 목록 1실패 보존, 보완 뒤 89회귀·빌드·설치 확인 |
| `run61_b3_same_race` | 같은 경기 STOP·재ARM | 실제 재출발 0.375 s·3.073 m, 결합 HOLD 0.912 s 미달로 실패. [결과](run61_b3_same_race/README.md) |
| `run62_m4_clock_review` | CLEAR 경계 사후 보완 | 135회귀·입력 불변. 등록 래치는 통과, HOLD 미달 실패 유지. [분석](run62_m4_clock_review/README.md) |
| `run63_m4_new_race_preflight` / `_v2` | 새 경기 신호대기 준비 | 첫 143회귀 뒤 응답/상태 순서 보완, v2도 143통과. [사전 범위](M4_NEW_RACE_PLAN.md) |

사용자 정리 요청으로 run12에서 한 번 마친 뒤, 재개해 run13과 개선 후 run17을 각각 사전 등록한 조건으로 1회씩 실행했다. run9의 `run_valid=false`(평가 표식 누락)는 원본대로 보존한다. 부재만으로 해제하는 [run14 주행 등록안](RELEASE_DRIVE_PLAN.md)은 GPT 검토에서 반려·미실행이다. 관측 관문으로 바꾼 run17의 진행성은 개선됐지만 M3 최소 간격은 실패다. 정지·재개 결합과 오른쪽 피추월도 각각 1회 실행했고 최초 실패를 보존했다. [첫 코너 추종](FIRST_CURVE_DRIVE_PLAN.md) 한 번을 등록 기준 통과로 마치고 사용자 요청에 따라 종료했다. 기준 완화나 같은 실패의 자동 재시도는 하지 않는다. 오전 재개에서는 run19 실패의 다른 원인 가설(작은 중앙 보정의 미세 조향이 다음 주기 전 후보 거절을 부름)을 저장 입력으로 비교하고, GPT 승인 뒤 기본 false 옵션을 켠 정지·재개 1회(run27)로 확인했다. run19의 원래 실패는 그대로 보존한다.

첫 반례는 과거 광선의 거리 방향 여유를 만족해도, 그 장면의 측방 미관측 영역에서 같은 0.5 m/s 상계로 질의점에 도달할 수 있음을 보였다. 수정판은 해당 과거 관측의 재사용을 거절한다. 이는 과거 unknown/masked에서의 도달 검사이며 모든 가림·미래 접근·연속 외곽의 안전 증명이 아니다.

두 번째 반례는 모터축 속도 0과 자이로 회전 0.8 rad/s가 동시에 주어져도 정지 중 조향 정렬을 허용했던 경우다. 수정판은 최근 0.1초의 모터축 0·낮은 자이로와 운동 이력의 공백/노후 관문을 함께 확인한다. 두 센서가 조용해도 미끄러지는 차체가 정지했다는 뜻은 아니다.

`run4_guarded_replay`에서 측방 선택 후 양수 명령은 기록 조향 기준 G1/R0+/정지재개 **10/31/0**, 자기 조향 반사실 **11/6/10**이다. 단독 **2,837/2,837 명령 동일**. 활성 판단 p95는 약 **57–71 ms**여서 50 Hz 운용 성능은 아직 확보하지 못했다. 사후 간격은 원래 차량 궤적의 값이며 후보가 주행한 결과가 아니다. 원래 run3의 양보 양수 결과를 최신 수정판의 성과로 인용하지 않는다.

정확한 실행 명령·종료 코드·로그·소스 스냅샷·해시는 각 run 폴더에 있다. 추가 결과와 설계 결정은 [E3 보고서](../../../../docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md)에 정리한다.

10/25 오전 후속은 run32 한 번으로 마쳤다. 첫 직선 결과의 재현과 코너 실패를 분리했으며 동일 실패를 재실행하지 않았다. 정지 장면의 미지지 질의는 [좌표 감사](run32_corner_after_threat/stationary_queries.json)에, 같은 측면 재관측에 의한 기록 갱신은 [제한 합성 상태 전이](release_reacquisition_review.json)에 남겼다. 후자는 주행 또는 물체 연관의 일반적 안전 증명이 아니다. 상세 원시 입력/재생은 삭제하지 않고 기존 정책대로 Git에서 제외한다.

마지막 `run64_b3_new_race`는 새 경기·재ARM 뒤 신호 부재에서 10.482 s 정지, M4 별도 16조건 통과·원래 교통 실패 보존·89/89 재생·133파일 불변을 확인했다. [결과·한계](run64_b3_new_race/README.md). 사용자 정리 요청에 따라 모든 자체 시험을 정상 종료하고 새 작업을 시작하지 않았다.
