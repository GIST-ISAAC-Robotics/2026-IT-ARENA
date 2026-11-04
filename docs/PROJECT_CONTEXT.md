# 현재 현황과 다음 작업

갱신: **2026-11-03, 기둥 없는 코스의 300초 교통 주행 종료·5랩 미달, 단독 증속은 격리 검증까지만 완료**. 실행 월드는 v2026.11.01 팀 파생본이다. 과거 월드의 주행 실적을 현행 구성의 성적으로 옮기지 않는다.

목표는 **빠른 완주·꾸준한 속도·불필요한 급정거 최소화**, 직선 최고 2 m/s급에서 **10 km/h**로 개선하는 것이다. **필수 기능을 각각 대회에 쓸 수준까지 완성하고, 전체가 그 수준을 갖추면 다시 성능을 높인다.** 단독 속도·다중 차량·운영·실물 준비의 완성도와 경기 영향으로 우선순위를 정하며 고정 시간·분야 순서로 교대하지 않는다. [현행 계획](DEVELOPMENT_PLAN.md)·[사용자 결정 ADR 0028](decisions/0028-speed-continuity-and-final-gearing.md)·[직전 상세 현황 원문](archive/2026-11-02-plan/PROJECT_CONTEXT.before.md)을 구분해 읽는다.

## 현재 구성

| 영역 | 적용 상태 |
|---|---|
| 차체·구동 | 20×15 cm 목표, MB4266-24180 브러시 DC 하나+MD20A+기계식 차동, PGE-213 **모터축 단일 28카운트**, 65 mm 바퀴. **실행 공칭비 8.9706:1**, 이전 15:1 증거 보존. 축거·윤거 135 mm, 바퀴 폭 12 mm, 약 2 kg·무현가 강체·접지는 임시값. [구매 근거](../assets/hardware/procurement/2026-11-01/README.md) |
| 센서·제어 | B0183 RGB·C1 순차 10 Hz·LSM6DSOX·모터축 운동 보정, 관측 벽 기반 지역 Pure Pursuit. C1 95 mm 높이·후방 60° 마스크·카메라 위치는 임시값. [C1·지주·PDB 적층 비교](../artifacts/validation/2026-11-03/lidar_mount_study_v1/README.md)의 저상·전진안은 기구팀 목업 후보, 실행 구성 미적용. [11/4 주행 비교](../artifacts/validation/2026-11-04/lidar_mount_performance_v1/README.md): 지주·뒤 마커 각도를 unknown으로 둔 opt-in JR은 R과 비슷한 랩으로 완주(33.853 s 대 R 34.253 s, 각 1회, 같은 빌드 반복 없음, 이득/비열등 미확정). 앞 지주가 센서 앞인 C는 현행 전방 ±100° 관측 계약에서 출발하지 못했다(정책 재설계 시 가능성·비용 미측정). 지주 반환을 그대로 처리하면 둘 다 출발 실패. 앞 지주–센서 x 40 mm(|y|62·Ø7)는 작동한 잠정 참고값이고 30 mm 변형은 1회 악화. VND018 ESP32-S3·버스 서보. /drive → 독립 LiDAR 보호층 → /drive/safe, 정답 위치·상대차 상태는 평가 전용 |
| 공식 원본 | **v2026.11.01** ZIP을 입력으로 사용. main 17178293029d446fa603f3229556dd865bbddd38 기준 규정. 방지턱 제거·검정 그리드·2×5 신호·ID45 왼쪽/ID0 오른쪽. [원본/차이/한계](reports/OFFICIAL_UPDATE_2026_11_02.md) |
| 실제 실행 월드 | **v2026.11.01 기반 팀 파생본**. 사용자 확인에 따라 코스 안 `tl_post` 시각·충돌만 제거했다. 신호 패널/가로대·광원 기하·마커·그리드 유지, 광원 재질 소등 시작과 팀 결승 표시는 기존 파생이다. [기하 대조·선두 주행](../artifacts/validation/2026-11-03/post_free_facility_v1/README.md)·[프로필](../config/tracks/official_v2026.11.01.yaml). 후방 ID49와 공식 원본 보존 |
| 환경·성능 | WSL2 Jazzy/Harmonic, Jetson Humble. 기본 G1 상한 1 m/s, 고속은 명시 선택 시험 프로필. 오래된 Jetson 기록 재생과 최신 통합 후보의 처리 여유를 구별한다 |
| 운영 | 기본 OFF 후보 `race_ops` 웹 화면, 같은 경기 STOP/RESET/START, 재시작 복원, 사건 내보내기 연결. [대표 흐름 a6](../artifacts/validation/2026-11-02/official_ops_v1/README.md) 시뮬 통과. 공용망·실제 MCU·실제 정지 미검증. [사용법](autonomy/RACE_OPERATOR_RUNBOOK.md)·[설계](autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md) |

## 확보한 주행 결과

| 근거 | 확인한 결과 | 한계 |
|---|---|---|
| [11/3 기둥 없는 교통 5랩 목표](../artifacts/validation/2026-11-03/post_free_race_distance_v1/README.md) | **300초에 229.545 m, 5랩까지 3.619 m 미달.** 183 m 부근 10.573초 정체. 접촉 이벤트 0·최대 두 바퀴 노면 이탈·명시 STOP/정상 종료 | 앞차·옆차의 각본 추적 및 평가 요약 간격도 실패. 동일 시험 반복 전 정체 입력과 실행기 지연을 분리. 독립 5대 경기 아님 |
| [11/3 기둥 없는 선정 고속 구성](../artifacts/validation/2026-11-03/solo_post_free_v1/README.md) | **34.763초·최고 2.394 m/s(8.62 km/h)**, 거리 기준 한 랩 46.633 m 통과·정지 전 총 진행 48.653 m. 접촉/노면 이탈0·STOP/정상 종료 | 단독1회·후열grid3/수평 카메라. 0명령0.44초·10km/h 미달은 남음. 선두 상향/모든 교통/실차 증명 아님 |
| [11/2 현재 반대 벽 고속 후보](../artifacts/validation/2026-11-02/solo_alternative_speed_v1/README.md) | **34.451초·최고 2.398 m/s(8.63 km/h)**. 같은 월드 35.606초보다 1.155초 단축, 접촉/노면 이탈 표본 0·STOP/정상 종료 | 단독 1회·명시 선택 구성. 후반 코너 0명령 0.14→0.46초로 증가. 10 km/h·전체 본선·실차 미검증 |
| [11/2 새 공식 월드 단독](../artifacts/validation/2026-11-02/official_facility_v1/README.md) | **35.606초·차체 최고 2.373 m/s(8.54 km/h)**, 후열 실제 RGB 출발, 0명령 0.14초, 접촉/노면 이탈 표본 0·STOP/정상 종료 | 단독 한 번. 기본 OFF 선택 구성. 전체 본선·실차 검증 아님. 교통 미실행 |
| [11/2 새 월드 대표 교통](../artifacts/validation/2026-11-02/official_traffic_v1/README.md) | 계속 막은 앞차 우회·우회 중 재출발 뒤 합류·근접 끼어들기 세 장면이 경기 목표 평가 통과. 접촉 0, 최소 간격 0.128/0.312/0.206 m, 최대 두 바퀴 잔디, STOP·정상 종료 | 각 1회, 기본 OFF 후보. 원본 판정의 옛 HOLD·차체 도로 안 실패는 보존. 전체 본선·독립 상대 아님 |
| [11/2 경기 운영 흐름](../artifacts/validation/2026-11-02/official_ops_v1/README.md) | 최초 신호 없는 START 거부. 왼쪽 복구, ID30 안 보이는 오른쪽 복구, 프로세스 재시작 후 오른쪽 복구가 START 1회 뒤 시뮬 0.40–0.43초에 허가되고 평가 정답으로 본선 진행. 지름길 표본 0. 준비 중 STOP·연결 상실 취소, 응답 유실 1회 실행 | 단독 1회. 접촉 센서 미포함. [교통 결합](../artifacts/validation/2026-11-02/official_ops_traffic_v3/README.md)에서 재개·우회·복귀 확인. 브라우저 출발 준비·STOP 요청 수락은 확인했으나 자동 STOP이 먼저라 주행 중 버튼 정지 판정은 실패. 재획득 옵션 ON 기준 |
| [11/2 새 월드 전방 교통](../artifacts/validation/2026-11-02/official_forward_v1/README.md) | 끼어들기 장면 통과(접촉 0·잔디 0·STOP 통과). 급정지·재출발 a4는 재개 지연 0.489초·접촉/노면 이탈 0이나 출구 0.061 m 미달·최종 간격 1.204 m로 실패 | 짧은 대표 장면 각 1회. 기본 OFF 관측 이동 후보 ON 기준. 근접 끼어들기·전체 본선·독립 상대 아님 |
| [11/1 단독 최단](../artifacts/validation/2026-11-01/solo_corner_handoff_v1/README.md) | **35.281초·차체 최고 2.440 m/s(8.78 km/h)**. 코너 끝 0명령 1.20→0.26초, 접촉/노면 이탈 표본 0·STOP/정상 종료·1,880판단 재현 | 이전 팀 파생 v10.24 월드. 기본 OFF 선택. 단독 한 번, 개별 수정의 폐루프 손익은 분리하지 않음. 교통에는 연결하지 않음 |
| [별도 최고속도 후보](../artifacts/validation/2026-11-01/solo_ray_join_v1/README.md) | **2.657 m/s(9.56 km/h)** | 랩 38.858초로 더 느림. 10 km/h의 95% 이상은 0.089초. 최단 랩 구성과 다름 |
| [11/2 새 월드 교통 거리 5랩](../artifacts/validation/2026-11-02/official_race_distance_v2/README.md) | **293.576초에 거리 5랩**, 300초 주행·최장 정체3.859초·접촉/노면 표본0·STOP/정상 종료 | 각본 상대2대·1회. 원시 HOLD/최종 간격 실패 및 우측 간격79.6mm의 옛150mm 미달은 보존. 독립5대/모든 교통/실차 아님 |
| [11/1 교통 거리 5랩](../artifacts/validation/2026-11-01/race_distance_v2/README.md) | 294.478초에 거리 기준 5랩, 300초 총 237.784 m, 접촉/벽 표본 0, 최대 두 바퀴 잔디·5 cm 내 진행 최장 1.165초. 명시 STOP 확인 | 녹화기의 실제 시간 한도로 **전체 실행 정상 판정 실패**. 최신 1 mm 후보 포함 아님. 독립 5대 경기 아님 |
| [11/1 코너 피추월](../artifacts/validation/2026-11-01/corner_catchup_v1/README.md) | 20초·12.578 m·출구 통과, 실제 코너 동행 0.40초, 접촉/자차 노면 이탈 표본 0 | 왼쪽 상대 두 바퀴 잔디. 모든 병렬·오른쪽 사각·도로 내 두 차량의 보장 아님 |
| [10/31 1랩/영상](../artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1/README.md)·[10/29 저속 5랩](../artifacts/validation/2026-10-28/multilap_preparation_v1/RESULTS_2026_10_29.md) | 당시 구성의 완주·우회 중 상대 재출발/복귀 기록 | 다른 감속비/설정. 10/31 마지막 요약 0.417초 누락은 별도 실패로 보존. 후속 성공을 소급하지 않음 |

추종/우회 준비 분리·양보 갱신의 원본 실패와 후속 완주는 [대표 코스 원본](../artifacts/validation/2026-11-01/course_compare_v1/README.md)·[전진 관측 양보](../artifacts/validation/2026-11-01/yield_progress_v1/README.md)에 있다. 코너 v2/v3의 미완주·미성립, 0.5초 간격의 19.837 m 실패, 호출되지 않은 후보는 [직전 현황 보관본](archive/2026-11-02-plan/PROJECT_CONTEXT.before.md)과 연결한 결과를 유지한다.

## 주요 미해결 사항

1. **속도:** 선정 단독 구성은 기둥 제거 뒤에도 [34.763초·최고2.394m/s](../artifacts/validation/2026-11-03/solo_post_free_v1/README.md)로 완주했지만 **10km/h 미달**이다. 후반43m 부근0명령0.44초·42–44m 통과3.415초와 고속 목표 지속성이 보강 과제다. [저장 광선 진단](../artifacts/validation/2026-11-02/solo_observation_diagnosis_v1/README.md)에서 긴 직선 무반환이 회전하며 정면 분류로 들어오는 것을 확인했다. 39개 중34개는 평가 기하의 첫 표면이12m 밖이고 나머지는 취득/렌더 근사 한계가 남는다. 과거 기둥이 있던 신호 지점의1m/s 상한은 [반대 벽 고속 허가](../artifacts/validation/2026-11-02/solo_alternative_speed_v1/README.md)로 보완됐으며 현재는 기둥도 제거했다. [기둥 대응 코드 대조](../artifacts/validation/2026-11-03/post_change_audit_v1/README.md)에서는 대표 교통 명령 변화/추가 감속 없이 우회 검사 비용 약0.2%를 확인했다. 기존 먼 벽 연결 조합은 [최고2.737m/s(9.85km/h)](../artifacts/validation/2026-11-02/solo_ray_combination_v1/README.md)였으나43.433m에서 기억점 래치로 미완주해 보류했다. 첫 래치 표면 대응은 약0.2mm 왕복으로10µm 수렴 조건을 못 채웠고 반복 증가/감쇠만으로 해결되지 않았다. 선정 완주 구성을 유지하며 실제 관측·수치 수렴/해제·고속 유지 손실을 분리해 보강한다.
2. **교통:** 새 월드 기본 동작은 [대표 세 장면](../artifacts/validation/2026-11-02/official_traffic_v1/README.md)에서 이어졌다. 앞차 근접 정지 뒤의 영구 교착을 조향 가능 전이 선택으로 풀었고, 조기 복귀가 처음 실제 호출됐다. 새300초 구성에서는 오른쪽 옆차와 초기정지 앞차를 함께 만난 뒤 왼쪽 우회로 들어가 공식 기둥 앞에서61.85초 정체했다([실패 원본](../artifacts/validation/2026-11-02/official_race_distance_v1/README.md)). [현재 반환과 깊게 겹치는 우회 진입을 제외](../artifacts/validation/2026-11-02/entry_obstruction_v1/README.md)한 뒤 [동일 초기 배치30초](../artifacts/validation/2026-11-02/entry_obstruction_closed_loop_v1/README.md)는16.278m·정체3.857초·노면/벽/접촉 표본0·STOP/정상 종료를 확인했다. 우측옆차 약79.6mm는 원래15cm 기준 실패로 보존한다. [새 경기 거리 시험](../artifacts/validation/2026-11-02/official_race_distance_v2/README.md)은293.576초에거리5랩·최장정체3.859초·STOP/정상 종료를 확인했다. 접촉/노면 표본0, 옛HOLD/최종 간격/15cm 실패는 별도로 보존한다. 그 밖에 남은 것은 우회 중 재출발 뒤 약4.2초의0.35m/s, 끼어들기의 늦은 인지와 강한 제동, 왼쪽 우회, 지속 코너 병렬, 비스듬한 정지차, 갈림길 마커와 벽 동시 가림이다. [오른쪽 직선 재접근](../artifacts/validation/2026-11-03/right_reapproach_v3/README.md)은 11/3 한 각본에서 확인했으며 다른 후방 접근까지 보장하지 않는다. [대표 오목 감지 윤곽](../artifacts/validation/2026-11-03/irregular_actor_v1/README.md)은11/3 우회/복귀·접촉0·최대2바퀴잔디를 확인했지만 낮은 차체나 모든 개방형 구조의 검증은 아니다. 후보는 기본 OFF다. 제한 시험을 전체 본선 완료로 올리지 않는다.
3. **신호·재개:** 기본 수평 카메라의 선두2칸 광원 시야 문제는 남는다. 다만 **기둥 없는 월드·상향20° 후보의 선두index2**에서 RGB 최초 출발→본선38.014m·ID0/20/30/45→브라우저STOP을 [1회 확인했다](../artifacts/validation/2026-11-03/post_free_facility_v1/README.md). 원본 분석은 정지 창/조향 기록 간격 두 실패이며, 같은 자세의 연속정지·전 조향각 접지 외곽 보충 결과와 구별한다. 전체 랩·다른 선두칸·높은 앞차 가림·실물 장착은 미검증이다. 같은 경기 복구는 [대표 흐름](../artifacts/validation/2026-11-02/official_ops_v1/README.md)과 [마커 없이 오른쪽 본선18.9m](../artifacts/validation/2026-11-02/official_ops_recovery_v2/README.md)에서 확인했다. 일반 STOP→START는 [RESET 버튼 없이 재개](../artifacts/validation/2026-11-03/ordinary_resume_v1/README.md)·같은 경기7.939m 진행·양쪽 정지를 확인했다. 최초 신호·새 입력/경로·MCU 재ARM을 유지한다. 평가기 열 수 오류/run1, 중간 단계 대기 timeout/run2의 원본 exit1은 보존하며, 저장된 실제 상태/자세의 행동 확인과 구별한다.
4. **실물·연산:** 실제 배치·전원/STOP·조향·제동·C1 시각·IMU 축·RGB 입력과 최신 Jetson 처리 여유 미검증. [4 cm 검사 묶음 계산](../artifacts/validation/2026-11-01/sweep_batch_v1/README.md)은 PC 중앙 비용 7.488→6.533 ms, [최신 79파일 이관 묶음](../artifacts/validation/2026-11-02/handoff_profile_bundle_v1/README.md)은 단독 1,860·교통 1,059판단의 격리 일치와 로컬 비용을 확인했다. 총비용 p95 단독 13.50 ms·교통 27.17 ms이며 교통 판단 자체 p95 23.47 ms다. Jetson/전체 노드 마감 검증은 아니다. 함수 분해 후 [동일 호출의 벽 적합 중복 제거](../artifacts/validation/2026-11-02/same_call_reference_v1/README.md)는 1,059판단을 그대로 유지했고 로컬 중앙 비용 12.37→11.15 ms였다. 최신 v2 묶음의79파일 해시·격리import·교통1,059/단독1,860판단 일치도 확인했다. 추가 성능 최적화는 잠시 멈추고 새 월드 교통 거리 시험으로 이어간다. 이전 77파일 묶음은 별도 시점으로 보존한다.
5. **제거/후순위:** 방지턱 통과·전용 인식·정밀 현가는 공식 제거로 필수에서 뺐다. 유효 음수 축 값은 현재 부호만으로 전진을 막지 않는다. 일반적인 도로/잔디 접지·헛돎·피치/롤 관측 한계는 남긴다. [과거 턱 실패](../artifacts/validation/2026-10-31/g2_bump_edge_diagnosis_v1/README.md)는 보존한다.

<a id="다음-작업과-진행-범위"></a>

## 다음 작업과 종료 상태

**11/3 작업 진행 중.** GPT가 설계·구현·검토하고 GPT-6 Luna가 시뮬레이션을 실행·회수한다. 사용자가 종료를 요청할 때까지 현행 계획의 필요한 작업을 이어간다.

- **단독·운영:** 기둥 없는 월드에서 [선정 한 랩](../artifacts/validation/2026-11-03/solo_post_free_v1/README.md) 34.763초·최고 2.394m/s와 [일반 STOP→START](../artifacts/validation/2026-11-03/ordinary_resume_v1/README.md)를 확인했다. [고속 표면 정합·모터축 필터 비교](../artifacts/validation/2026-11-03/surface_finite_solver_v1/README.md)는 43.433m 실패를 해결하지 못해 미채택이다.
- **교통:** [비사각형 상대 우회/복귀](../artifacts/validation/2026-11-03/irregular_actor_v1/README.md)는 15.186m·최소 116.7mm·접촉 0이었다. 좁은 병렬 출발을 막던 측방 시차 겹침은 [출발 후보](../artifacts/validation/2026-11-03/side_departure_runtime_v1/README.md)로 보완했다. [재접근 v1](../artifacts/validation/2026-11-03/right_reapproach_v1/README.md)은 장면 미성립/상대 지연 실패, [v2](../artifacts/validation/2026-11-03/right_reapproach_v2/README.md)는 장면 성립 뒤 중앙 복귀로 측면 접촉했다. 두 실패를 보존한다.
- **이번 결과:** [현재 차선 유지 후보](../artifacts/validation/2026-11-03/rear_lane_continuity_v1/README.md)는 기본 OFF다. 관련 223회귀·2패키지 빌드 뒤 [같은 각본 v3](../artifacts/validation/2026-11-03/right_reapproach_v3/README.md)에서 최소 84.97mm·접촉/겹침 0·2.880m·STOP/정상 종료로 별도 목표를 통과했다. 0명령 표본 0.98→0.48초, 최고 1.593→0.985m/s다. 원시 옛 150mm/뒤 유지 실패는 별도 보존한다.
- **합류와 경기 거리:** [우회 중 앞차 재출발](../artifacts/validation/2026-11-03/merge_lane_preference_v1/README.md)은 재출발 뒤 3.0초 복귀·최종 FOLLOW·15.902m·최고 1.569m/s·접촉 0이었다. 새 선호 적용은 0회로, 켠 상태에서의 공존만 확인했다. 원시 HOLD/도로 안 실패는 보존한다. [기둥 없는 300초 경기 거리](../artifacts/validation/2026-11-03/post_free_race_distance_v1/PLAN.md)를 이전 두 상대 각본·판정 그대로 1회 등록해 실제 진행과 장기 정체를 확인한다. 단독 고속·교통·선두 상향 카메라 실적은 한 설정의 성적으로 합치지 않는다.

센서·물성·독립 보호·STOP를 유지했다. 커밋·푸시·Jetson·전원 변경·주최 측 게시는 하지 않았다.

다음 개발 재개는 [분야별 완성 기준](DEVELOPMENT_PLAN.md#다음-작업과-분야별-완성-기준)을 따른다.

1. **완료한 이번 묶음:** 기둥 제거만으로 상향20° 선두 한 칸에서 본선38m·갈림길 마커 관측을 확인했다. [기둥 대응 전후 대조](../artifacts/validation/2026-11-03/post_change_audit_v1/README.md)는 대표 교통2개/기둥 없는 출발에서 명령 동일, 단독 고속 제한 완화, 우회 진입 검사 비용 약0.2%다. 기둥 전용 추가 개발은 종료하며 일반 장애물 기능은 유지한다. 이전 전체 회귀13실패·기둥 포함 정체·고속43.433m 미완주를 삭제하거나 성공으로 바꾸지 않는다.
2. **시설 정책:** 실제 트랙 안에 기둥이 없다는 사용자 확인에 따라 팀 파생본의 `tl_post`/충돌 제거를 적용했다. 다른 링크 변경0, 원본ZIP 보존. 신호등은 공식 파일 자체보다 제작 사양을 기준으로 선택·보완하며 [평면 발광/소등 한계](reports/OFFICIAL_UPDATE_2026_11_02.md#31-112-사용자-후속-확인과-신호-광학-검토)는 남는다. 기둥 제거 후 제어 기능을 전부 되돌릴 이유는 대조에서 발견되지 않았다. 정지 조향/GUARD 선택 후보는 기둥 없는 출발에서 필수였다고 주장하지 않는다.
3. **단독 속도 재개 지점:** [직선 증속 후보](../artifacts/validation/2026-11-03/solo_speed_envelope_v1/README.md)는 완주 입력 2개에서 속도 명령 최고 2.78 m/s, 2.5 m/s 이상 1.38/1.50초였다(최초 허가~STOP, 결승 뒤 포함). 42–44 m에 변경은 없었다. 격리한 제어기에서 OFF 1,849판단 동일, ON 주행 창 1,774판단 독립 계산 일치. **실행 트리에 미적용·새 빌드/주행 미실시**다. 재개 시 [등록한 파일 적용→관련 회귀·빌드→단독 1랩](../artifacts/validation/2026-11-03/solo_straight_closed_loop_v1/PLAN.md)에서 결승 전 실제 차체 속도와 랩을 확인한다.
4. **다중 차량·운영·실물/Jetson:** 각 기능이 필요한 동작을 수행할 수준까지 이어서 작업하고 다음 필수 기능으로 넘어간다. 전체 기본 기능을 갖춘 뒤 성능·대응 범위를 다시 보강한다. 초안의 시간별 교대와 고정 순환은 사용자 정정에 따라 폐기했다.
5. 사용자 예상은 부품까지 약 1주일·실차 시험은 마지막 약 1주일이다. 확정 납기로 쓰지 않고 그동안 시뮬과 운영 준비를 병행한다. 공식 대회는 11/14, 결산은 11/13.

원시 증거·활동 이력은 보존했다. [11/3 활동](activity/2026-11-03.md)·[11/2 활동](activity/2026-11-02.md)·[직전 계획 원문](archive/2026-11-02-plan/DEVELOPMENT_PLAN.before.md)에서 변경 경위를 확인한다.
