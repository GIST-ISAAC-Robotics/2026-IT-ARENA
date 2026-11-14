# 시뮬레이션·계산 작업 보존 색인 — 2026-11-14

전시용으로는 [성공 시뮬레이션 영상 8편](SIMULATION_RECORDINGS_2026_11_15.md)을 별도로 선정했다. 코스별 속도 대표·시설물/센서 오차 조건·기능 시연을 구분한다. 아래의 실패·중단·미실행 이력은 영상 선정과 관계없이 보존한다.

경기 종료 후 기록 정리용 문서다. 사용자는 경기 시작 시 사용 가능한 MCU가 없고 Jetson에도 운용 장치가 연결되지 않아 차량이 움직이지 못했다고 보고했다. 이 현장 결과는 [경기 종료 보고](COMPETITION_RESULT_2026_11_14.md)에 보존하며, 아래 시뮬레이션 성공·개발 완료와 혼합하지 않는다. 공식 순위·점수나 고장 원인은 여기서 추정하지 않는다.

**대회용 최종 소스는 [ESP32 최종본](../releases/FINAL_ESP32_2026_11_14.md)의 `r533 selected_v2` 734파일이다.** 현재 저장소의 후속 코드, 아래 시험·수정, UNO Q 대체안이 모두 그 최종본에 포함되었다는 뜻이 아니다. 당시 소스와 후속 결과를 구분해 보존한다. 이 색인은 새 시험·설치·개발 재개의 지시가 아니다.

## 전체 목록과 판정 읽기

- 전체 산출물 그룹은 [공개·로컬 보존 안내](ARTIFACT_CATALOG_2026_11_14.md)와 [기계 색인](../../artifacts/validation/2026-11-14/repository_closeout_v1/artifact_index.json)에서 찾는다. 아래는 중요한 갈래의 안내이며 모든 로그를 재나열하지 않는다.
- 진행 당시의 우선순위·설정·미해결 항목은 원문 보관본 [PROJECT_CONTEXT](../archive/2026-11-14-closeout/PROJECT_CONTEXT.before.md)와 [DEVELOPMENT_PLAN](../archive/2026-11-14-closeout/DEVELOPMENT_PLAN.before.md)에 남아 있다. 그 안의 향후 실행 문구는 과거 계획이다.
- **완료**는 해당 구현·정적 검사·저장 입력 계산·정해진 시뮬 대표의 완료만 뜻한다. **실패**는 원 판정을 보존한다. **부분 실행·중단**은 중단 시점까지의 관측만, **미실행**은 준비·초안만 남았음을 뜻한다.
- 주행 판정, STOP, 정상 종료/PID 회수, 입력 무결성, 계산 처리시간은 각각 별도 판정이다. 한 항목의 통과로 다른 실패를 덮지 않는다. 모든 시뮬·저장 입력 계산은 실차 주행 증거가 아니다.

## 이전 성공과 후속 비교의 기준

| 갈래 | 보존할 결과와 한계 | 근거 |
|---|---|---|
| 현재 경로 우선 단독 | r475는44.66m·GREEN 이후49.882sim초 완주, 접촉/노면이탈/명령만료0. 후속 실측 조향 상한이나 다른 출발·벽 조건의 성공으로 소급하지 않는다. | [r475 결과](../../artifacts/validation/2026-11-14/solo_full_lap_r475_v1/RESULT.md) |
| 출발 변형·독립 guard | r477·r485의 정체 실패 이후 r489의 독립 보호 조건 정리를 거쳐 r492는 출발 변형1회44.77m 완주. 이전 실패는 별도다. | [r485 실패](../../artifacts/validation/2026-11-14/solo_variation_recovery_r485_v1/RESULT.md), [r489 수정](../../artifacts/validation/2026-11-14/independent_guard_route_r489_v1/RESULT.md), [r492 결과](../../artifacts/validation/2026-11-14/solo_guard_recovery_r492_v1/RESULT.md) |
| 앞차 제한 대표 | r494 현재 관측 경로 점유 비교 뒤 r496은10sim초5.15m·최장정지0.102초·최소간격0.632m. 전체 교통 랩·추월 완료와 구분한다. | [r494 수정](../../artifacts/validation/2026-11-14/traffic_current_geometry_r494_v1/RESULT.md), [r496 결과](../../artifacts/validation/2026-11-14/traffic_current_geometry_r496_v1/RESULT.md) |
| Jetson 저장 계산 | r476은 plan217/current113회, r493은 이전 선택의 plan508회 계산이다. r493 중앙66.001/p95104.994/최대158.650ms·100ms 초과35개. 소스·모집단이 다른 계측의 개선율을 합산하지 않는다. | [r476](../../artifacts/validation/2026-11-14/r476_runtime_route_cost_v1/RESULT.md), [r493](../../artifacts/validation/2026-11-14/jetson_lap_cost_r493_v1/RESULT.md) |
| 연속 입력·명령 전달 | r523 입력소비, r524 compact 기록, r526 fresh-current 전달 변경은 당시 공통 통합 이력이다. 양수 명령의 긴 공백 감소와 짧은0명령 증가를 함께 보존한다. 최신 실센서·guard·MCU 동시10Hz 달성 근거가 아니다. | [r523](../../artifacts/validation/2026-11-14/jetson_drain_budget_r523_v1/RESULT.md), [r526](../../artifacts/validation/2026-11-14/fresh_current_delivery_r526_v1/RESULT.md), [r536 사후 감사](../../artifacts/validation/2026-11-14/current_delivery_audit_r536_v1/RESULT.md) |

## r533 이후 주요 갈래

### 소스 묶음·입력 전달·운영 도구

| 작업 | 정리 시점 상태 | 근거·재현 출발점 |
|---|---|---|
| r533 업데이트 묶음 | selected_v1/v2/v3 생성 이력 보존. **v2가 ESP32 최종 소스 기준**이며 설치 성공을 뜻하지 않는다. v3는 미채택 반대벽 전환 초안을 포함해 실행 대상에서 제외했다. selected_v4는 준비 후 중단하여 **미생성**이다. | [v1 준비·생성](../../artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/RESULT.md), [v2 영수증](../../artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/SELECTED_V2_RESULT.md), [v3 미채택 정정](../../artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/SELECTED_V3_RESULT.md), [v4 중단](../../artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/selected_v4_preparation_v1/PAUSED.md) |
| r535·r551 운영 화면 | 재연결 후 이전 미리보기 응답 표시와 숨김 탭의 미리보기 lease/카메라 요청을 보완한 PC 구현·검사 완료. 운영 heartbeat 유지. 실제 Jetson 결합 부하 개선은 미측정이다. | [r535 계약·수정](../../artifacts/validation/2026-11-14/field_panel_contract_r535_v1/RESULT.md), [r551 공통 이관](../../artifacts/validation/2026-11-14/panel_preview_idle_r551_v1/common_transfer/RESULT.md) |
| r536 전달 경계 감사 | 저장 r523 timeline 분석과 최신6ms/20ms/compact 조합의 계측 레시피 준비 완료. 최신 조합의 Jetson 통합 재계측은 **미실행**이다. | [결과](../../artifacts/validation/2026-11-14/current_delivery_audit_r536_v1/RESULT.md), [계측 경계](../../artifacts/validation/2026-11-14/current_delivery_audit_r536_v1/MEASUREMENT_RECIPE.md) |
| r543·r547 guard 상태 축약 | r543 선택형 compact 공통 이관·ROS 연결21검사 완료, 기본 full 유지. r547 overlay/상세 sidecar 회수·복원 PC12검사 완료. 실제 Jetson 전달량·명령 연속성 효과는 **미측정**이다. | [r543 이관](../../artifacts/validation/2026-11-14/guard_status_payload_r543_v1/common_transfer/RESULT.md), [r547 준비](../../artifacts/validation/2026-11-14/guard_status_runtime_recipe_r547_v1/RESULT.md) |

### 굴곡벽·기억점과 실측 조향 상한

| 작업 | 정리 시점 상태 | 근거·재현 출발점 |
|---|---|---|
| r530→r538→r539 | r530은 앞차와의 최종간격 기준 실패. r538은 현재/직전 제동 경로 밖 기억에 의한 전역0.1m/s 제한을 정리했다. r539는 같은 조건12sim초5.660m·최종간격0.636879m로 원11개 주행 기준 **PASS**. 전체 관측 목록 무결성 **FAIL**과 실제 실행 입력267개 불변을 별도로 보존한다. 추월 성공은 아니다. | [r530 원 실패](../../artifacts/validation/2026-11-14/field_wall_resource_drive_r530_v1/RESULT.md), [r538 수정](../../artifacts/validation/2026-11-14/guard_memory_relevance_r538_v1/RESULT.md), [r539 원 결과·무결성 구분](../../artifacts/validation/2026-11-14/field_wall_guard_relevance_drive_r539_v1/RESULT.md) |
| r534 단독 준비 | 굴곡벽 단독 레시피·수집 준비만 완료. 해당 실행 단위의 주행은 **미실행**이다. 뒤의 실측 조향 단독은 별도 r544로 기록한다. | [r534 READY](../../artifacts/validation/2026-11-14/field_wall_solo_drive_r534_v1/READY.md) |
| r540 동일각도 비용 | 정확히 같은 현재/직전 각도의 중복 계산 제거를 후속 공통 채택. 제한된 Windows 합성 비용·출력 동등성 근거이며 r539에 포함된 변경이나 ARM 전체 성능 근거가 아니다. | [r540 결과·채택](../../artifacts/validation/2026-11-14/guard_relevance_cost_r540_v1/RESULT.md) |
| r541·r542 조향 전달 | 중립2811·운영목표2600–2980counts, 대칭 중앙 상한0.28475436568260193rad를 프로필·제어·파생 차량에 연결했다. C++401목표·관련110회귀 완료. 바퀴각은 CAD 환산이고 당시 장치 설치/flash는0이다. | [r541 선택](../../artifacts/validation/2026-11-14/measured_steering_apply_r541_v1/RESULT.md), [r542 전달](../../artifacts/validation/2026-11-14/steering_limit_propagation_r542_v1/README.md) |
| r544 첫 시도 / attempt2 | 첫 시도는 실행기 이름 감사 오류로 자식 시작0·주행0인 **하네스 실패**. 별도 attempt2는 첫 코너 이후19.56m에서 정체하여 **미완주 FAIL**, STOP·14자식 정상 종료·PID0을 확인했다. 두 실패를 합치거나 성공으로 정정하지 않는다. | [첫 시도](../../artifacts/validation/2026-11-14/measured_steering_lap_r544_v1/RESULT.md), [attempt2](../../artifacts/validation/2026-11-14/measured_steering_lap_r544_attempt2_v1/RESULT.md) |
| r545·r556 조향 검토 | 새 상한과 코너 경로 연결, 오른쪽 추가43counts 활용 가능성을 **읽기 전용 분석**했다. 비대칭 조향은 명령→guard→host/MCU→시뮬 계약 전체 변경이 필요하며 이번에 구현·주행하지 않았다. | [r545 코너 연결](../../artifacts/validation/2026-11-14/measured_steering_corner_review_r545_v1/README.md), [r556 비대칭 검토](../../artifacts/validation/2026-11-14/r556_asymmetric_steering_review_v1/NOTE.md) |
| r548 비대칭 벽 | 가정된 비대칭 굴곡·밀림·접합의 합성2대표 계산 완료. 현재 경로0.10m/s와 실제 전방 장애물0을 구분했다. 새 공통 수정·Gazebo 주행은 없다. | [r548 결과](../../artifacts/validation/2026-11-14/asymmetric_field_wall_r548_v1/RESULT.md) |

### 짧은 경로 재획득·광선 방향·희소 상대

| 작업 | 정리 시점 상태 | 근거·재현 출발점 |
|---|---|---|
| r549 revision1/2/3 | 초기 반대벽 전환 초안은 Y갈림길 반례로 **미채택**. revision2는 선호벽을 유지하며 저속 최소 관측 길이를0.207m로 조정했다. revision3는 센서 전방 오프셋에 따른 근거리/원거리 끝점 정렬을 실제 광선 방향으로 수정했다. 기능115통과와 옛 HEAD SHA 계약1불일치를 구분한다. | [최종 결과·각 revision](../../artifacts/validation/2026-11-14/current_short_path_r549_v1/RESULT.md), [초안 철회](../../artifacts/validation/2026-11-14/current_short_path_r549_v1/REVISION1_NOT_SELECTED.md) |
| r550 | 이전19.56m 정체를 통과했으나25.81m에서 다시 정지해 **미완주**. 관측 벽 접촉0·도로 밖 최대1바퀴·STOP/정상 종료·입력 불변은 완주와 별도다. | [r550 결과](../../artifacts/validation/2026-11-14/measured_steering_reacquire_lap_r550_v1/RESULT.md) |
| r552→r553→r557 | r552는 최대0.1m/s 재획득의 운동 여유를 원판으로 적용했다. r553은19.56/25.81m를 통과했지만25.94m에서 발행 전 재검사로 **미완주**. 같은 scan의2ms 경과에 따른 여유 차이를 분석해 r557은 저속 재획득에 실제 반환점+고정55mm 여유를 선택했다. 저장 current0→0.1·215회귀 근거이며 새 전체랩 성공은 없다. | [r552 이관](../../artifacts/validation/2026-11-14/radial_motion_reacquire_r552_v1/INTEGRATION.md), [r553 실패](../../artifacts/validation/2026-11-14/measured_steering_radial_lap_r553_v1/RESULT.md), [r557 이관·한계](../../artifacts/validation/2026-11-14/r557_current_radial_stall_v1/INTEGRATION.md) |
| r554 광선 방향 후속 | 추월 최초 벽 기준 한 곳에 실제 광선 방향 전달을 공통 이관했다. 관련219검사 중 **217통과·2실패**를 보존하며 전체 통과로 쓰지 않는다. 새 교통 주행 성공의 근거는 아니다. | [조사 범위](../../artifacts/validation/2026-11-14/ray_order_followup_r554_v1/NOTE.md), [이관 결과](../../artifacts/validation/2026-11-14/ray_order_followup_r554_v1/common_transfer/RESULT.md) |
| r537·r546 희소 actor | 지주형 상부·보수 collision 외곽과 동일 actor 후면 마커 OFF/ON 비교 레시피를 준비했다. 명목 광선 교차·기하 감사는 실제 영상 검출이나 폐루프 비교가 아니다. | [r537 레시피](../../artifacts/validation/2026-11-14/sparse_actor_recipe_r537_v1/README.md), [r546 조향 반영 준비](../../artifacts/validation/2026-11-14/sparse_actor_measured_steering_r546_v1/README.md) |
| r555·r559 | r555는 빈 `vehicle_marker_profile_path`의 ROS CLI 해석 오류로 **출발 전 실패·주행0**. r559는 빈 optional 전달을 생략하는 후보와 trace OFF 후속을 분리했다. 원 실패는 유지하며 실제 수정 적용 주행은 별도 r560이다. | [r555 원 결과](../../artifacts/validation/2026-11-14/r555_sparse_actor_ready_v1/RESULT.md), [r559 후보](../../artifacts/validation/2026-11-14/r559_optional_launch_parameter_v1/stage/RESULT.md), [trace 후속 준비](../../artifacts/validation/2026-11-14/r559_optional_launch_parameter_v1/trace_stage/RESULT.md) |
| r558 단독 | r557 이후 같은 조건 단독 실행을 준비했으나 사용자 중단으로 **미실행**. r553 실패를 해소한 완주 결과는 없다. | [r558 PAUSED](../../artifacts/validation/2026-11-14/r558_solo_observed_crawl_ready_v1/PAUSED.md) |
| r560 희소 상대 | 수정 launcher로 **부분 주행 후 사용자 중단**. 약4.69m·최소 상대간격0.2965m·관측 접촉/벽겹침0, STOP·정상 종료·PID0·필수467입력/ZIP 불변을 확인했다. 원 FAIL 유지.12sim초 대표 완료·추월·전체랩 성공이 아니다. | [r560 결과](../../artifacts/validation/2026-11-14/r560_sparse_actor_drive_v1/RESULT.md) |
| r561 사후 반환 감사 | r560 이후 번호로 확인된 보조 작업. `audit_returns.py` 첫 초안만 작성했고 **구문·단위·실자료 평가 모두 미실행**. r560 실제자료 평가 결과는 없으며 준비 완료 도구로 쓰지 않는다. | [r561 미검증 상태](../../artifacts/validation/2026-11-14/r561_sparse_actor_return_audit_v1/README.md) |

## 후속 오프라인 대체안과 원자료 재현 경계

UNO Q 펌웨어·호스트, 임시 IMU 자동 보정과26×21cm 자체 반환 마스크는 별도 후속 PC 구현/검사 이력이다. [UNO Q 보고서](../hardware/UNO_Q_DEPLOYMENT_2026_11_14.md)와 [자체 반환 마스크](../sensors/QUICK_SELF_MASK_2026_11_14.md)에 원 코드·검사 범위를 보존한다. 오프라인 빌드·테스트가 끝났어도 현장 업로드·실제 센서 보정·차량 구동은 하지 못했으며 ESP32 최종본의 실측 보정으로 취급하지 않는다.

각 실행의 재현은 현재 `main`으로 재실행하는 방식이 아니라 해당 보고서가 가리키는 **당시 source snapshot/manifest, 선택 YAML·SDF, 원 실행기·명령, 원 판정**에서 시작한다. `r533 selected_v2`와 v3는 서로 다른 SHA의 묶음이며, r539의 성공은 r540 최적화나 실측 조향 설정을 적용한 결과가 아니다. r549의 미채택 revision1과 후속 revision도 섞지 않는다.

대용량 원시 scan·pose·bag·ZIP 등은 [산출물 보존 안내](ARTIFACT_CATALOG_2026_11_14.md)의 공개/로컬 경계를 따른다. GitHub에 상세 바이너리가 없다는 이유로 결과를 새 소스의 증거로 대체하거나 원 실패를 삭제하지 않는다. 이 문서는 기존 근거를 읽어 연결했으며 새 계산·검사·시뮬·장치 작업은 수행하지 않았다.
