# 센서 분리 중 실물 진입 구성 대조

11/14 사용자 재개 후 주 담당의 정적 대조다. 새 실물 설정을 채택하거나 장치를 실행한 결과는 아니다.

r401 실물 진입 템플릿의 execution/motion/follow 설정과 r475 실제 완주 기록의 header를 대조했다. JSON 문자열은 같은 값으로 파싱하여 비교했다. 차이는 LiDAR frame ID(`c1_unmounted_preview` 대 `laser_frame`)와 r475의 `crop_edge_component_trial=true`뿐이다. 실행 설정과 차량 기하·현재 경로 우선 선택기의 두 플래그는 같다. 실물 템플릿에 없는 crop 연결은 r444/r446 및 r453의 명시 `host_estimated_crop_v1` 진입으로 다루며 시각을 verified로 바꾸지 않는다.

이는 시뮬의 모든 런타임 파라미터/센서 계약이 실물에서 같다는 뜻이 아니다. 비교 대상 header 밖의 실제 `use_sim_time=false`, `native_estimated_v1`, `timing_verified=false`, `host_estimated_v1`, 센서 토픽/프레임, 설치 영점 bundle, MCU 출력 허가는 기존 실물 진입 절차를 유지한다. r481 이후 소스 수정은 통합 뒤 해당 저장 입력으로 별도 검증한다.

차량 도착 후 우선할 것은 기존 [최소 명령안](../../2026-11-13/operator_handover_r453_v1/MINIMAL_HANDOVER.md)의 장착 profile/영점·방향·물리 STOP 확인과 짧은 저속 출력이다. 센서 분리 상태에서 이 확인을 완료 표시하거나 임의 실장값을 생성하지 않았다.

근거: [header 차이 및 원본 SHA](header_comparison.json). 새 실물 출력·센서 접근·시뮬 실행 없음.
