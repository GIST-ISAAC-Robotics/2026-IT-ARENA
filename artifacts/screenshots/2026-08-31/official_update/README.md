# 공식 트랙 갱신 확인 사진과 이슈 증거

촬영일: 2026-08-31. WSL Ubuntu 24.04·ROS 2 Jazzy·Gazebo Sim 8.11.0에서 실제 화면을 캡처했습니다. GUI 영역을 잘라 PNG로 저장했으며 AI 생성·기하 합성 사진이 아닙니다. 평면 도식 두 장은 아래에 별도로 표시합니다.

## 현재 프로젝트 실행본

| 파일 | 내용 |
|---|---|
| [official_track_overview.png](official_track_overview.png) | 공식 본선 45 cm·지름길 20 cm 기반 전체 사선 조감도. 벽 부착 마커, 임시 신호등/방지턱/노면 표시 포함 |
| [official_start_area.png](official_start_area.png) | 출발 슬롯 6개·차량 대체 형상·빨간 신호·체크무늬 표시 |
| [official_marker_id30_after.png](official_marker_id30_after.png) | 같은 설계 측면의 벽으로 옮긴 10 cm 판/7 cm 코드 ID 30과 지름길 입구. 정적 차량 표본 통과와 노면 영역 침범 0은 서로 다른 조건 |

사진용 실행은 `render_sensors:=false depth_camera:=false tof_safety:=false`로 렌더링 센서 계산과 ToF 보호층을 비활성화했습니다. 차량·센서 몸체와 질량·트랙 기하는 유지됩니다. 자율주행 검증 실행은 별도로 모든 필요한 센서와 ToF 보호층을 켜서 수행했습니다. 마감 시 GUI는 전체 조감도에서 일시정지 상태로 남겼습니다.

전체 뷰 카메라 위치는 `(10.292, 0.96, 12.256)` m, 주시점은 `(5.6, 7.25, 0.05)` m입니다. 출발 구역은 위치 `(10.8, 3.7, 1.1)`, 주시점 `(9.97, 5.45, 0.08)`입니다. 마커 근접 화면은 차량 카메라가 아니라 운영자 관찰 시점입니다.

## 공식 원본의 오류 재현

| 파일 | 내용 |
|---|---|
| [upstream_full_overview.png](upstream_full_overview.png) | 원본 좌표·치수의 전체 배치 참고 화면 |
| [upstream_marker_id30.png](upstream_marker_id30.png) | 지름길 입구에 떠 있는 원본 ID 30 판. 검게 보이는 재질 문제도 관찰됨 |
| [upstream_bump_axis.png](upstream_bump_axis.png) | 진행 방향으로 긴 원본 방지턱 |
| [upstream_signal_all_lit.png](upstream_signal_all_lit.png) | 원본의 빨강·노랑·초록 동시 발광 |
| [upstream_id30_collision_plan.png](upstream_id30_collision_plan.png) | **계산 도식**: 실제 CSV 141표본과 SDF 판에 20×15 cm 외형을 대조해 10표본 간섭 재현 |
| [upstream_bump_axis_plan.png](upstream_bump_axis_plan.png) | **계산 도식**: 원본 SDF와 설명의 진행/횡단 치수 비교 |

원본은 `v2026.08.31` / 커밋 `d61c5db9252cedfbc163cd044a47671df91e1660`입니다. 엄격 SDF 파서에서 실패하므로 촬영용 사본에는 기존 `build_runtime_world.py`의 시스템 추가·정적 링크 병합·재질/경로 호환성 변환만 적용했습니다. 마커 위치·방지턱 치수·신호 발광값은 그대로 두었으며, 보존 ZIP이나 원본 출력물은 변경하지 않았습니다.

도식은 `python3 scripts/illustrate_official_issues.py`로 보존 ZIP에서 재생성할 수 있습니다. 원시 XWD·초기 변환 시험 파일은 Git 제외 경로 `build/official_capture_raw/`에 남겼습니다.

사진·도식·원본 수치의 구분과 게시용 본문은 [승인 대기 이슈 초안](../../../../docs/track/OFFICIAL_UPSTREAM_ISSUE_DRAFTS.md)에 있습니다. **사용자의 명시적 승인 전에는 upstream에 이슈를 게시하지 않습니다.**
