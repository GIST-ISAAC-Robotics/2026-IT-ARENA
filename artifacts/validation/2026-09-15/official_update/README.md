# 2026-09-15 공식 갱신 검증 자료

## 후속 내부 원인·종료 진단

- [OpenCV 내부 추적 144조건](aruco_internal_trace.json), [그룹화 전 유효 후보 탈락 집계](aruco_internal_summary.json), [해설](../../../../docs/track/ARUCO_DETECTION_ROOT_CAUSE_2026_09_15.md).
- SIGINT 재시험: [1](shutdown_group_01/shutdown_trace.json), [2](shutdown_group_02/shutdown_trace.json), [3](shutdown_group_03/shutdown_trace.json), [전체 코스](shutdown_overview_04/shutdown_trace.json), [추가 1회](shutdown_warm_/shutdown_trace.json). 이번 5회 모두 정상 종료로 간헐적 지연은 재현되지 않았습니다.
- [서버 종료 서비스 후보](shutdown_service_01/shutdown_trace.json): 응답 true·반환0·잔존없음, 아직1회입니다. [설치 버전 소스/해시](shutdown_sources/manifest.json), [과거 스택 재분석과 한계](../../../../docs/simulation/SHUTDOWN_DIAGNOSIS_2026_09_15.md).
- 각 실행 폴더 report.json·사진·서버/브리지 로그는 그대로 보존합니다. 기본 월드/검출기/종료 함수는 이 진단에서 변경하지 않았습니다.

## 후속 도형 앞면 A/B

- [PNG 대조](face_compare_pbr/report.json): 지지대 포함, 기본 9/24, 촬영·정상 종료.
- [도형 비교](face_compare_cells/report.json): 원본 링크 유지+별도 앞면, 기본 15/24, 촬영 완료·서버 종료 -9.
- [조건 일치·후보 좌표](face_comparison.json), [설정 비교](face_detector_probe.json): 0.02에서 PNG 17/24·도형 19/24.
- [설명·재현·한계](../../../../docs/track/GANTRY_MARKER_FACE_COMPARISON_2026_09_15.md). 기본 월드/검출기는 유지합니다.

## 앞선 공식 갱신

기준: `v2026.09.14`, 커밋 `0eb3a217e79a75c249b3ce9ec6b087bf82238c0c`.

- [공식 원문·릴리스·이슈 전체 댓글 스냅샷](upstream/manifest.json)
- [원본 비교·엄격 SDF·200스텝 실행](release_audit.json)
- [원본 마커 정면 렌더링](raw_front/result.json): 네 ID 정상 표시·검출, 정상 종료. 카메라마다 3프레임 수신, 마지막 저장 프레임의 검출 결과.
- [원본의 차량 상당 거리별 정지 시야](raw_route/report.json): 기본 검출 10/24, 촬영 완료 후 서버 종료 -9. 실패 보존.
- [임시 신호 간섭 발견](runtime_route/report.json): ID 0을 팀 신호 가로대가 가림.
- [신호 0.60 m 이동 후](runtime_final/report.json): 기본 10/24로 원본과 동일, 정상 종료. 출발 슬롯 6개 사진 포함.
- [동일 RGB의 검출 매개변수 비교](detector_probe.json): 0.05→0.02에서 10/24→17/24. 중복 후보 등 후속 검증 필요. 운영 검출기 미변경.
- [짧은 차량·센서 시험](smoke/official_low_load_30_smoke.json): 직진·회전·센서·명령 중단 정지 관측, Gazebo 종료 실패. 실차나 완주 검사 아님.

[전체 재감사](../../../../docs/track/OFFICIAL_V2026_09_14_REAUDIT.md)와 [회의 변화](../../../../docs/reports/OFFICIAL_UPDATE_2026_09_15.md)에 확정/임시/미검증을 구분했습니다. 이전 증거는 삭제하지 않았습니다.

## 마지막 후속: 임시 갠트리 지지대

- [최초 지지대 간섭 실패](support_clearance_failure.json): ID 30의 지름길 쪽 기둥이 branch_1의 11개 표본을 막음.
- [최종 생성·보존·정적 검사](final_build.json): 그 기둥을 생략한 한쪽 지지형, 나머지는 양쪽 기둥형. 공식 마커 링크/PNG/scene 보존.
- [최종 거리·그리드 RGB](supported_route/report.json): 기본 목표 ID 검출 9/24, 정상 종료.
- [지지대 포함 검출 후보 비교](supported_detector_probe.json): 후보 0.02에서 17/24, 출발 6곳 빨강 유지.
- [최종 전체 코스 5방향·지지대 상세 2장](final_overview/report.json): 7장 촬영 완료, 서버 종료 -9·잔존 프로세스 없음. 사진은 해당 폴더 PNG 원본.
- [최종 5패키지 빌드](final_colcon.log), [Python/ROS 210개](final_pytest.log), [엄격 SDF](final_strict_sdf.log), [공식 해시](final_hash_check.json), [실험 해시](experimental_final_hash_check.json).

기본 검출의 감소는 ID 20 1.5 m 표본 한 개입니다. 이전 실패·지지대 없는 공식 원본 증거는 보존했습니다. 추가 지지 구조는 공식 제작 도면이나 강도 검증이 아닙니다.
