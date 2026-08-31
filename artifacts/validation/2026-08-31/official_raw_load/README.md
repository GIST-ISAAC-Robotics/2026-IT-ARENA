# 공식 원본 로드 재감사 근거

2026-08-31, 공식 `v2026.08.31` / `d61c5db9252cedfbc163cd044a47671df91e1660`를 새로 압축 해제하여 검사했습니다. **원본 자체의 파싱 실패를 포함하는 근거 묶음이며 전체 통과 보고서가 아닙니다.**

- [현재 단일 이슈 초안](../../../../docs/track/OFFICIAL_UPSTREAM_ISSUE_DRAFTS.md)
- [정리 전 감사 문서·개별 초안 보관 ZIP](../issue_consolidation/before_consolidation.zip): 당시 본문을 복구할 수 있는 이력이며 현재 제출 목록이 아닙니다.
- [기계 판독 요약: 로드 15건](summary.json)
- [23개 원본 파일과 5종 사본 해시](fixtures.json)
- [초안 재현 코드·로그·RGB·보존·링크 최종 대조](review_validation.json)

## 주요 관찰

| 단계 | 관찰 | 직접 근거 |
|---|---|---|
| ZIP 그대로 GUI 실행 | Code 14·8 각 4회, Code 9 1회; 월드 파싱 실패. 그런데 gz sim exit 0 | [원본 GUI 로그](logs/raw_gui_01.log) |
| 경로만 수정 | Code 8·9로 여전히 실패 | [서버 로그](logs/paths_only_server_01.log) |
| script 제거 | 엄격 SDF 검사 통과 | [검사 로그](logs/remove_script_check_01.log) |
| script/경로 수정, 1113 links | 월드 초기화 메시지 이후 첫 갱신 180초 내 미완료, 시험 프로세스 강제 정리 | [180초 서버 로그](logs/script_and_paths_server_02_long.log) |
| 동일 형상 10 links | 약 2.886초·2.884초, 첫 갱신 1회 후 정상 종료 | [1차](logs/merge_static_server_01.log), [재확인](logs/merge_static_server_03_repeat.log) |
| 1113 links, Physics 제외 | 약 5.946초에 첫 갱신·종료. 물리 경로를 제외한 대조일 뿐 물리 통과 아님 | [대조 로그](logs/script_and_paths_server_03_no_physics.log) |
| albedo 경로 오류 | 각 마커 Unable to find file, RGB 검은 판 | [렌더러 로그](render/path_bad/server.log), [결과](render/path_bad/result.json) |
| 경로 수정 후 | 파일 오류 없이도 4개 마커가 검게 표시됨 | [결과](render/path_fixed/result.json) |
| diffuse 흰색만 추가 | 같은 카메라·광원에서 4/4 ID 검출 | [결과](render/diffuse_white/result.json) |

## 실제 RGB 비교

- ID 30: [경로 수정만 적용 — 검은 판](render/path_fixed/marker_30.png), [diffuse만 추가 — 패턴 표시](render/diffuse_white/marker_30.png).
- ID 0/20/45도 각 `render/` 폴더에 원본 PNG와 SHA-256을 보존했습니다. RGB는 밝기·대비·좌우 방향을 보정하지 않았습니다.
- 두 조건의 SDF 차이는 [diffuse 네 줄](patches/diffuse_only.diff)뿐입니다. 초기 로드 수정은 [script 제거](patches/remove_script.diff), [경로만 수정](patches/paths_only.diff), [두 수정 합계](patches/script_and_paths.diff)로 분리했습니다.

진단 카메라는 마커 정면 35 cm, 높이 10 cm, 640×480, 수평 화각 0.65 rad, 2 Hz입니다. 이 RGB 시험은 Physics를 제외했습니다. D435i·실차·주행 중 인식·마커 배치 안전성 시험이 아닙니다. `capture_complete`는 사진 수집 완료이지 모든 마커가 보인다는 뜻이 아닙니다.

## 보존과 공개 상태

- 원본 ZIP·운영 월드·차량·기존 활동 기록은 변경하지 않았습니다.
- 공개용 로그는 사용자 경로를 `<WORKSPACE>`·`<WSL_HOME>`로 치환하고 ANSI 제어문자를 제거했습니다. 내용이 바뀌지 않은 원시 로그·초기 JSON·전체 시험 사본은 `build/official_load_audit_20260831/`에 있으며 원시 로그 해시를 요약에 남겼습니다.
- 초기 감사 도구의 물리 로드 문구 판정 오류는 공개 요약에서 로그를 다시 읽어 정정했습니다. 당시 원시 파일은 덮어쓰지 않았습니다.
- 최종 검사에서 첫 45초 강제 종료 로그 두 개의 끝에 잘린 ANSI 색 코드가 남아 있음을 확인했습니다. 검토용 사본에서는 제어 바이트만 `<ESC>`로 표시하고 잘린 문자열 자체를 남겼습니다. 원시 로그·해시·관측 결과는 유지했습니다.
- 이 묶음을 만들 당시에는 로컬에만 준비했으며 주최 측 이슈·댓글·팀 저장소 push는 하지 않았습니다. 이후 단일 초안으로 정리하면서 팀 저장소 반영 대상으로 포함했습니다. 과거 JSON의 9건 상태와 공개 여부는 그 검사 당시의 기록이며, 현재 범위와 게시 상태는 프로젝트 현황에서 확인합니다.
- 기존 관찰용 Gazebo는 유지했고, 이번에 만든 시험 프로세스는 종료했습니다.
