# r547 결과

**선택형 overlay와 종료 후 회수/복원 준비 완료, Windows 검사 12개 통과(0.503초)입니다.** 실제 실행·성능 측정은 하지 않았습니다. r543 공통 채택과 최신 설치 확인을 기다리는 선택 옵션이며 실측 설치/주행의 선행조건이 아닙니다.

- 기본 `full` 유지. `compact_v1` 선택 시 guard 파라미터 2개만 변경하고 새 UUID run 안의 절대 상세 경로를 지정합니다. 나머지 profile 전체 동일, planner compact 값 보존을 확인했습니다.
- 기존 r536 launcher·node wrapper·control·drain은 변경/복제하지 않았습니다. 원 r536 파일 SHA 일치와 wrapper의 `super().control()` 1회 호출을 확인했습니다.
- 원 결과 폴더 전체+상세 sidecar를 새 collection에 보존하고 복원본을 별도 작성합니다. full/pending/reference 혼합 fixture의 모든 상태 필드·수신 stamp가 동일했습니다. profile 불일치·상세 누락/변조/다른 run·덮어쓰기 거부도 확인했습니다. compact 미관측/full fallback을 축약 성공으로 표시하지 않습니다.
- r543 순수 복원 함수 2개는 AST 동일합니다. r543 원 폴더, r536 원 helper, `selected_v2` ZIP/manifest는 보존됐습니다. 공통 소스/설정 변경 0, Jetson·ARM·ROS·WSL 실행/조회 0입니다.

근거: [사용법](README.md), [fixture 로그](fixture.log), [검사](test_recipe.py), [SHA 기록](manifest.json), [새 선택본](prepared/r547_20261114T044513Z_e67a6c04bc624667ac859fbb734da581/selection.json).

미검증: 실제 설치 파라미터 반영·writer 회수·ROS 상태 전달량과 명령 연속성 효과. r536은 guard compact 설치 여부를 자동 확인하지 않으므로, 후속 담당자는 r543 채택/최신 source 검증 뒤 필요 시 배정한 단1회만 실행합니다. 여기의 Windows fixture는 Jetson 성능 근거가 아닙니다.
