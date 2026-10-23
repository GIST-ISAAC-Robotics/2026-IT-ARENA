# 방지턱 기하·전방 광선 v2 — 저장 실패 보존

23회귀 통과 뒤 분석 결과 JSON 직렬화에서 `numpy.bool_` 오류로 exit 1. `report.json`은 **1,484 B의 부분 파일이며 유효한 분석 결과가 아니다**. 실행 실패와 당시 소스·로그를 보존한다. [실패 로그](study.log), [시험 로그](tests.log), [명령](run_checks.sh). 소스 해시 차이는 0 B. 수정은 [v3](../bump_geometry_v3/README.md)에 별도로 수행했다.
