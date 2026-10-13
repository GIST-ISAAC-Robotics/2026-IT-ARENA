# 분석 v1의 출처 검사 수정

Gazebo 두 실행은 입력·STOP·정지·종료에 통과했지만 최초 후처리 `analysis.json`은 등록 소스 검사 2건에서 실패했다. 시뮬레이션을 다시 실행하거나 원시 결과를 고치지 않는다.

원인은 `registered_manifest.json`의 `sources`를 전체 실행 파일 목록으로 오해한 새 분석기였다. 10/11에서 재사용한 이 목록은 r7 보호/변경 파일 중심이며, 전체 `report.source_sha256`에는 이전 제어기·월드·검사 도구 등 35개 경로가 더 있다. 동일성을 확인하지 않고 이들을 누락 허용하는 방식으로 수정하지 않았다.

수정판은 사전 등록 r7 목록과 **이미 Git에 보존돼 있던 10/11 T2 report의 전체 source_sha256**을 함께 검사한다. 전체 경로 집합/값이 모두 같아야 하며 두 기준에 겹치는 값이 충돌하면 거부한다. 참고 report의 SHA-256은 `359bb526edd60f3eaf15a7e9c1ba6ec6ad5efda9d4d611d4c9976d69a31123be`, Git blob은 `1e0aebcca926fe06dee11f54ab14f061084a9400`이다. 로컬 파일과 현재 HEAD에 저장된 blob이 동일한 것도 확인했다. 실행별 원시 해시·소스 ZIP·r7 필수 출처 검사도 그대로 유지한다.

최초 코드 `analysis_v1.py`, 결과 `analysis.json`, 로그 `analysis.log`와 11개 통과한 당시 단위 시험 로그를 보존한다. 누락/변경/알 수 없는 추가 파일/기준 충돌을 거부하는 회귀를 추가하고 수정 후처리는 `analysis_final.json`으로 별도 저장한다. 시뮬 재실행·source manifest 재작성·판정선 완화는 없다.

좌우 속도 차이의 `within=false`는 두 노면의 물리 차이가 기존 해석 허용치를 넘었다는 **관측 결과**이지, 입력/출처 비교의 실패가 아니다. 이것을 `surface_comparison_valid`와 혼동하지 않는다.
