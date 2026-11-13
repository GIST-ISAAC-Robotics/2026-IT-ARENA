# r485 실행 준비 완료

승인된 3개 Python source와 비실행 jetson_access 보조 변경만 확인했다. 실제 import/native 경로 검증·동결은 exit0이며 새 빌드는 없다.

- 입력: 1,444개
- prepared SHA: `216f0465c035f1ece070c6e617704c61b13987385645a31ab3e97f0014265876`
- archive SHA: `b80b6ef6f5b423736b4921a3c577fa4dc4ad3e41273833ed3eed81e37781372e`
- root의 조건부 실행 승인을 `cases/start_left20_yaw_plus3/root_execution_review.json`에 기록했다.
- 같은 +20mm/+3deg 첫 조건을 기존 실행 명령으로 한 번만 수행한다. 원 r477 실패는 보존한다.

초기 헤더 줄바꿈 차이 실패는 `preflight_v1`에 보존했고 원 바이트 복원 후 `preflight_v2`가 완료됐다.
