# 분리 관측 벽 조각 — 오프라인 증거 v1

`run.sh` exit 0, 관련 회귀 111개 통과. `run/report.json`의 기존 판단 368/368개가 재생 일치했다. fresh 장면 74개 모두 실제 조각의 지지 조건을 충족했다. 원래 경계 미확인 HOLD 중 지지한 장면은 23개였다.

원시 입력 `follow_overlap_v1/side_left_s0/follow_decisions.jsonl` SHA-256:
`c767135d5de762261aff4a8619622c05385b669be2ee11c59245959a0ce97daa`.
입력/소스 해시 불변, 시험 당시 소스는 `run/source_snapshot.zip`에 보존했다. 이후 함수가 운영 패키지의 선택형 후보 모듈로 이동했지만 이 보존본을 소급 변경하지 않았다.

가림 간격을 관측 길이나 자유 공간으로 채우지 않고 모든 점유를 보존하는 증거 검사다. 이 v1은 새 주행 명령을 내거나 차량을 움직이지 않았다. [설계·다음 연결](../../../../../docs/autonomy/G1_FRAGMENTED_BOUNDARY_2026_10_13.md).
