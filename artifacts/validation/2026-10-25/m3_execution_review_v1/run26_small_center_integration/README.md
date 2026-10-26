# run26 — 작은 중앙 보정 생략 옵션의 연결·관련 회귀·빌드

Claude Opus 5.5가 실행했다(`run26.sh`). Gazebo/ROS 주행은 없다.

- **변경:** 기본 false `ExecutionConfig.small_center_straight`(관측 기반 복귀 필수), 노드 `g1_m3_small_center_straight`, launch 인자, 시험 CLI `--m3-small-center-straight`(M3·`--m3-release-to-baseline` 필수). 새 출처 계약 [`m3_small_center_r0_sha256.json`](../../../../../config/tests/m3_small_center_r0_sha256.json)은 run19 실행 사본 ZIP 바이트에서 현재 4파일로 잇는다. 이전 계약·실행 사본은 수정하지 않았고, 사슬 목록을 쓰는 시험 4개에 새 계약 이름만 덧붙였다(`../run25_center_hold_compare/extend_chain_lists.py`). release-gate 계약 시험의 현재 바이트 대조는 다른 후속 계약과 같은 방식으로 새 계약을 되돌려 확인한다.
- **관련 회귀:** 14파일 **281통과**(`pytest.xml`, 신규 `tests/test_m3_small_center_straight.py` 17건 포함). 신규 시험은 방향 오차·곡선(반경 1.2/4 m)·누적 드리프트·90 mm 복귀·직진 명령 거절 뒤 기존 순서·전방 unknown 비면제·위협 기록/복귀 관문 ON=OFF·두 번째 측방 물체·세대 단절 HOLD·앞차 재정지의 같은 종방향 제한을 확인한다. 합성 장면 결과이며 주행 안전 증명이 아니다.
- **빌드·설치본:** `arena_autonomy`·`arena_bringup` 빌드 성공, 새 셸에서 `m3_execution_trial`·`follow_trace`·`local_pursuit`·`g1_follow.launch.py` 설치본 해시가 소스와 일치(`install_import_results.json`).
- **기본 OFF 재생:** run13 637/637, run9 636/636 원래 판단과 같다. 입력 해시 불변.
- 실행 전후 소스·시험·기록 해시 차이 0, 잔존 프로세스 0.
