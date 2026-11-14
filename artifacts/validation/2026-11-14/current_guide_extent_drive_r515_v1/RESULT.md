# r515 현재 전방 외벽 범위 후보의 오른쪽 옆차 대표 1회

2026-11-14, GPT. r513과 같은 `right_side_start` 12 sim초를 r514 공통 후보로 단1회 실행했다. 이전 2.45 m 지속 정체는 이번 실행에서 사라졌고 원 평가 진행은 6.01 m다. **원 평가는 `inside_road`로 실패했고, 별도 추월·복귀 목표도 실패했다.** 재실행이나 판정 변경은 하지 않았다.

## 입력과 실행

- [실행 조건](PLAN.md): 상대 +0.50 m/오른쪽 −0.23 m, 0.50 m/s, 200×150×220 mm, RTF 목표0.25, `g1_open_1p5`/`medium_v1`, 시설3, GREEN 이후12 sim초. 기하601표본과 native 바이너리를 재사용했다. Jetson·센서·실물 출력·잔디 우선순위 변경은 포함하지 않았다.
- r513 공통 동결 대비 변경은 `moving_pass.py`(`bcd58132`→`699a9ebe`)와 비제어 시험 `tests/test_moving_path_preview.py`(`e956d5ed`→`3bd59805`)뿐이다. [r514 근거](../current_guide_extent_r514_v1/validation.json)의 관련130검사는 root가 완료했으며 실행 담당은 반복하지 않았다.
- 실제 import·공통1,332파일 대조·경쟁 프로세스0을 확인한 뒤 총1,421입력을 동결했다. `prepared.json` SHA `36d3b4d1f8f8e7d076c993a692b8a6a426eab87f0782390e3eb780d15983f369`, `sources.zip` SHA `e693beed72e5b62a1817138c8394c988e8199734111cd7e1adb3fbf1de26033a`. ZIP 내부1,420파일도 동결 해시와 일치했다.
- [검토 영수증](root_execution_review.json)은 root의 recipe·후보 SHA 검토와 실행 담당의 동결 ZIP·입력 검사 범위를 구분한다. root가 ZIP 내부를 직접 대조했다고 기록하지 않았다.
- 실행 UTC02:48:41–02:53:08, wall267초, owner PID8020. GREEN7.401–19.401 sim초. 원 validator exit1과 정상 수집 exit0을 구분한다. 감시 STOP 조건은 도달하지 않았고 원 평가기가 명시 STOP과 정상 종료를 수행했다.

## 결과

| 항목 | 결과 |
|---|---|
| 원 판정 | FAIL: `inside_road`만 실패. `progress`·`no_permanent_stall`·`final_gap` 포함 나머지 원 관문 통과 |
| 진행·정지 | 원 dense 평가6.010 m, 상대 이동 중 최장정지0.304 sim초 |
| 별도 목표 | FAIL: `actually_alongside=false`, `passed_actor`/`final_follow`/`rejoined_main` 실패. `no_long_stall`/`post_return_progress` 통과 |
| 별도 표본 지표 | 최종5.937753 m, 최장정지0.606 sim초, 최종 `MOVING_PASS`, 중심선 편차0.153877 m |
| 상대 간격 | 표본 최소0.213327 m, swept 하한0.213121 m, 표본겹침/actor접촉0 |
| 벽·도로 | 벽접촉0. 원 차체 도로밖123표본. 현 CAD 바퀴중심 도로밖0. 접촉면의 일부라도 도로밖인 기준은 최대1바퀴·0.098 sim초 |
| 속도·전달 | 최고0.934379 m/s, 명령만료0 |
| STOP·회수 | STOP6항목 통과, 14자식 정상종료, 강제정리 없음, 잔여 소유PID0 |
| 무결성 | 실행 전후1,421입력 변경0, ZIP불변, 원 결과 SHA불변 |

원 dense 진행과 별도 trajectory 표본의 최종 진행·정지값은 집계가 다르므로 섞지 않는다. 원 도로 판정을 바퀴 중심 판정으로 덮어쓰지 않았다. 접촉면·바퀴 중심은 계산 지표이며 실제 타이어 접촉이나 심판 판정이 아니다.

## 기록에서 확인한 변화와 남은 문제

[원 로그 발췌](failure_inputs/summary.json)의 창 안 plan call165행에서 `handoff_follow`98, `moving_pass_entry`34, `moving_pass_current_follow_return`28, `moving_pass_current_path_unavailable`5회였다. r513의 `entry_guide_current_wall_too_short`는0회다.

일시 정지5회는 runtime seq104–108, sim12.62–13.02초의 `rejoin` 단계에서 `no_entry_command_candidate`/`known_occupied_command_hold`로 기록됐다. 이후 양수 명령으로 복귀했다. 이 작업은 원 로그만 발췌했으며 어떤 원시 반환이 점유 거절을 만들었는지는 재생·귀속하지 않았다. 최종에는 다시 `moving_pass_entry`였고 상대를 완전히 추월하거나 정상 추종으로 최종 복귀하지 못했다.

일시 정지 주변과 마지막5계획의 원문 JSONL 바이트를 로컬에 보존했다. 앞선 영구정지용 `extract_failure.py`는 이번 결과에 적용하지 않았고, [관측 요약 도구](execution_v1/summarize_observed.py)로 별도 회수했다. 이전 r499/r503/r513 실패와 원시 입력은 수정하지 않았다.

## 자료와 범위

- [원 결과](run1/report.json), [실행 요약](execution_summary.json), [별도 목표](assessment_summary.json), [노면 기하](surface_run1.json).
- [입력 무결성](integrity.json), [실제 import](preflight/result.json), [동결](preflight/freeze.json), [종료 후 입력 검사](execution_v1/check_after_run.json), [소유 프로세스 회수](execution_v1/owned_processes_after.json).
- WSL 실행 소유권은 정상 회수 후 root에 반환했다. 이후 문서·발췌는 Windows의 저장 로그만 사용했다. 생산 코드·공통 문서·커밋·푸시는 변경하지 않았다.

이는 220 mm 가시 상대 가정의 같은 장면 한 번에서 지속 정체가 해소된 근거다. 추월 완료·전체 교통 랩·낮은 차량·실차·실시간10 Hz의 성공 근거는 아니다. 후속 제어 판단은 root가 맡는다.
