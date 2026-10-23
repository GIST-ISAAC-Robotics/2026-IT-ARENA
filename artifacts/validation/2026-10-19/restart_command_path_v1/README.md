# 10/19 제어기 재시작 중 MCU 고정 — 원인 분리·시험기 수정·진단 실행

기준 문서: [RGB·프로세스 중단과 정지·재출발 경계 §7](../../../../docs/autonomy/PIPELINE_FAULT_VALIDATION_2026_10_04.md#7-1019-제어기-재시작-중-mcu-고정--v5v6-원인-분리와-시험기-수정). 이 폴더는 원시 산출물과 재현 명령만 둔다.

| 경로 | 내용 |
|---|---|
| `baseline/`, `baseline_sha256.txt` | 수정 전(10/17 v6 실행과 같은 hash) 검사기·MCU 거절 진단·구동 계약/ROS·관련 시험 사본 |
| `validator_change.diff` | 검사기 최종 변경 diff(baseline 대비) |
| `v9_to_final_validator.diff`, `v9_to_final_probe.diff` | v9 실행 snapshot 대비 최종본 차이(시작 그래프 `startup` 정책, 호스트 진단 호출 전 값 복사) |
| `line_ending_note.txt` | v7/v8 실행 당시 CRLF였던 두 파일의 hash와 LF 변환 hash |
| `ros_v7/` | 진단 포함 첫 실행. 시작 그래프 감사의 `NodeNameNonExistentError`로 사례 전 실패(보존) |
| `ros_v8/` | 그래프 수정 뒤 1회. 43/43 통과. 2번째 재시작의 임계 미만 동시 정지 기록 포함 |
| `ros_v9/` | 느린 발행·spin·스레드 표본 추가 뒤 1회. 43/43 통과, 정지 재발 없음 |
| `analyze_restart_trace.py`, `ros_v*/restart_trace_summary.json` | 읽기 전용 요약 도구와 결과. 판정이 아니라 시간 순서 정리 |
| `regression_full/` | v9 실행 소스와 같은 소스의 전체 회귀(3,041개) |
| `probe_field_note.txt` | v7~v9 호스트 로그 `last_source_us_before`가 실제로는 호출 후 값이었다는 정정 메모(원시 로그 미수정) |
| `regression_related_after_review/` | 1차 검토 정정(호스트 진단 호출 전 값 복사) 뒤 관련 회귀(158개). 중간 상태 기록 |
| `regression_related_final/` | 시작 그래프 정책(`startup=True`, 발견 준비 부족만 대기)까지 반영한 최종본 관련 회귀. v9 소스와의 차이는 기준 문서 §7.5에 정리. 정정본 ROS 미실행 |

각 `ros_v*/run.sh`가 실행 명령·소스 hash 전후·snapshot·`/proc` 프로세스표를 남긴다. ROS 실행·대기·회수는 Claude Haiku 5.5 서브에이전트(도구가 전체 ID를 거부해 별칭 `haiku` 사용, 자기 보고·결과 metadata `claude-haiku-5-5`), 분석·구현은 Claude Opus 5.5가 맡았다. 실행 전 주 에이전트가 사용자 승인으로 Windows 전원 구성표와 Armoury Crate 모드를 터보로 바꿨다(환경 기록일 뿐 인과 주장 아님).

`restart_traces.json`의 시각 의미: `wall_s`/`sim_s`는 **검사기가 spin에서 읽은 시각**이고 `source_sim_s`·`now_us`는 발행 노드 자신의 시계다. `/proc` 표본은 검사기 tick에서 읽은 누적 계수이므로 두 표본 사이 증가량만 해석한다. 자식 로그의 `SLOW_PUBLISH`·`MCU_FAULT_CONTEXT`는 발생한 프로세스 안에서 측정하고 측정 뒤에 파일로 쓴다.
