# run32 인계 — 정지·재개 뒤 첫 코너 연결 진단 (미실행)

2026-10-25 09:53 KST, Claude Opus 5.5 작성. 사용량 여유 감소(five_hour 0.86)에 따른 GPT 지시로 **실행하지 않고** 인계한다. 이후 설계 검토·승인과 분석은 GPT가, 실행은 GPT-6 Luna가 맡는다. 근거 설계는 [CORNER_AFTER_THREAT_PLAN.md](../CORNER_AFTER_THREAT_PLAN.md)의 GPT 선택 (a)다.

> **후속:** 아래는 인계 당시 원문이다. 이후 GPT 검토와 GPT-6 Luna 실행을 마쳤으며 [최종 결과](README.md)를 따른다. '만료 뒤 해제 경로 없음'은 재관측 없는 상황의 한계로 정정한다. 같은 측면의 새 관측으로 기록이 갱신되는 경로는 E3 §8.17의 GPT 재검토에 남겼다.

## 준비 파일 (sha256)

| 파일 | 내용 | sha256 |
|---|---|---|
| `config/tests/g1_corner_after_threat_v1.json` | 새 case `stop_restart_first_curve`(stop_restart 행동·first_curve 경로·40 s). 앞차 0–4 s 0.35/4–9 s 0/9–40 s 0.35 m/s, 끝 s=58.496. 부모 extensions_v1·g1_traffic_v1 해시 기록 | `b89d309e…4fcbd2` |
| `config/tests/e3_stop_restart_corner_v1.json` | run19/27과 같은 잔디 옆차(+0.32 m, 44.146→49.396). 0–15 s 0.35 m/s 뒤 15–40 s 종점 정지. 부모 해시 기록 | `04a63baa…c5bd50` |
| `../prepare_corner_after_threat_config.py` | 위 두 파일 생성기(있으면 거절) | `0d34823f…70e0` |
| `../corner_after_threat_preflight.py`, `preflight.sh` | 정적 검사 | `c26339d9…da93` |
| `run32.sh`, `files_to_hash.txt` | run27.sh와 같은 실행·회수 절차(중복 실행 거절, 기존 프로세스 있으면 중단, 설치본 해시 확인, 판단 재생·노면 평가) | `79700781…417f` |
| `../corner_window_eval.py` | 사후 창 분리 평가기(**미실행·미검증**) | `2321f22d…3607` |

기존 config/overlay·제어기·평가기·검증기는 수정하지 않았다. 코드 트리는 run27과 같다(첫 묶음 출처 계약 after 해시 일치).

## 확인한 것

- `preflight.json`(exit 0): run27 config 대비 launch·actor·seed·vehicle 차이 0, 초기 배치 동일(앞차 시작 46.246, 옆차 44.146→49.396·+0.32 m)이다. 앞차 코너 경로는 도로·벽 1,226표본(1 cm), 옆차 잔디·벽은 526표본을 통과했고 두 actor 표본 최소 간격은 0.226 m다.
- `bash -n run32.sh` 통과. 새 셸 설치본 해시 4/4 일치(m3_execution_trial·follow_trace·local_pursuit·g1_follow.launch). WSL 시뮬레이션·시험 잔존 프로세스 0.

## 고정 실행 명령 (승인 뒤 1회, 재시도 없음)

```bash
wsl.exe bash -lc 'bash "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-25/m3_execution_review_v1/run32_corner_after_threat/run32.sh"'
```

내부 주행 명령은 run27 옵션에 `--case stop_restart_first_curve --config config/tests/g1_corner_after_threat_v1.json --actor-overlay config/tests/e3_stop_restart_corner_v1.json`을 더한 것이다(`run32.sh` 32행). 종료는 green+40 s case 창 뒤 기존 STOP 절차이며, wall timeout은 2400 s, 예상 wall은 8–10분이다. 완료 표식은 `DONE.txt`(`RUN32_DONE drive_exit=… replay_exit=… surface_exit=…`)다.

## 사후 분해 (실행 뒤)

```bash
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" && set +u && source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && set -u
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests"
python3 -B artifacts/validation/2026-10-25/m3_execution_review_v1/corner_window_eval.py artifacts/validation/2026-10-25/m3_execution_review_v1/run32_corner_after_threat/window_eval.json
python3 -B artifacts/validation/2026-10-25/m3_execution_review_v1/analyze_small_center_run.py artifacts/validation/2026-10-25/m3_execution_review_v1/run27_small_center_stop_restart/run artifacts/validation/2026-10-25/m3_execution_review_v1/run32_corner_after_threat/run --output artifacts/validation/2026-10-25/m3_execution_review_v1/run32_corner_after_threat/compare_run27.json
```

`corner_window_eval.py`는 먼저 run27(15 s case)에 같은 계산을 적용한다. 원래 보고서의 재개 지연·재개 후 진행·최종 간격·정지 유지·gate와 맞지 않으면 assert로 실패한다. 이 재현 확인을 **아직 한 번도 실행하지 않았다.** 실패하면 평가 결과로 쓰지 말고 스크립트를 먼저 고친다. `analyze_small_center_run.py`의 앞차 이동 창은 green+9–15 s로 고정되어 있어 run32 코너 구간에는 맞지 않는다. 위협·직진 우선 집계만 참고한다.

## 판정 해석 (등록 내용)

- 평가기 내장 gate는 40 s 창 전체 값이다(최종 간격·재개 후 진행 포함). 원래 정지·재개 판정으로 읽지 않는다. 정지·재개는 사후 15 s 창, 코너는 내장 `curve_exit_reached`/`curve_heading_changed`와 green+15 s 이후 최장 정지·판단 이유·전 후보 거절로 분리한다.
- 옆차 종점 정지(49.396 m, 잔디)는 새 자극이다. 그 옆을 지날 때의 측방 위협 재촉발과 코너 기하에 의한 정지(직진 예측 편차·곡선 판정·정지 중 꼬리 띠 미지지)를 판단 기록으로 구분한다.
- 결과는 교착 확인/반증용이며 통과해도 대회 성공·전체 랩·코너 병렬주행 근거가 아니다. 성공·실패와 무관하게 재시도하지 않는다.

## 미해결

- 만료 뒤 해제 경로 없음, 정지 중 꼬리 띠 미지지(D7), 0.35 m/s 상한, 측방 약 70 mm, 후방 사각, 실시간 비용.
- 첫 두 묶음 결과는 [E3 §8.16–8.17](../../../../../docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md#817-1025-오전--해제되지-않은-m3가-편차-22-mm첫-코너를-만날-때)과 실행 묶음 README에 있다. 이 인계는 현황·활동 기록에 따로 반영하지 않았다.

## 실행 증거 정정 — run27 재현 래퍼

최초 재현 평가 출력은 `window_precheck.json`에 남아 `matches_original=true`를 보였으나, 실제 Python 프로세스 종료코드는 별도로 캡처되지 않았다. `run27_reproduction_exit_code_inferred.txt`의 `0`은 JSON 결과를 근거로 사후 추정해 기록한 값이며 실측 종료코드가 아니다. 정본 `run27_reproduction_exit_code.txt`는 `unknown`으로 정정했다. 최초 PowerShell→WSL 호출에서 Bash의 `rc=$?`/`exit "$rc"` 상태 전달이 빈 변수로 처리되어 래퍼 종료코드 1과 stderr `bash: line 1: exit: : numeric argument required`가 발생했다. 원시 래퍼 stdout/stderr/exit code와 정확한 명령은 `run27_wrapper_stdout.txt`, `run27_wrapper_stderr.txt`, `run27_wrapper_exit_code.txt`, `run27_wrapper_command.txt`에 보존했다. 검사는 재실행하지 않았다.
