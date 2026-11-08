# 새 코스 단독 프리셋 정적 검사 (2026-11-09)

## 범위

루트 README의 현재 구성·시작 명령을 공식 v2026.11.06·B0390/C1·JR60P20 기준으로 바로잡고, 단독 본선 조합을 이름 있는 프리셋 `scripts/run_new_course_solo.py --preset new-course-solo`로 묶었다. Gazebo·녹화·주행은 실행하지 않았다.

`scripts/run_current_solo.py`는 고치지 않았다. 이 파일은 r11·r14·r15·r19 등 소스 해시 계보와 r25 `registered.json`의 `sources`에 포함되어 있어, 수정하면 과거 계약을 바꾸거나 우회해야 한다. 새 진입점은 기존 진입점을 `--print-args`로 호출해 인자를 받으므로 인자 목록을 따로 복제하지 않는다. 기존 무옵션 실행(v2026.11.01·B0183)과 `--previous-jr50`은 그대로다. 결과 경로는 호출 위치 기준으로 한 번 절대화하여, 존재 검사·출력·실행에 같은 경로를 쓴다. 실행에도 출력·대조한 최종 인자를 그대로 사용한다. `--print-args`와 `--check-r25`는 함께 지정할 수 없다.

## 결과

| 검사 | 결과 |
|---|---|
| `build_official_track.py --check --profile config/tracks/official_v2026.11.06.yaml` | 종료 0, `status: ok`(원본 691,524 B·25항목, 실행 월드 17파일) — [track_check.log](track_check.log) |
| `--check-r25` | 종료 0, 옵션 33개, `differences: []` — [check_r25.json](check_r25.json) |
| `--print-args` | 종료 0, 결과 경로 미생성 — [print_args.json](print_args.json) |
| 관련 회귀 5파일(검토 반영 후 재실행) | `36 passed` — [pytest.log](pytest.log) |

대조 기준은 [r25 registered.json](../../2026-11-08/solo_gap_envelope_connected_v3/registered.json)의 `args`다. 경로 구분자, 로컬 절대 센서 경로(저장소 상대로), `--output` 값만 정상화하고 나머지 옵션·값은 그대로 비교한다. 옵션 순서는 비교하지 않는다. `--output`·`--local-sensor-config` 값이 없으면 숨기지 않고 오류로 처리한다. r25 등록 인자에서는 `--g1-solo-corner-approach`가 마지막 옵션이다. 중복 옵션은 오류로 처리한다.

## 한계

- 일치를 확인한 것은 **실행 인자**뿐이다. 현재 소스가 r25 당시와 바이트 단위로 같은지는 확인하지 않았다. r25의 계측 기록기(`run.py`의 바퀴 접촉·구동 토픽 기록), 물성, 실제 실행 결과도 같다고 볼 수 없다.
- 교통 프로필은 이 프리셋에 포함하지 않았다.
- 재실행: WSL에서 `bash artifacts/validation/2026-11-09/new_course_quickstart_v1/checks.sh`(같은 로그를 덮어씀).
