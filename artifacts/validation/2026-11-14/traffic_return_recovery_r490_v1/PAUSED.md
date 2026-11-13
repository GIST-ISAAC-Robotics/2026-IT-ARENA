# r490 준비 완료 후 사용자 요청 중단

- 준비 호출은 `prepare_entry_v1.json`의 exit 0으로 완료됐습니다(2026-11-13 18:29:32 UTC / 11-14 03:29:32 KST). 새 빌드·회귀·Gazebo 주행은 하지 않았습니다.
- `preflight/result.json`: 실제 5개 제어 모듈과 기존 native 경로/해시 확인 통과. r478 대비 승인한 제어 Python 5파일과 미실행 `scripts/jetson_access.py` 보조 변경만 확인했습니다. r489 독립 보호 변경은 포함하지 않았습니다.
- `preflight/final_check.json`: 동결 입력 1,356개 확인. r478의 actor 속도 .30→.58 m/s, 오른쪽 .12 m, 높이 220 mm, 초기 중심 간격 1.6 m, 10 sim s, 시설 3개 및 기존 판정·STOP 조건을 유지했습니다.
- `prepared.json` SHA256: `9c598ef876fb4fae1203afa24c98ddc71d5832772ccb07f11eb40f658ef064f0`
- `sources.zip` SHA256: `7f9382151fce0ddad70fb6b4ee0353665657e55b4a34956bbc37c68eaf57c5e1`
- 실제 실행 0회입니다. `run1`, `execution_v1/run.pid`, 실행 등록 영수증, `root_execution_review.json`이 없습니다.
- `pause_process_audit.json`: 18:31:52 UTC 종료용 읽기 전용 조회 exit 0, `remaining_r490_processes=[]`. Windows의 남은 `wsl.exe`도 없었습니다. WSL 소유권을 반환하며 추가 명령을 실행하지 않습니다.

## 재개 위치

사용자 명시 재개 후 부모 담당자가 prepared/archive와 현재 입력의 동일성을 확인하고 `root_execution_review.json`을 작성해야 합니다. 그 전에는 실제 주행을 시작하지 않습니다. 재개 시 승인된 단일 실행 명령은 다음과 같습니다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_return_recovery_r490_v1/execution_v1/execute_once.sh'
```

## 선행 r485 인계

`../solo_variation_recovery_r485_v1/RESULT.md`와 `failure_inputs/README.json`에 봉인했습니다. 실제 1회에서 41.98 m까지 진행한 뒤 독립 보호의 `lateral_metric_outside_trial`로 `/drive` 양수와 `/drive/safe=0`이 갈렸습니다. 8 sim s 무진행 기준의 owned STOP으로 회수했고, 원 실패 판정·벽 접촉 0·정상 종료·1,444개 입력 불변·소유 PID 0을 보존했습니다. 최초 입력 묶음은 r489 담당자에게 전달했으며 전체 독립 보호 내부 상태가 모두 기록된 것으로 확대하지 않았습니다.
