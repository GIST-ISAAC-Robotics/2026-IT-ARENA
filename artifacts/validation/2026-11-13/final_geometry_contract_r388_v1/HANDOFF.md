# r388 고정 실행 인계

준비 상태: controller 호출0, 테스트/빌드/WSL/Jetson/시뮬/장치 실행0. 주 에이전트가 자원 담당자에게 배정한 뒤에만 아래1회를 실행한다. r386은 보류한다.

- archive `bundle_sources.zip`: 2,735,368 bytes, SHA256 `223c2df288f25f2147b78ef26a47bc235ee7929cd3be3b346ec43fae0bab265e`
- `source_manifest.json`: SHA256 `52d5394c0dbc45f193be186ae69d5f9b6e3b5ce04d8e59394c1d1666fbfac992`
- `execute_isolated.py`: SHA256 `76a0eba0c7d96aedea30596062a1f9267b89f9e83a4536c9fd2fe457196c79db`
- `geometry_contract.patch`: SHA256 `eac5c527de4aeaf9a68ecccfda7cda34a97ac41ff9d8419605fc508d4ff1efaf`
- bundle543파일, 원r377소스256파일/두arm, geometry변경9파일. 원후보 계보 변경0. 원trace 행의 별도 사본만 포함해 새 번들은 약2.7MB다.

WSL 안에서 고정 명령:

```bash
python3 -B '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/final_geometry_contract_r388_v1/execute_isolated.py' --archive '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/final_geometry_contract_r388_v1/bundle_sources.zip' --sha256 223c2df288f25f2147b78ef26a47bc235ee7929cd3be3b346ec43fae0bab265e --root "$HOME/it-arena-r388-geometry-20261113-a"
```

공유 `build/install` 사용 없음. 새 root와 결과 ZIP이 이미 있으면 중단하며 임의 삭제/덮기/재시도하지 않는다. native1회(120s), 순수회귀1회(120s), frozen/legacy/explicit 마지막 입력 각1회(각120s), aggregate1회(120s). timeout이면 자기 자식 process group만 TERM→KILL하고 결과를 남긴다.

결과는 `$HOME/it-arena-r388-geometry-20261113-a/r388/results_native_v1/`, 단계 로그/PID/종료코드는 `r388/execution.json` 및 `*.log`다. 자동 회수 ZIP은 원 archive 옆 `bundle_sources-results.zip`이다. `remote_result/`에 회수해 `execution.json`, `comparison.json`, arm별 전체 `report.json`을 검토한다. `queries.npz`는 원시 질의좌표 로컬 보존용이며 Git에 넣지 않는다.

원 기대값 재현/legacy동일/입력·소스 불변 실패는 실패 그대로 보고한다. 새기하 arm에서 정지해도 시험을 통과시키려고 조건을 바꾸지 않는다. 재현이 깨져도 만들어진 출력은 보존하며 원인 검토 후에만 새 실행을 배정한다.

준비 과정에서 한 번 `config/hardware/g1_input_shadow_v1.yaml`이 r377 소스 ZIP에 없음을 엄격 목록 검사로 확인해 준비가 중단됐다. 이 YAML은 controller 비교/import에 필요하지 않고 preview 순수명령 테스트도 파일을 열지 않는다. 임의 현소스를 섞지 않고 해당 비필수 수집을 제외한 뒤 첫 archive를 만들었다. 제어 호출/회귀 실행은 없었다. 이 인계문은 archive 생성 후 해시를 기록한 외부 문서이며 archive inventory에는 포함되지 않는다.
