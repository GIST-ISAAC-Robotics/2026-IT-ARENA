# r386 최종 기하 revision 고정 인계

준비만 완료했다. 원r386 archive는 미실행으로 보존하고 새 묶음만 지정 담당자가 배정 후 실행한다. 공통 source/장치/Gazebo 실행은 포함하지 않는다.

- `bundle_sources.zip`: 116,879,896bytes, SHA256 `bf53444bbed329082c904d046921656e0035f9c1715c18e6e11812fb48cbfd52`
- `source_manifest.json`: `cf3e0806871de1345a113aaa3f2c1552e6d0ad82f0fb2b1744f777d2a35f8a3f`
- `execute_isolated.py`: `b24e723b1cce76fd2cf1d69918eb297106b31170225647c3a24a9740537c9526`
- 원문 candidate: `befa39afe1d8036059de6fccbdb86d0539332813bb472f793dd0ec9c9f28d791`
- 원문 budget: `590bfcbb72f52589c57a6aa4f7db22b51c5aa15676ca19fb7432c58f96ff74c2`
- 원r386 archive: `4a116fd85b60a9ece7bb9dbdeb197aa872314f595b7219ed9b0b3dafe025ea04`
- 승인r388 r1 archive: `f0d6b8496b02a41686adef66f62114e21b98c77d928cc9e780f8a35524a25a40`

고정 WSL 명령:

```bash
python3 -B '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/current_corner_budget_r386_v1/revision_final_geometry_v1/execute_isolated.py' --archive '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/current_corner_budget_r386_v1/revision_final_geometry_v1/bundle_sources.zip' --sha256 bf53444bbed329082c904d046921656e0035f9c1715c18e6e11812fb48cbfd52 --root "$HOME/it-arena-r386-final-geometry-20261113-a"
```

native1회120s→원후보26+영구기하29 순수검사(55예상)1회120s→frozen795/legacy795/final223+15 각1회(각900s)→aggregate120s다. 모든 단계는 직렬이다. 공유 build/install을 사용하지 않는다. 기존 루트/ZIP이 있으면 중단하고 임의 삭제·재시도하지 않는다. 실패하면 이후단계 중단, 자기 자식 process group만 TERM→KILL, 종료코드/PID/로그/전후소스해시를 보존한다.

결과 경로: `$HOME/it-arena-r386-final-geometry-20261113-a/r386/results_native_v1/`. 자동 ZIP은 이 revision의 `bundle_sources-results.zip`. `remote_result/`에 회수한다. `execution.json`, 각단계 로그, `comparison.json`, 세 arm별 `report.json`/`rows.jsonl`을 검토한다. raw행/ZIP은 Git에 넣지 않는다.

원재현223/legacy795/비대상572/15반례STOP와 기하 baseline 효과·후보 효과·추가비용을 분리한다. 결과가 개선0이거나 반례실패라면 그대로 보고한다. 물성/10mm/TTL/목표상수/기하/조향을 바꾸어 통과시키지 않는다. 비용은 단일직렬 PC계측이며 명령 만료/10Hz/안전허가 판정이 아니다.

영구 순수검사 후보는 `staged_tests/test_vehicle_geometry_contract.py` 한 파일이다. 최종 적용 위치는 repo의 `tests/test_vehicle_geometry_contract.py`; artifact/env/저장입력 의존은 없다. 현재 공통 테스트에는 적용하지 않았다.

이 인계문은 archive 생성 후 해시를 기록한 외부 문서이며 inventory에는 포함되지 않는다. 준비 상태는 제어호출0/테스트0/native0/WSL0/장치0이다.
