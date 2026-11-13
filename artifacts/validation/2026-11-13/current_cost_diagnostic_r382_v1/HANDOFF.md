# r382 실행 담당자 인계 — 동결본

새 archive는 `bundle_sources.zip`, bootstrap은 `remote_bootstrap.py`다. 두 파일을 아래 위치로 복사하고 해시를 확인한 뒤, 주 에이전트가 Jetson 소유권을 배정한 담당자만 실행한다. 원 r380 폴더/결과를 재사용하거나 덮어쓰지 않는다.

| 항목 | 값 |
|---|---|
| archive bytes | 83,802,236 |
| archive SHA256 | `462f962f66f4891ce85fa0e1b637ec57f7c719ecba723516c94b97298f262e16` |
| manifest SHA256 | `081056cd120378920437728d791fbda531131392933a9e25f245fdf74890eda8` |
| bootstrap SHA256 | `117fda294d625103ee669d09f33756ca086c2cb4985cd77abbde914a75084194` |
| 원 r380 archive SHA256 | `122f13672bdac590285bc4bf4bc15be0341158dac0849d9f65a7c916c8530a44` |
| 전체 파일 / 원본 | 278 / 원266파일 바이트 모두 동일 |
| 동적 후보 | r375→r370→r352→r346→r343→11/12 r281→r266, 7개 해시 포함 |

원격 archive: `/home/q/it-arena-r382-20261113.zip`

원격 bootstrap: `/home/q/r382-bootstrap.py`

새 루트: `/home/q/it-arena-r382-20261113` — 존재하면 재사용하지 않고 중단한다.

```bash
/usr/bin/python3 /home/q/r382-bootstrap.py --archive /home/q/it-arena-r382-20261113.zip --sha256 462f962f66f4891ce85fa0e1b637ec57f7c719ecba723516c94b97298f262e16 --root /home/q/it-arena-r382-20261113 --build-native
```

진행은 native 빌드120초 → import60초 → 저장572+반례17 단일 직렬 계측900초 → 계측 종료 후 index9 관찰60초다. 비용 계측 중 동일 Jetson 경쟁 부하를 추가하지 않는다. 센서/ROS/Gazebo/모터/서보/펌웨어/전력 설정 변경은 없다. timeout이면 해당 bootstrap 소유 자식 process group만 회수한다.

**exit0은 진단 수집 완료이며 x86 재현 통과가 아니다.** `execution.json`과 `jetson/report.json`의 `diagnostic_completed`, `same_arm_invariants_passed`, `x86_existing_comparator_reproduction_passed`, `applied_count`, `applied_count_matches_x86`, `changed_sources`를 각각 회수한다. x86 차이만으로 전체를 중단하지 않지만 ARM 내부 불변/비대상/17반례/해시/예외는 즉시 실패한다. 원 r380 실패는 그대로 남는다.

결과 archive는 `/home/q/it-arena-r382-20261113-results.zip`이다. 단계 로그, `execution.json`, `import_preflight.json`, `jetson/report.json`, `timing_rows.jsonl`, `comparison_rows.jsonl`, `query_probe/report.json`, `intermediate_arrays.npz`를 회수한다. query probe의 출력은 비계측 native 출력과 exact 동일해야 한다. 원인 확정 여부는 별도이며 ARM 관찰만으로 architecture 차이를 단정하지 않는다.

준비 검증: Windows 로컬의 진단 논리 7검사/AST/문서 링크 검사 통과, archive 내부278해시 대조, 독립 추출본 Python import 사전 검사278파일·7계보·rebase0 통과. native 빌드/589호출/원격 실행은 수행하지 않았다. 현재 작업 트리의 r379 공통 변경은 이 번들에 포함하지 않았다. 자세한 판정과 원 실패 분석은 [README](README.md), 영수증은 [bundle_receipt.json](bundle_receipt.json)이다.
