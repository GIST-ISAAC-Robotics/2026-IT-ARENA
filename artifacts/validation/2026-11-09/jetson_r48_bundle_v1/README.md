# r48 Jetson 기록재생 최소 묶음 재생성 (2026-11-09)

**r48까지의 현재 코드로 follow-profile 최소 묶음(96파일, 36,292,340바이트)을 기존 생성기로 다시 만들었다. 검사기의 멤버·해시·archive 재폐포 검사를 통과했다. 격리 추출본과 저장소에서 문서상 CLI `profile_follow_trace.py`를 두 trace 전체(1,904 + 548판단)에 각각 1회 실행했고, 명령 열이 모두 같았다. 원 기록 출력과의 차이도 0이었다.** 새 주행·제어 설계가 아니며 생산 소스는 바꾸지 않았다. 실제 Jetson 전송·실행은 하지 않았고, **업로드·전송에는 별도 승인이 필요하다.**

| 항목 | 값 |
|---|---|
| archive (Git 제외 `build/`) | `build/jetson_follow_20261109_r48.tar.gz`, SHA-256 `2023c30cd6bee62065a7c57a29208d7af5a0b9f2b44fe45012bd180801e7ff72` |
| manifest | [bundle_manifest.json](bundle_manifest.json) (SHA-256 `ce41a57e…dcf26`)와 같은 바이트 사본 `build/jetson_follow_20261109_r48.manifest.json` |
| 격리 추출본 | `build/jetson_follow_20261109_r48` (새 폴더, `filter='data'`, 96파일 = manifest) |
| 계획·실행기 | [PLAN.md](PLAN.md), [run.sh](run.sh), [cli_run.py](cli_run.py), [extract.py](extract.py), [compare.py](compare.py), [hashes.py](hashes.py) |
| 결과 | [verify_result.json](verify_result.json), [extract_result.json](extract_result.json), [parity.json](parity.json), `run_*.json`, `commands_*.json.gz`, 저장소 CLI 출력 [cli_repo/](cli_repo/) (추출본 CLI 출력은 추출 폴더 안 같은 상대 경로) |

이전 r42 묶음(v1·v3 archive, manifest)은 덮어쓰지 않았다. 실행 전후 해시가 같다([전](input_hashes_before.json)/[후](input_hashes_after.json)). v3 archive는 `c7a4f615…798df`다.

## 묶음 내용

- 생성 명령: `python3 -B scripts/prepare_jetson_bundle.py --follow-profile-only --follow-trace <r48> --follow-trace <r44> --archive build/jetson_follow_20261109_r48.tar.gz --manifest artifacts/validation/2026-11-09/jetson_r48_bundle_v1/bundle_manifest.json`. 생성기는 manifest를 `artifacts/` 아래에만 쓰므로 `build/` 쪽은 새 파일 복사본이다.
- 폐포: 진입점 6개, 91파일, 외부 의존은 `numpy`·`pytest`뿐이다(r42 v3와 같은 파일 수). 데이터·문서 3개와 원시 trace 2개를 더해 96파일이다.
- r42 v3 manifest 대비 소스 변경 6개: `corner_boundary_support.py`, `lateral_intent.py`, `local_follow.py`, `local_scene.py`, `moving_pass.py`, `pass_release_trial.py`. trace는 이전 4개(r25·r41 원본과 100판단 픽스처)에서 아래 2개로 교체됐다.

| trace (원시, Git 제외) | 행 | 판단 | reset | footer |
|---|---:|---:|---:|---|
| r48 실제 코너 `observed_corner_continuity_r48_v1/run1` | 2,286 | **1,904** | 380 | accepted=written=2,285, dropped 0 |
| r44 빠른 추종 직선 `faster_entry_consistency_r44_v1/straight_moving_lead/run1` | 931 | 548 | 381 | accepted=written=930, dropped 0 |

r48 trace의 실제 판단 수는 1,904다. 작업 지시에 나온 1,891은 사본 코너 기록의 값으로 보이며 이 기록과는 다르다(추정).

## 격리 실행 방법

- 검사: `python3 -B scripts/verify_follow_profile_bundle.py build/jetson_follow_20261109_r48.tar.gz <manifest> --output verify_result.json`의 결과는 `passed`, 멤버 96, 누락·문제 0, 재폐포 91파일이었다.
- 추출: 대상이 workspace `build/` 하위의 절대경로이고 아직 없는 폴더일 때만 만들었다. 기존 폴더는 재사용·덮기·삭제하지 않았다.
- CLI: `python3 -I -B cli_run.py`로 실행했다. PYTHON/ROS/AMENT/COLCON/RMW/DDS 환경을 제거하고 `PYTHONNOUSERSITE=1`로 두었다. cwd는 저장소 루트로 두어 혼입 여부를 함께 시험했다. `sys.path` 앞에는 대상 루트의 `src/arena_autonomy`, `src/arena_vehicle_interface`, `scripts`만 넣었고, 문서상 명령 `scripts/profile_follow_trace.py <trace> --input-mode stream --warmups 0 --repeats 1 --output <새 폴더>`를 `__main__`으로 실행했다. 실행 결과 `sys.flags.isolated=1`이었고, 시작 경로에 cwd는 없었다.
- 명령 비교: 래퍼가 `LocalFollow.command`/`BypassHandoffFollow.command`의 최외곽 반환값을 바꾸지 않은 채, `comparable_decision(freeze(output))`의 해시와 속도·조향·사유를 기록했다. CLI 내부의 `replay_events` 기준 재생 1회와 계측 재생 1회가 모두 기록되어 판단 수의 2배다.

## 결과 ([parity.json](parity.json), 보조 실행 1회 종료 0)

| trace | CLI 종료/passed (저장소·추출본) | 원 기록 일치 | 저장소=추출본 명령 열 | CLI 행(사유·handoff·선택) | 실제 import 프로젝트 모듈 | 스냅샷 |
|---|---|---|---|---|---|---|
| r48 코너 | 0/true · 0/true | 1,904/1,904, 불일치 0 | 동일 (3,808호출, `4a76ae82…10ac4`) | 동일 | 66개, 추출본은 전부 추출 폴더 | 저장소 87(완전) · 추출본 86 + `source_snapshot_omitted: ['tests/test_follow_profile.py']` |
| r44 직선 | 0/true · 0/true | 548/548, 불일치 0 | 동일 (1,096호출, `27ffe3df…8de89`) | 동일 | 60개, 추출본은 전부 추출 폴더 | 같음 |

- 두 trace 모두 현재 r48 코드의 출력이 원 기록과 전부 같았다(허용오차 1e-10, `equal_output`). 별도로 보고할 원출력 차이는 없다. 사유는 전부 `handoff_follow`였고, 양수 속도 판단은 r48 1,753개·r44 490개다.
- 저장소 실행에서도 import된 프로젝트 모듈은 저장소 안에만 있었고, 추출본 실행에서 추출 폴더 밖 프로젝트 파일은 0개였다(`outside: []`).
- 총비용 p50/p95(ms, 저장소 → 추출본): r48 24.66/49.39 → 24.97/50.35, r44 13.83/95.52 → 14.46/93.13이다. 이 값은 WSL2 x86_64·Python 3.12.3·NumPy 1.26.4에서 1회 측정했고 기록 래퍼 부하를 포함하므로 성능 근거로 쓰지 않는다.

## 한계

- 실제 Jetson 전송·Python 3.10/ARM NumPy 실행, ROS 노드·영상·보호층을 포함한 실시간 마감은 미검증이다. 전송·설치는 별도 승인 범위다.
- 이 묶음은 기록 판단의 오프라인 재생 도구다. 새 주행 결과가 아니다. r48 기록은 코너 병렬 0.301 s(0.5 s 목표 미달)의 부분 완료 장면이며, 실물 또는 Jetson 검증 완료를 뜻하지 않는다.
- 정적 폐포에는 실행되지 않은 지연 import도 포함된다(91파일 중 이번 실행의 import 66·60개).
- 이 폴더의 `cli_repo/`(약 1.9 MB: report.json·source_snapshot.zip)와 `commands_*.json.gz`(약 0.5 MB)는 현재 `.gitignore` 대상이 아니다. 원시 trace와 archive는 Git 제외 대상이다(`artifacts/validation/2026-11-0[78]/**/*.jsonl`, `build/`).
