# r48 Jetson 기록재생 최소 묶음 재생성 계획 (2026-11-09)

목적: r42 이후 r43–r48 소스가 빠진 follow-profile 묶음을 현재 코드로 다시 만들고, 격리 추출본과 저장소에서 같은 문서상 CLI를 전체 trace에 1회씩 실행해 명령 동일성을 확인한다. 새 주행·제어 설계, 생산 소스 수정, Jetson 전송은 하지 않는다.

## 입력 (읽기만 함)

| 구분 | 경로 | 판단 수 |
|---|---|---:|
| r48 실제 코너 주행 | `artifacts/validation/2026-11-08/observed_corner_continuity_r48_v1/run1/follow_decisions.jsonl` | 1,904 (reset 380, footer accepted=written=2,285) |
| r44 빠른 추종 직선 | `artifacts/validation/2026-11-08/faster_entry_consistency_r44_v1/straight_moving_lead/run1/follow_decisions.jsonl` | 548 (reset 381, footer accepted=written=930) |

두 기록 모두 `g1_g2_handoff_decision_stage_only`, schema 5다. 실행 전후 SHA-256과 이전 r42 archive·manifest 해시를 `input_hashes_*.json`에 남긴다.

## 순서 ([run.sh](run.sh), 보조 실행 1회)

1. `prepare_jetson_bundle.py --follow-profile-only` + 위 두 `--follow-trace` → `build/jetson_follow_20261109_r48.tar.gz`. 생성기는 manifest를 `artifacts/` 아래에만 쓰므로 [bundle_manifest.json](bundle_manifest.json)에 만든 뒤 `build/jetson_follow_20261109_r48.manifest.json`으로 새 파일 복사한다.
2. `verify_follow_profile_bundle.py`로 archive 멤버·해시·재폐포 검사 → `verify_result.json`.
3. [extract.py](extract.py): 대상 `build/jetson_follow_20261109_r48`가 workspace `build/` 하위이고 존재하지 않을 때만 만들고 `filter='data'`로 추출, 추출 파일을 manifest와 대조한다.
4. [cli_run.py](cli_run.py): `python3 -I -B`, PYTHON/ROS/AMENT/COLCON/RMW/DDS 환경 제거, cwd는 저장소 루트(혼입 시험)로 두고, `sys.path` 앞에 대상 루트의 `src/arena_autonomy`, `src/arena_vehicle_interface`, `scripts`만 넣는다. 문서상 CLI `scripts/profile_follow_trace.py <trace> --input-mode stream --warmups 0 --repeats 1 --output <새 폴더>`를 `__main__`으로 실행한다. 출력에 영향을 주지 않는 기록 래퍼로 최외곽 `command()` 출력(`comparable_decision(freeze(...))`)의 해시와 속도·조향·사유를 남기고, 실행 후 실제 import된 프로젝트 모듈 파일 위치를 검사한다. 저장소·추출본 × 두 trace = 4회.
5. [compare.py](compare.py): 저장소/추출본 명령 열·CLI 행(사유·handoff 상태·선택)·원 기록 일치 수·스냅샷 생략 표시를 비교해 `parity.json`을 쓴다.

## 판정선

- 검사기 `passed`, 추출 파일 = manifest.
- 각 CLI 종료 0, `passed`, `baseline_matched` = 판단 수, `output_mismatches` 0.
- 저장소 = 추출본 명령 열(해시 전부 일치), CLI 행 동일.
- 추출본 프로젝트 모듈 전부 추출 폴더 내부, 추출본 보고서의 `source_snapshot_omitted == ['tests/test_follow_profile.py']`.
- 입력 trace·r42 archive 해시 불변.

비용 수치는 단일 PC(WSL2) 1회 값이며 기록 래퍼 부하가 포함되므로 성능 근거로 쓰지 않는다.
