# 산출물과 보존 위치 — 경기 종료 정리

2026-11-14 경기 종료 시 저장소의 코드·문서·산출물을 정리했다. 파일을 다른 실험의 성공 근거로 재분류하거나 원본을 삭제하지 않는다. [경기 결과](COMPETITION_RESULT_2026_11_14.md), [ESP32 최종 기준](../releases/FINAL_ESP32_2026_11_14.md), [후속 시뮬레이션](SIMULATION_WORK_INDEX_2026_11_14.md)은 각각 다른 축이다.

## 찾는 순서

| 자료 | 위치와 해석 |
|---|---|
| ESP32 최종 소스 | [r533 selected_v2 ZIP](../../artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/selected_v2/runtime_handover_sources.zip)·내부734파일 해시. GitHub에서 받을 수 있는 source-only 보존본 |
| 모든 공개 산출물 그룹 | [artifact_index.json](../../artifacts/validation/2026-11-14/repository_closeout_v1/artifact_index.json). `artifacts/`, `assets/`, `outputs/`의 공개 대상 그룹별 파일 수·크기·보고서·manifest 위치 |
| 새로 공개 대상에서 제외한 자료 | [local_only_manifest.json](../../artifacts/validation/2026-11-14/repository_closeout_v1/local_only_manifest.json). 로컬 상대 경로·크기·원본 SHA256. 재설치 가능한 의존성은 해시 대신 분류만 남김 |
| 마지막 준비 묶음과 바이너리 | [bundle_index.json](../../artifacts/validation/2026-11-14/repository_closeout_v1/bundle_index.json). ESP32 공개 소스와 미사용 UNO Q 로컬 ZIP/두 바이너리를 분리 |
| 검증 보고서 | `artifacts/validation/YYYY-MM-DD/<실험>/`의 README/RESULT·판정 JSON·manifest. 날짜/폴더명만으로 성공 여부를 추정하지 않음 |
| 활동·설계·결정 | `docs/activity/`, `docs/hardware/`, `docs/sensors/`, `docs/decisions/`. 당시 사실과 미확정 조건을 보존 |
| 공식 원본 | `assets/`와 각 트랙 감사/manifest. 원본 ZIP을 팀 파생 월드나 새 버전으로 덮어쓰지 않음 |
| CAD·그림·영상 | `assets/hardware/`, `outputs/`, `artifacts/videos/`, `artifacts/promotion/`. 공개 이미지/보고서와 로컬 대용량 STEP/STL·영상을 구분 |
| 종료 전 긴 현황·계획 | [문서 보관본](../archive/2026-11-14-closeout/README.md).4파일을 byte 단위로 보존하고 원래 경로·해시 기록 |

## 공개와 로컬 보존의 경계

코드·설정·보고서·시험 소스·작은 평가 결과·실패 기록·manifest를 Git에 남긴다. 11/14의 원시 센서/고빈도 주행 JSONL, 상세 trace/acquisition, 입력 포함 소스 압축, 재생성 메시와 실행 의존성은 기존 날짜의 제외 정책을 이어 적용한다. `signal_sequence.jsonl` 같은 작은 시험 조건은 계속 공개한다. 기존에 추적하던 파일은 이번 정리에서 강제로 제거하지 않았다.

ESP32 최종 `selected_v2`는 원시 센서가 없는 약2.3MB 소스 ZIP이므로 공개 보존 예외로 선택했다. UNO Q v3 ZIP과 ELF/bin은 `build/`에 로컬로 남고 코드·빌드 도구·관련 검증은 공개된다. 두 종류를 혼동하지 않는다.

`local_only_manifest.json`은 이번에 새로 제외한 프로젝트 자료의 목록이다. 이미 오래전부터 제외하던 모든 원시 로그, 전체 WSL, 도구 캐시, 계정/장치 설정까지 포함한 디스크 백업이 아니다. 원시 입력을 요구하는 재생에는 해당 로컬 파일이 추가로 필요하다. 재설치 의존성·로컬 도구 설정·인증 정보는 공개 대상이 아니다.

큰 STEP/STL 원본은 삭제하지 않고 원래 `outputs/steering_cad_2026_11_13/` 위치와 해시를 보존한다. GitHub에서 이 목록을 보았다고 해당 대용량 파일도 내려받을 수 있는 것은 아니다. 작은 도면 SVG/PNG·핀별 표·CAD 비교 보고서는 계속 찾아볼 수 있다.

## 목록 재생성

목록은 [catalog_repository_closeout.py](../../scripts/catalog_repository_closeout.py)로 만들었다. 삭제·이동·Git 명령에 의한 stage/commit·장치 접근은 하지 않는다. 정리 시작 당시 로컬 `build/repository_closeout_20261114/untracked_before.json`이 있어야 같은 신규 제외 범위를 계산할 수 있다.

공개 목록은 결과 자체가 아니라 **증거를 찾기 위한 색인**이다. 실험 당시 소스·조건·성공/실패·정상 종료 여부는 링크된 원 보고서가 기준이다. 마지막 소스와 예전 성공을 결합해 새로운 완주 결과를 만들지 않는다.
