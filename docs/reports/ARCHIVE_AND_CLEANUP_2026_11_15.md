# GitHub 보존과 로컬 정리 — 2026-11-15

사용자는 보존할 자료를 GitHub에서 복구할 수 있게 한 뒤 로컬 저장소를 삭제해도 필요한 자료가 유실되지 않도록 요청했다. 최종 소스·전체 설계·선정 영상과 실제 센서/차량 명령/ESP32 사건 원문을 보존하고, 미선정 반복 시뮬레이션의 상세 로그와 재생성 환경을 정리했다.

## 어디서 복구하는가

| 자료 | 보존 위치 |
|---|---|
| 코드·설정·실험 요약·실패 판정·문서·공식 코스 원본 | 공개 저장소 `main`과 Git 이력 |
| 대회용 최종 ESP32 | [고정734파일 소스와 복원](../releases/FINAL_ESP32_2026_11_14.md). 현재 main의 후속 연구와 구분 |
| 최종 전체 차량 CAD·ESP32 전체 회로도 | [공개 설계 릴리스](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/releases/tag/final-design-2026.11.14) · [파일 해시](../../artifacts/validation/2026-11-15/final_design_archive_v1/manifest.json). 11/14 제작 기준 STEP이며 실제 조립 상태 확정이나 통전 검증이 아님 |
| 성공 영상8편·최소 재렌더 입력 | [공개 영상](SIMULATION_RECORDINGS_2026_11_15.md) · [재렌더 입력](../../artifacts/validation/2026-11-15/replay_preservation_v1/README.md) |
| 실제 센서·차량 명령·ESP32 사건·CAD 이전본·미사용 UNO Q 준비본 | [비공개 증거 저장소](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA-evidence)의 [보존 릴리스](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA-evidence/releases/tag/archive-2026-11-15). 원문9묶음, 799,784,228 bytes |
| 선정 실행7개와 r544 실패1개의 비교 원문 | 같은 비공개 릴리스의 `representative-simulation-originals.zip`. 전체 과거 실험의 상세로그를 남긴 것은 아님 |

비공개 원본에는 실내 영상/점군·장치 식별자·플래시 백업이 포함된다. 공개 main에는 정제한 진단 근거와 복원 안내를 두며, 비공개 원문은 해당 조직 저장소 접근 권한이 있어야 받을 수 있다. 계정·장치 인증이나 시스템 패키지·전체 WSL 환경의 백업을 의미하지 않는다.

## 새 PC에서 시작

```powershell
git clone https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA.git
gh release download final-design-2026.11.14 --repo GIST-ISAAC-Robotics/2026-IT-ARENA --dir ./vehicle-design
gh repo clone GIST-ISAAC-Robotics/2026-IT-ARENA-evidence
gh release download archive-2026-11-15 --repo GIST-ISAAC-Robotics/2026-IT-ARENA-evidence --dir ./physical-evidence
```

비공개 저장소의 `archive-manifest.json`으로 ZIP/tar SHA-256과 멤버별 해시·원래 상대 경로를 확인한다. 별도 폴더에 압축을 풀면 당시 `build/`·시험 경로를 복원할 수 있다. 내부에 들어 있는 원 tar/zip도 그대로 보존했다. ESP32 사건의 `stage-path.txt`, 원격 `mounted-dc-test.5qyIBL` 83파일과 추가57파일은 이 경로로 찾는다. [사건 분석](../hardware/ESP32_FAILURE_REVIEW_2026_11_15.md)을 함께 읽는다.

소프트웨어 재개에는 [보관 실행 절차](../simulation/ARCHIVED_RUNBOOK_2026_11_15.md)에 따라 ROS/Gazebo/ESP-IDF 등 의존성과 빌드를 다시 준비한다. 보존 소스는 장치에 즉시 안전하게 설치 가능한 완성 펌웨어나 고장 해소 증명이 아니다.

## 정리한 자료와 검증

- Git 제외 파일 **65,146개 / 77,465,561,102 bytes(77.47 GB)**: 미선정 상세 시뮬레이션·중간 산출물69.86 GB, 빌드/캐시4.73 GB, 원격 백업으로 대체한 사본2.53 GB, 설계/렌더 중간물0.34 GB.
- 공개 main의 옛 상세 시뮬 JSONL **131파일 / 414,402,684 bytes**도 제거했다. 보고서·요약·소스·시험 fixture는 유지하며, 이 파일은 기존 Git 커밋 `561cfac9161cc17ce6f2c64351f9983d762a7624`에서 복원할 수 있다. [파일별 해시와 과거 경로](../../artifacts/validation/2026-11-15/github_preservation_v2/tracked_detail_cleanup.json). Git 이력을 다시 쓰지는 않았다.
- 폴더 경계·Git 추적 여부·크기·수정시각을 대조하고 원본 보존 대상1,749파일은 ZIP 멤버 SHA-256도 확인했다. 원격9묶음 및 공개 최종 설계 ZIP은 다시 다운로드한 전체 바이트의 SHA-256으로 검증했다.
- 공용 런타임 캐시로 연결된8,529파일/375.56 MB는 저장소 밖 자료이므로 삭제하지 않았다. 연결을 포함한 생성 링크399개만 제거했다.
- `.codex/config.toml`48 bytes는 기기별 실행 설정이다. `build/official_upstream`의 수정 없는 공식 저장소 임시 복제본은 재귀 삭제가 자동 승인 정책에 차단되어 남았다. 둘 다 프로젝트의 유일한 보존 원본은 아니다.

[기계 집계](../../artifacts/validation/2026-11-15/github_preservation_v2/summary.json)에 범위와 해시를 기록했다. 이전 `local_only_manifest` 및 당시 보고서에 적힌 로컬 경로는 정리 전 위치를 나타내며, 파일이 계속 로컬에 있다는 뜻이 아니다. 미선정 시뮬 원시 입력은 의도적으로 제거했으므로 전체 과거 시험의 세부 프레임 재생은 보장하지 않는다. 결과 요약과 실패 판정을 삭제하거나 성공으로 바꾸지는 않았다.

## 정리 중 CAD 보조 자료 복구

`outputs/cad_compare_2026_11_14/`가 전체 `*` 제외 규칙 아래 있어 보조 코드·JSON·그림까지 중간 산출물로 분류해 삭제한 오류가 있었다. 최종 STEP과 공식 PDF는 삭제 전에 원본 해시로 GitHub에 보존되어 있었다. 생성 당시 입력에서 코드8개·README·source.json을 복구했고10파일 모두 삭제 전 바이트 길이와 일치했다. 삭제 전 파일 해시가 없으므로 원바이트 동일성을 독립적으로 확인한 것으로 표시하지 않는다.

검증된11/13·11/14 STEP으로 비교 JSON·그림을 재생성하고 핵심 수치와 화면을 대조했다. [비교 자료](../../outputs/cad_compare_2026_11_14/README.md)와 [복구 검증](../../outputs/cad_compare_2026_11_14/restoration_report.json)을 공개 Git에 추가했으며, 제외 규칙은 STEP 사본·B-rep·mesh·로그 등의 중간물만 대상으로 좁혔다. 위 삭제량은 정리 시점의 총량이며 이후 복구한 약2.7MB를 포함한 순감소량과 구분한다.
