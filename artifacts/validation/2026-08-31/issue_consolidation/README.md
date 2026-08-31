# 단일 월드 로드 이슈의 검증 근거

확인일: 2026-08-31. 제출할 글은 [이슈 초안 한 건](../../../../docs/track/OFFICIAL_UPSTREAM_ISSUE_DRAFTS.md)입니다. 이 폴더는 재현 자료와 정리 전 기록의 보관소이며 별도 이슈 목록이 아닙니다.

## 이번에 확인한 순서

공식 `v2026.08.31` ZIP을 새 폴더에 압축 해제한 뒤 아래 변경을 누적했습니다. [실행 요약](retest/summary.json)은 로드/SDF 검사 11회, [사본 비교](retest/fixtures.json)는 원본 23개 파일과 각 단계의 해시를 담습니다.

| 단계 | 변경·관측 |
|---|---|
| `raw` | 무수정 원본. `script/name` 누락으로 Code 8 네 회·Code 9 한 회. 서버 종료 코드 0이어도 월드 초기화 실패 |
| `remove_script` | 불완전한 script 네 개만 제거. SDF 검사 통과, 첫 물리 갱신은 180초 내 미완료. 강제 종료 포함 192.2548초 |
| `merge_only` | 경로는 그대로 두고 정적 링크 1,113→10개 병합. 첫 물리 갱신과 정상 종료 2.8388초. RGB 렌더러는 PNG 네 개를 찾지 못함 |
| `merge_static` | 네 PBR 경로를 `aruco/…`로 수정. 파일 오류는 사라졌으나 마커 4개가 모두 검게 표시됨 |
| `ready` | 네 재질에 흰색 diffuse만 추가. 첫 갱신 2.8334초, 별도 2,000회 물리 계산 5.9453초 후 정상 종료. 물리+RGB 실행에서 네 ID 검출 |

- 다섯 단계 모두 충돌체 1,107개·시각 형상 1,113개 및 위치·회전·치수·접촉 설정의 정규화 해시가 같습니다. 마커 PNG·UV·위치와 조명은 변경하지 않았습니다. 우리 운영 월드·차량도 변경하지 않았습니다.
- 이번 [RGB 세 조건](retest/render/)은 **모두 Physics와 Sensors를 함께 활성화**했습니다. 정면 35 cm·높이 10 cm 고정 카메라 4개, 640×480·2 Hz·수평 화각 0.65 rad, Ogre2와 OpenCV 4.6.0으로 확인했습니다. [최종 결과](retest/render/04_ready/result.json)는 각 10프레임, 시뮬레이션 시각 0.002→4.5초, ID 0/20/30/45, bridge/서버 종료 코드 0/0입니다.
- **중간 조건의 종료 실패도 보존했습니다.** `03_black_material`은 프레임 수집 후 서버가 SIGINT·SIGTERM에 종료되지 않아 SIGKILL로 정리됐습니다(bridge 0, 서버 -9, 잔류 PID 없음). 원본 재질과의 인과관계는 확인하지 않았으므로 별도 upstream 결함으로 주장하지 않습니다. 최종 `04_ready`의 정상 종료와 구분합니다.
- 앞선 [원인 분리 감사](../official_raw_load/README.md)는 Physics를 제외한 RGB 대조였습니다. 이번 결합 실행으로 그 한계를 보강했으며 과거 결과를 덮어쓰지 않았습니다. 어느 시험도 차량 주행·실물 가시성·다른 OS/물리 엔진을 검증한 것은 아닙니다.

## 다시 실행하는 방법

WSL Ubuntu 24.04.4 / ROS Jazzy / Gazebo 8.11.0 / sdformat 14.9.0, 실제 물리 엔진 DART에서 실행했습니다. 저장소 루트에서 아래 명령을 사용합니다. `build/official_load_recheck`는 아직 없는 폴더여야 하며, 재실행할 때 기존 기록을 지우지 말고 새 이름을 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/audit_official_world_load.py --workdir build/official_load_recheck --prepare --sequential --cases raw --timeout 30 --line-buffered
python3 scripts/audit_official_world_load.py --workdir build/official_load_recheck --cases remove_script --timeout 180 --line-buffered
python3 scripts/audit_official_world_load.py --workdir build/official_load_recheck --cases merge_only merge_static ready --timeout 30 --line-buffered
python3 scripts/audit_official_world_load.py --workdir build/official_load_recheck --cases ready --modes server --iterations 2000 --run-id sustained --timeout 45 --line-buffered
python3 scripts/audit_official_marker_render.py as_is --workdir build/official_load_recheck --fixture merge_only --output-name 02_bad_path --physics
python3 scripts/audit_official_marker_render.py as_is --workdir build/official_load_recheck --fixture merge_static --output-name 03_black_material --physics
python3 scripts/audit_official_marker_render.py as_is --workdir build/official_load_recheck --fixture ready --output-name 04_ready --physics --min-frames 10
```

이 도구들은 실패 조건도 수집합니다. 도구 자체의 종료 코드 0을 모든 조건의 성공으로 해석하지 말고, 각 `result.json`의 오류·관측 상한·자식 종료 코드·정리 결과를 확인합니다. 원시 실행 자료는 `build/official_single_issue_20260831/`에 별도 보존했고, 공개 로그에서만 사용자 경로와 제어 문자를 정리했습니다. `retest/summary.json`의 게시 상태는 **검증 자료 생성 시점**의 기록입니다.

전체 Python/ROS 회귀 검사 **114개**와 공식/실험 트랙 생성기의 `--check`를 통과했습니다. 이는 기존 코드·생성 결과와 재현 도구의 검사이며 배포 원본의 오류가 사라졌다는 뜻은 아닙니다.

## 원본과 정리 전 기록 보존

- 공식 ZIP SHA-256: `897183d2e2541458d190a0a1e3f76bff754c67fab894f2302c3110830a86149b`
- 원본 `world.sdf` SHA-256: `62d7ae4abf2dbcc0ce850065de5ade28703a67649ab6a8c62311c432cc2200de`
- [정리 전 압축본](before_consolidation.zip): 13개 파일, 65,934 bytes, SHA-256 `422a03bda5c30d5b66ab0d95346965e19c79005b715ce86d7b0debd2f166c88c`. 개별 초안 9개·manifest·이전 색인·원본 감사 문서·정리 직전 활동 기록을 원래 상대경로로 보존했습니다. 생성 당시 각 파일의 바이트 일치를 검사했습니다.
- 중복 개별 초안·manifest·감사 설명문은 현재 문서 구조에서 제거했습니다. 더 이상 쓰지 않는 `scripts/illustrate_official_issues.py`는 [정리 전 Git 이력](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/blob/71d04ee562abac11b9a4ac73bcc3612cfcfc451d/scripts/illustrate_official_issues.py)에서 복구할 수 있습니다. 기존 사진·도식·실행 로그·활동 기록은 삭제하지 않았습니다.
- 나머지 시설·기하 관찰은 [공식 자료 감사 §5](../../../../docs/track/OFFICIAL_SOURCE_AUDIT.md#5-공식-출력물-내부-불일치와-재현한-한계)에만 보류 상태로 관리합니다. 운영진 업데이트 후 재확인하며, 현재 주최 측에 게시한 이슈·댓글은 없습니다.
