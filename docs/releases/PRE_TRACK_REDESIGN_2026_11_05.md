# 코스 개편 전 개발 스냅샷 — 2026-11-05

사용자가 예고한 큰 코스 단순화 전에, 기존 코스에서 개발한 코드·구성·성공과 실패 기록을 보존한다. **완성품 배포가 아닌 개발 시점 보관본**이다. 새 공식 코스는 아직 공용 실행 월드에 적용하지 않았다.

- 태그: `snapshot-2026.11.05-pre-track-redesign`
- [GitHub 릴리스](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/releases/tag/snapshot-2026.11.05-pre-track-redesign)
- 당시 [현황](../PROJECT_CONTEXT.md)·[계획](../DEVELOPMENT_PLAN.md)·[활동](../activity/2026-11-05.md)은 이 태그에서 읽는다. 이후 main 문서와 구분한다.

## 보존한 구성과 결과

- 공식 원본 **v2026.11.01** ZIP, 기둥 없는 팀 실행 월드와 생성·출처 기록. 방지턱 없음, 2×5 신호와 ID0/45 유지.
- JR60P20·Ø20 배선 예약 외곽, B0183·C1 순차 10 Hz·운동 보정, 모터축 단일 28카운트·8.970588:1·65 mm 바퀴.
- 단독 직선 속도 후보 r15: 한 랩 **31.850초**, 차체 최고 **2.854 m/s(10.27 km/h)**, 10 km/h 이상 **0.663초**, 접촉·노면 이탈 0. [실행·회귀·한계](../../artifacts/validation/2026-11-05/solo_straight_closed_loop_v1/README.md).
- 저속 이동차 추월 r12와 가속 상대 양보·뒤 합류 r14의 대표 주행, 이전 실패와 시험 당시 소스. [추월](../../artifacts/validation/2026-11-05/moving_pass_v1/README.md)·[양보](../../artifacts/validation/2026-11-05/moving_pass_withdrawal_v1/README.md).
- r16 복귀→추종 인계 수정은 적용·재생·회귀·등록까지만 완료했다. **새 폐루프 주행은 미실행**이다. [재개점](../../artifacts/validation/2026-11-05/moving_pass_handoff_v1/README.md).
- 별도 세션의 [직각 코너 시험](../../artifacts/validation/2026-11-05/right_angle_course_v1/README.md)도 보관한다. 정확한 새 코스 치수를 모르는 상태의 가정 코스이며 공용 코드/공식 실행 월드를 바꾸지 않았다. 6회 모두 전체 시험은 실패했고, 격리 후보에서 고립 왼쪽 직각 두 곳만 통과했다. 새 코스 대응 완료가 아니다.

마지막 코너에서 실제 정지 **0.138초**가 남는다. r14는 뒤 합류했지만 기존 최종 간격 평가에 실패했고 전이 정지도 있었다. 기본 OFF 후보를 이 릴리스에서 자동 활성화하지 않는다. 단독 1회·대표 교통을 실차 또는 모든 경기 상황의 검증으로 확대하지 않는다. [코너 분석 가설](../../artifacts/validation/2026-11-05/solo_corner_continuity_v1/README.md)은 미구현·미검증이다.

## 릴리스 첨부물

| 파일 | 내용 |
|---|---|
| GitHub 자동 Source code ZIP/TAR | 태그에 고정된 저장소 코드·문서·Git 추적 증거 전체 |
| `track-v2026.11.01-team-snapshot.zip` | 공식 원본 ZIP, 팀 실행 월드·텍스처/중심선, 트랙 프로필·센서/차체 구성 23파일. 내부 `SHA256.json`으로 검증 가능 |
| `solo-r15-31.850s-10.27kmh.mp4` | 해당 단독 한 랩 영상. 실시간 녹화 속도와 시뮬 시간 기준 측정값을 구별한다 |
| `SHA256SUMS.txt` | 위 트랙 ZIP·영상의 SHA-256 |

[첨부물과 내부 파일 해시](PRE_TRACK_REDESIGN_2026_11_05.assets.json)를 함께 보존한다. 기존 정책대로 원시 센서 bag·상세 수신/판단 로그는 로컬에 남는다. 릴리스는 전체 디스크/WSL 환경 백업이 아니며, 원시 입력이 필요한 과거 재생은 해당 로컬 로그가 추가로 필요하다. 저장소 루트에 잘못 생성된 `0`, `POWERSHELL_LASTEXITCODE=0`, `POWERSHELL_LASTEXITCODE=1` 세 임시 파일은 게시 대상에서 제외하고 로컬에 보존한다.

## 나중에 이 상태를 열기

현재 작업트리를 덮어쓰지 않고 별도 디렉터리에 태그를 연다.

```bash
git fetch origin --tags
git worktree add --detach ../IT-ARENA-pre-track-redesign snapshot-2026.11.05-pre-track-redesign
```

WSL2 Ubuntu 24.04·ROS 2 Jazzy·Gazebo Harmonic 환경에서 해당 디렉터리의 [시작하기](../../README.md#시작하기)와 [GPU 설정](../simulation/GPU_RENDERING.md)을 따른다. 새 worktree는 별도로 빌드하고 그 `install/setup.bash`를 사용한다. 현재 main의 설치 결과를 섞지 않는다.

```bash
# 빌드·환경 연결 후. 새 결과 경로를 사용한다.
python3 scripts/run_current_solo.py outputs/pre_track_baseline
python3 scripts/run_current_solo.py outputs/pre_track_r15 --straight-budget
```

첫 명령은 기본 단독 기준, 두 번째는 r15 직선 속도 후보를 명시적으로 켠다. 과거 실행 경로가 박힌 등록 스크립트는 보관 증거이며 새 위치에서 그대로 실행된다고 보장하지 않는다. 새 코스로 옮길 때 이 태그의 원본과 실패를 덮어쓰지 않는다.
