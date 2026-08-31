# 공식 트랙 업스트림 이슈 검토 인덱스

**명시적 승인 전 등록 금지 / 현재 GitHub에 미등록 상태입니다.**

검토 기준: 2026-08-31, `MOSW626/istech-it-arena`의 커밋 `d61c5db9252cedfbc163cd044a47671df91e1660` / 릴리스 `v2026.08.31`.

릴리스 ZIP SHA-256: `897183D2E2541458D190A0A1E3F76BFF754C67FAB894F2302C3110830A86149B`.

## 1. 승인 후 바로 사용할 제출 묶음

일곱 이슈의 제목·label·본문 경로·출처 커밋·승인 상태는 [manifest.json](issues/official-v2026.08.31/manifest.json)에 정리했습니다. 각 본문 파일은 바깥쪽 코드 블록이나 작업 메모가 없는 **순수 Markdown**입니다. 내용 전체를 복사해 GitHub 본문에 붙여 넣을 수 있습니다. 제목은 아래 표 또는 manifest의 `title`을 사용합니다.

| 번호 | 제목 (`title`) | 순수 본문 | 기본 labels | 승인 상태 |
|---|---|---|---|---|
| 1 | `[문제] 코스 ArUco의 벽 이격 및 ID 30의 지름길 2 충돌` | [본문 1](issues/official-v2026.08.31/01-aruco-wall-placement.md) | `bug` | `unapproved` |
| 2 | `[문제] 과속방지턱 SDF의 진행 길이와 횡단 폭 축 반전` | [본문 2](issues/official-v2026.08.31/02-bump-axis.md) | `bug` | `unapproved` |
| 3 | `[문제] ArUco script name 누락에 따른 SDF 로드 실패 및 texture 경로 오류` | [본문 3](issues/official-v2026.08.31/03-aruco-sdf-material.md) | `bug` | `unapproved` |
| 4 | `[문제] 시뮬레이션 신호등의 3색 동시 emissive 및 화면 상태 미연동` | [본문 4](issues/official-v2026.08.31/04-traffic-light-emission.md) | `bug` | `unapproved` |
| 5 | `[문제] 설계 start_line과 신호등·ID 0·출력 도면의 기준 s 불일치` | [본문 5](issues/official-v2026.08.31/05-start-finish-origin.md) | `bug` | `unapproved` |
| 6 | `[문제] starting_grid의 실제 stagger 0.20 m와 메타데이터 0.30 m 불일치` | [본문 6](issues/official-v2026.08.31/06-grid-stagger.md) | `bug` | `unapproved` |
| 7 | `[문제] 출발 그리드 visual이 노면 아래에 묻혀 보이지 않음` | [본문 7](issues/official-v2026.08.31/07-grid-paint-height.md) | `bug` | `unapproved` |

manifest의 `approval_status`는 전체·개별 항목 모두 `unapproved`, `submission_status`는 `not_submitted`, `issue_url`은 `null`입니다. 문서를 완성하거나 팀 저장소에 공개한 것만으로 업스트림 등록 승인이 생기지는 않습니다. 이 작업에서는 이슈 생성 API·CLI와 브라우저 Submit을 사용하지 않았습니다.

## 2. 업스트림 양식·사용 안내 준수

다음 파일을 직접 확인했습니다.

- [`.github/ISSUE_TEMPLATE/bug.md`](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/.github/ISSUE_TEMPLATE/bug.md): 이름 “🐛 파일/트랙 문제”, 제목 접두사 `[문제]`, label `bug`.
- [공식 README의 이슈 사용법](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/README.md#이슈issues-사용법): **이슈 하나에 주제 하나**, 파일/트랙 문제에는 스크린샷 첨부.
- [`docs/github-guide.md`](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/docs/github-guide.md): 알맞은 템플릿 선택, 스크린샷 첨부, 기존 open/closed 이슈 중복 검색.

각 본문은 공식 템플릿의 **어떤 파일에서 → 무엇이 이상한가요 → 환경** 필드 순서를 유지합니다. 이후 재현 방법·기대동작·실제결과·영향·근거자료·수정제안·완료 체크리스트를 확장했습니다. 공식 양식이 없다고 가정해 일반 양식으로 대체하지 않았습니다.

2026-08-31에 GitHub REST API를 **GET으로만 조회**하여 `bug`가 실제 label 목록에 있음을 확인했습니다. 전체 label은 `accessibility`, `bug`, `documentation`, `duplicate`, `enhancement`, `good first issue`, `help wanted`, `invalid`, `proposal`, `question`, `wontfix`, `진행관리`의 12개입니다. 기존 추가 후보인 `track`, `simulation`, `geometry`, `facility`, `gazebo`, `metadata`, `generator`, `rendering`은 실제로 존재하지 않아 manifest의 `suggested_extra_labels_not_available`에 참고 정보로만 보존했습니다. 제출용 `labels`는 `bug` 하나이며, 존재하지 않는 label을 만들거나 임의로 추가하지 않습니다.

### 기존 이슈 중복 확인

같은 날 `GET /repos/MOSW626/istech-it-arena/issues?state=all&per_page=100`을 페이지 끝까지 조회하고, PR을 제외한 이슈 **6개 전부의 제목·본문**을 읽었습니다. open 6개, closed 0개였으며 이번 일곱 파일 문제의 직접 중복은 발견하지 못했습니다. 댓글은 검사 범위가 아니며, 등록 직전에는 새 이슈나 후속 수정 여부를 다시 확인해야 합니다.

| 기존 이슈 | 이번 초안과의 관계 |
|---|---|
| [#1 차량 식별 마커 모양 공유](https://github.com/MOSW626/istech-it-arena/issues/1) | 차량 후방 식별 마커 제작 진행이며, 초안 1의 코스 벽 마커 배치와는 별개입니다. |
| [#2 과속방지턱 3D 프린팅 파일 공유](https://github.com/MOSW626/istech-it-arena/issues/2) | 명목 진행 길이 0.05 m와 실물 자료 공유를 관리합니다. 초안 2는 기존 명목값을 SDF에 내보내는 축 반전만 다룹니다. |
| [#3 출발 신호등 제작](https://github.com/MOSW626/istech-it-arena/issues/3) | 실물 제작·후속 점등 시퀀스 공개를 관리합니다. 초안 4는 현재 시뮬레이션 화면의 동시 발광과 상태 미연동을 다룹니다. |
| [#4 학교별 물품 주문 엑셀 제출](https://github.com/MOSW626/istech-it-arena/issues/4) | 이번 파일 오류들과 직접 중복되지 않습니다. |
| [#5 traffic_light.py 랜덤 홀드 추가 방법](https://github.com/MOSW626/istech-it-arena/issues/5) | 초안 4와 관련 있지만 랜덤 hold 자체는 이미 별도 논의 중입니다. 중복 제안은 제거하고 기존 이슈로 연결했습니다. |
| [#6 세부 규칙 회의 안건 수합](https://github.com/MOSW626/istech-it-arena/issues/6) | 규칙 미정 안건을 관리하며 이번 기하·재질·메타데이터 오류는 본문에 없었습니다. |

검증 환경은 **Windows 호스트의 WSL Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.11.0**입니다. `gz sim --versions`에서도 `8.11.0`을 확인했습니다. SDF 로드 실패는 이 환경에서 원본 파일을 `gz sdf -k`로 검사한 결과이며, JSON/CSV 수치 검사는 별도의 정적 분석입니다.

## 3. 어떤 항목끼리 묶을 것인가

| 묶음 | 권고 | 이유 |
|---|---|---|
| 1: ArUco 벽 이격 + ID 30 간섭 | 한 건 유지 | 같은 마커 pose 생성 함수에서 발생하는 배치 문제와 그 통과 경로 영향입니다. |
| 3: ArUco material + texture 경로 | 한 건 유지 | 동일한 마커 재질 내보내기 경로의 로드·렌더링 오류입니다. 1번의 물리 배치와는 별도입니다. |
| 2: 방지턱 축 / 4: 신호등 상태 | 각각 독립 등록, 서로 교차 참조 | 둘 다 시설 placeholder이지만 담당·재현·수정 지점이 다릅니다. |
| 5: start line / 6: stagger | **별도 등록** | 공통 JSON 소비자에 영향을 주더라도 출발선 의미와 슬롯 간격 적용값은 서로 다른 주제입니다. |
| 7: 그리드 z | 독립 등록 | 실제 슬롯 위치나 stagger가 아니라 표시 형상의 높이 문제입니다. |

승인 후 우선순위는 **3번(월드 로드 차단) → 1번(지름길 시설 충돌) → 5·6·7번(출력 일관성·표시)**입니다. 2·4번은 “현재 배포된 시뮬레이션 placeholder의 재현 오류”라고 명시해 별도 등록하고, MeKENic·Pinocchio의 후속 자료가 오면 대조합니다. 이슈 개수를 줄이려고 일곱 주제를 한 보고서형 이슈로 합치지 않습니다.

## 4. 확정 파일 오류와 placeholder의 경계

| 항목 | 현재 파일에서 확정한 사실 | 아직 확정하지 않는 부분 |
|---|---|---|
| ArUco pose | 벽 이격 및 지름길 2의 10표본 충돌 | 임의의 새 ID·사전·코스 규칙을 제안하지 않음 |
| 방지턱 | 로컬 X/Y에 진행 길이·횡단 폭이 반대로 전달됨 | 실물 최종 단면·재질·제작 공차는 MeKENic 후속 자료 대상 |
| ArUco SDF | 필수 script name 누락 4개·texture 경로 불일치. 호환성 변환 화면에서도 ID 30 판이 검게 보임 | 검은 판의 모든 렌더링 원인을 한 원인으로 단정하지 않음 |
| 신호등 | 세 색 emissive 동시 활성, UDP→Gazebo 화면 미연동 | Pinocchio 실물 색 순서·형상·높이·대기 시간은 미공개 |
| start line·stagger | 설계/출력 s와 실제 적용값/메타데이터가 다름 | 어느 start line이 운영상 의도인지 임의 결정하지 않음 |
| 그리드 z | 그리드 상단 0.0017 m < 노면 상단 0.0030 m | 새 슬롯 크기·번호 체계를 공식 규칙으로 추가하지 않음 |

이번 이슈 묶음은 트랙의 네 코스 마커에 대한 것입니다. 별도로 확정된 **차량 후방 5×5 cm 식별 마커 부착 의무**를 미정이라고 바꾸거나, 후방 마커 상세까지 확정됐다고 확대하지 않습니다.

## 5. 증거 생성 상태

아래 여섯 PNG가 실제 파일로 생성된 것을 확인했습니다. 본문에는 `https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/<filename>` 형태의 **완전한 이미지 URL**을 사용했습니다. 로컬 상대경로를 업스트림 이슈에 복사해 깨지는 방식이 아닙니다.

| 파일 | 로컬 상태 | 증거 종류 |
|---|---|---|
| [upstream_full_overview.png](../../artifacts/screenshots/2026-08-31/official_update/upstream_full_overview.png) | 생성 확인 | 공식 배포 좌표의 실제 실행 전체 화면 |
| [upstream_marker_id30.png](../../artifacts/screenshots/2026-08-31/official_update/upstream_marker_id30.png) | 생성 확인 | ID 30·지름길 2 실제 근접 화면 |
| [upstream_id30_collision_plan.png](../../artifacts/screenshots/2026-08-31/official_update/upstream_id30_collision_plan.png) | 생성 확인 | 릴리스 CSV 141표본·20×15 cm 외형·10표본 간섭 계산 도식 |
| [upstream_bump_axis.png](../../artifacts/screenshots/2026-08-31/official_update/upstream_bump_axis.png) | 생성 확인 | 원본 방지턱 실제 실행 화면 |
| [upstream_bump_axis_plan.png](../../artifacts/screenshots/2026-08-31/official_update/upstream_bump_axis_plan.png) | 생성 확인 | SDF 로컬 X/Y와 명목 진행/횡단 치수 비교 도식 |
| [upstream_signal_all_lit.png](../../artifacts/screenshots/2026-08-31/official_update/upstream_signal_all_lit.png) | 생성 확인 | 원본 3색 동시 발광 화면 |

원본 SDF는 엄격 검증에 실패하므로 실행 캡처에는 로드용 호환성 변환이 적용됐습니다. **좌표·치수는 변경하지 않았으며**, 이 화면을 원본 SDF 자체의 파서 통과 증거로 사용하지 않습니다. 원본 로드 실패는 실제 `gz sdf -k`의 Error Code 8 네 회와 Error Code 9로 확인했습니다. 스크린샷과 계산 도식은 본문에서도 각각 구별했습니다.

두 정량 도식의 재현 코드는 [scripts/illustrate_official_issues.py](../../scripts/illustrate_official_issues.py)에 보존했습니다. 이 스크립트는 보존된 공식 ZIP의 `scene.json`, `branch_1.csv`, `world.sdf`를 읽어 ID 30 간섭 도식과 방지턱 축 비교 도식을 생성합니다. 실제 Gazebo 화면 캡처를 생성하는 스크립트는 아닙니다.

전체 화면은 배치 맥락을 보여 줄 뿐입니다. start line·stagger·그리드 z의 정량 근거는 공식 JSON/SDF와 본문에 제시한 읽기 전용 재현 절차입니다. 화면에서 1.3 mm나 10 cm 차이를 측정했다고 주장하지 않습니다.

## 6. 등록 전 마지막 확인

- [ ] 마스터의 명시적 이슈 등록 승인 확보
- [x] 2026-08-31 읽기 전용 조회로 기존 open 6개·closed 0개의 제목·본문 확인: 직접 중복 미발견
- [ ] 등록 직전 open/closed 이슈와 후속 댓글·수정 사항을 다시 확인
- [ ] 최신 배포에서도 재현되는지 확인하고 해당 버전 명시
- [x] 2026-08-31 읽기 전용 조회로 `bug` 실제 존재 및 추가 후보 8개 미존재 확인
- [ ] 팀 저장소 `main`에서 일곱 본문·감사 문서·여섯 이미지의 공개 링크 열림 확인
- [x] 여섯 증거 이미지의 화면·도식에 계정·토큰·무관한 개인정보가 없는지 최종 확인
- [ ] 승인받은 항목만 각각 등록하고 실제 URL을 기록
- [ ] 등록 후에만 manifest의 승인/등록 상태와 `issue_url` 갱신

현재 체크리스트는 등록 절차의 경계입니다. 준비 문서 완성은 이슈 등록 완료를 뜻하지 않습니다.
