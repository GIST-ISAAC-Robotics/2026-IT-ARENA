# 공식 트랙 `v2026.09.01` 재감사

확인일: 2026-09-01

이 문서는 [주최 측 릴리스 `v2026.09.01`](https://github.com/MOSW626/istech-it-arena/releases/tag/v2026.09.01)을 새 현재 기준으로 검증한 기록이다. `v2026.08.31`과 이슈 #7~#11의 원래 증거는 [이전 공식 원본 감사](OFFICIAL_SOURCE_AUDIT.md)에 보존한다.

## 출처와 보존

| 항목 | 값 |
|---|---|
| 태그 커밋 | [`921f3f9a044f1a38ff849cb8e19d00182dd5533b`](https://github.com/MOSW626/istech-it-arena/tree/921f3f9a044f1a38ff849cb8e19d00182dd5533b) |
| 릴리스 자산 | `it_arena_track_v2026.09.01.zip`, 822,391 bytes |
| SHA-256 | `f40aca619a6207f48f33741a56716edc65e948a674c7937f70b44476205b894c` |
| 로컬 보존 | [`assets/track/official/v2026.09.01/`](../../assets/track/official/v2026.09.01/SOURCE.md) |
| 파일 수 | 파일 24개. 실행에 불필요한 Python 3.14 `__pycache__` 1개 포함 |

릴리스 자산의 GitHub `digest`, 다운로드 파일 해시와 로컬 보존본이 일치한다. 기존 `v2026.08.31`은 수정하거나 교체하지 않았다.

## 이슈 #7~#11 재확인

2026-09-01 재조회 시 이슈 #7~#11은 모두 `CLOSED`이며, 각 이슈에는 새 릴리스 반영 설명이 있다.

| 이슈 | 새 원본에서 확인한 상태 | 판정 |
|---|---|---|
| [#7 로드·물리·마커](https://github.com/MOSW626/istech-it-arena/issues/7) | 불완전한 재질 `script` 제거, 상대 경로·diffuse 수정, 링크 20개. SDF·첫 갱신·2,000회·정면 네 ID 검출 통과 | 해결 확인 |
| [#8 마커 벽 부착·ID 30](https://github.com/MOSW626/istech-it-arena/issues/8) | 네 마커 pose가 벽면으로 이동. 본선/지름길의 20×15 cm 정적 경로 표본 간섭 0개 | 보고한 정적 간섭 해결 확인 |
| [#9 그리드 엇갈림](https://github.com/MOSW626/istech-it-arena/issues/9) | `longitudinal_stagger_m=0.2`, 실제 좌우 슬롯 차이와 일치 | 해결 확인 |
| [#10 그리드 매몰](https://github.com/MOSW626/istech-it-arena/issues/10) | 표시 상단 0.0042 m > 노면 상단 0.003 m, 표시 collision 없음 | 해결 확인 |
| [#11 출발/결승선](https://github.com/MOSW626/istech-it-arena/issues/11) | 설계·scene의 기준이 `s=0`, 신호등·ID 0과 일치 | 해결 확인 |

구버전과 새 버전의 세계 좌표 기준 visual/collision 정규화 비교에서 변경된 형상은 ArUco 8개와 그리드 visual 6개뿐이었다. 도로·벽·잔디·접촉 형상의 예상 밖 변경은 찾지 못했다. 공식 최소 곡률 반경 약 0.299 m는 릴리스 설명에서도 의도한 예외로 유지됐다.

## 현재 팀 실행본

- 입력: [`config/tracks/official_v2026.09.01.yaml`](../../config/tracks/official_v2026.09.01.yaml)
- 생성: [`scripts/build_official_track.py`](../../scripts/build_official_track.py)
- 결과: `src/arena_gazebo/worlds/it_arena_official/`
- 결정: [ADR 0011](../decisions/0011-official-v2026-09-01-track.md)

새 릴리스에서 마커 재질과 pose가 고쳐졌으므로 공식 PNG/PBR 판을 그대로 보존한다. 이전 실행본의 벽 스냅·ID 30 추가 이동·비텍스처 코드 셀 대체는 적용하지 않는다. 방지턱의 코사인 단면·색띠, 낮은 신호등과 데모 순서, 그리드 U자/번호, 피니시 체크무늬만 아직 임시다.

공식 벽 collision의 `mu=mu2=0.8`만 유지한다. 도로·잔디·방지턱은 계수가 여전히 미지정이므로 엔진 기본값을 실물 마찰로 표현하지 않는다.

## 새로 발견한 관찰

1. **차량 접근각의 ArUco 검출 구간**: 정면 진단 카메라에서는 네 ID가 모두 잡히지만, 848×480·수평 69.4°의 차량 전방 카메라를 본선 중심선에 놓으면 벽면 마커가 비스듬하거나 곡선 뒤로 사라져 검출 구간이 짧다. 기본 12개 마커 조건 중 6개, 촘촘한 37개 조건 중 9개만 해당 ID를 검출했다. 저속 한 바퀴에서는 네 ID를 모두 검출했으므로 파일 파손이 아니라 고속·가림에 대한 설계 위험으로 분류한다.
2. **생성기 OpenCV 버전**: ROS Jazzy 기본 OpenCV 4.6에는 `generateImageMarker`가 없어 생성기가 그대로 실행되지 않았다. 공식 README는 별도 venv의 최신 패키지를 요구하므로 현재는 환경 호환 메모이며 upstream 결함으로 단정하지 않는다.
3. **ZIP의 `__pycache__`**: 실행과 무관한 `.pyc` 한 개가 포함됐다. 무결성·실행에는 영향이 없어 이슈 후보로 올리지 않는다.
4. **장시간 검사 종료**: 마커 시인성 장시간 실행 한 번은 ToF bridge `-11`, 다른 한 번은 Gazebo 종료 지연이 있었다. 같은 최종 월드의 독립 스모크·한 바퀴·원본 2,000회는 정상 종료했으므로 트랙 릴리스 결함으로 귀속하지 않는다.

자세한 수치와 실패 사진은 [검증 근거](../../artifacts/validation/2026-09-01/official_update/README.md)에 있다. ArUco 접근 시인성은 [미승인 이슈 초안](OFFICIAL_UPSTREAM_ISSUE_DRAFTS.md)에만 준비하며, 사용자의 별도 승인 전에는 게시하지 않는다.

## 검증 경계

현재 결과는 한 대의 차량이 본선을 저속으로 주행한 시뮬레이션이다. 지름길의 연속 진입·합류, 여러 차량의 가림, 실제 인쇄·조명·노출·모션 블러, 20 km/h에서의 마커 검출과 실물 마찰은 확인하지 않았다. 담당 동아리의 신호등·방지턱 파일과 후속 릴리스가 공개되면 이 버전과 새 버전을 별도 보존해 다시 비교한다.
