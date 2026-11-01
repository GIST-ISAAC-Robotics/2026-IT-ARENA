# 공식 11/1 회의·배포본 원문과 정적 비교

확인 시각: **2026-11-02 01:09 KST**. 팀 실행 기준은 `d8f47117efc053ce6c02a3dab354a9d2075b55e6`이며 이번 작업은 자료·계획 갱신이다. **실행 코드·설정·월드는 변경하지 않았고 Gazebo/Jetson도 실행하지 않았다.**

- 공식 main: `17178293029d446fa603f3229556dd865bbddd38`
- 공식 릴리스: `v2026.11.01`, 태그 커밋 `2e3e65b1e6202b31d111c3fe50aa43c6634a5f6a`
- [취득 원문·크기·SHA-256](source_manifest.json), [정적 모델 대조 결과](model_audit.json)
- [문서 링크·원본 해시·보관본 검사](documentation_checks.json)
- [규정·근사·적용 계획의 해석](../../../../docs/reports/OFFICIAL_UPDATE_2026_11_02.md)

## 파일

- `upstream/`: 고정 main의 MANUAL·회의록·track 문서/모델·세이브포인트 그림·과거 방지턱 안내, 조회 시점의 이슈 #2/#3/#6/#16과 댓글·commit/release/compare API 응답. 원문은 줄바꿈까지 보존한다. 내부 상대 링크는 공식 저장소 기준이다.
- [보존 ZIP](../../../../assets/track/official/v2026.11.01/it_arena_track_v2026.11.01.zip): 790,959 bytes, SHA-256 `ef50e0f125eb4b8cba3cfa736188a24038b7a8c99a4de6b6a782fac5dd2e5cf5`.
- `fetch_sources.py`: 이 스냅샷 취득에 사용한 코드. 처음 조회할 때 main/latest를 고정하고 같은 경로의 다른 바이트를 덮어쓰지 않는다. 이후 최신 자료 취득은 새 날짜 폴더에서 수행한다. 이 파일을 재실행해 기존 스냅샷을 갱신하지 않는다.
- `audit_models.py`: 보존한 v2026.10.24/v2026.11.01 ZIP, main SDF/scene와 11/1 팀 신호 fixture를 정적으로 대조한다.

## 재현

저장소 루트에서 Python 표준 라이브러리만으로 비교할 수 있다.

```powershell
python artifacts/validation/2026-11-02/official_update/audit_models.py
```

각 ZIP의 파일 바이트를 비교하고 SDF 네 링크(`ground_plane_link`, `track_surface`, `grass`, `walls`)의 존재와 정규화된 XML 구조를 대조했다. 네 링크는 collision·재질까지 동일하다. 23개 파일 중 15개 동일·8개 변경, 방지턱 제거, 0/45·신호·그리드 변경을 확인했다. main과 릴리스 SDF/scene도 동일하다. 렌더링/검출/주행 결과는 아니다.

## 수집·검사 중 수정한 점

- GitHub CLI의 PNG raw 응답에서 `transform: short source buffer`가 발생해 이미지 파일을 저장하지 못했다. GitHub contents API의 base64를 디코딩하는 경로로 바꾸어 원문 그림을 취득하고 manifest에 해시를 기록했다.
- 최초 정적 비교 코드가 `road_`/`grass_`/`wall_` 접두사를 찾았으나 실제 링크명과 달라 대상이 0개였다. 이를 성공 근거로 쓰지 않고 실제 네 링크의 존재를 assert한 뒤 다시 대조했다. 최종 JSON에는 비교 대상 이름이 명시되어 있다.

이 오류들은 수집/검사 코드의 오류이며 공식 트랙 주행 실패로 보고하지 않는다. 이전 공식 원본과 팀 주행 실패 자료는 바꾸지 않았다.
