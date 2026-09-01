# 공식 `v2026.09.02` 팀 실행 월드 전체 코스 사진

촬영일: 2026-09-02
해상도: 2300×1500 RGB PNG
대상: 공식 `v2026.09.02` 형상에 팀 시험용 경사 ArUco 브래킷·임시
신호등·곡선 방지턱·피니시를 적용한 `track:=official`

## 사진

| 파일 | 시점 | SHA-256 |
|---|---|---|
| [`track_top.png`](track_top.png) | 코스 중심 상공 정수직 | `4c135169c11c835e0097ccd376e1199c1210492268d7bd392e92d6c8edc69daf` |
| [`track_south_oblique.png`](track_south_oblique.png) | 남측 상공 사선 | `7a34b278c24d596747366aa4df80eb5fff7d1b75496b1ab541fdc092328fdf13` |
| [`track_west_oblique.png`](track_west_oblique.png) | 서측 상공 사선 | `3356618765271d7918705faa9dbcae3077f84ad0c8706de5c503b3bdf32bdbc8` |
| [`track_east_oblique.png`](track_east_oblique.png) | 동측 상공 사선 | `c3167f119ebfe7fecf5670df45dab2030eb3094f68f180a8b5e148f10cd773b2` |
| [`track_north_oblique.png`](track_north_oblique.png) | 북측 상공 사선 | `786f9f3d97ad0df6d37aed965dfdd131950f6d8491eb2f26e83a11ddd6cd527b` |

다섯 파일을 직접 확인해 코스 외곽과 두 지름길이 프레임 안에 모두 있고,
대규모 노면·잔디 공백이 다시 나타나지 않았음을 확인했다. 정수직 사진은
전체 형상 비교용이고, 네 사선 사진은 벽·노면 높이와 출발 구역·임시 시설의
상대 배치를 확인하는 용도다. 작은 ArUco 판의 차량 시점 검출 성능은 이
전체 조감 사진이 아니라 별도 RGB 검사 결과로 판단한다.

## 재현과 범위

[`scripts/capture_official_track_views.py`](../../../../scripts/capture_official_track_views.py)가
독립된 headless Gazebo 서버에 고정 외부 카메라를 순서대로 만들고 각 두 번째
프레임을 저장한 뒤 카메라와 서버를 종료한다. 실행 결과와 입력 해시는
[`capture_report.json`](capture_report.json), 프로세스 출력은
[`capture.log`](capture.log)에 있다. 다섯 장 생성, 비단색 검사, 종료 코드 0과
정상 종료를 통과했다.

이 사진은 팀 파생 실행 월드의 형상 확인 자료다. 공식 ZIP을 수정한 것이 아니며,
실물 시공·마찰·센서 성능이나 본선·지름길 주행 안전을 검증하지 않는다.
