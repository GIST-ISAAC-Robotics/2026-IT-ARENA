# 공식 v2026.11.06(610 mm 직각 코스) 원본 감사 산출물

확인: **2026-11-06 KST**. 판단과 우선순위는 [감사 보고서](../../../../docs/track/OFFICIAL_V2026_11_06_AUDIT.md) 한 곳에 정리했다. 이 폴더는 원문·추출본·도구·기계 결과·사진만 보존한다. **공용 `src/`·`config/`·`scripts/`와 실행 월드, 이전 공식 원본은 바꾸지 않았다.** 차량 주행·제어기 완주·실물 측정은 하지 않았다.

## 입력 고정

- 공식 main·태그 `v2026.11.06` = `45691aec825ebfe4ff3edd9f221118e0c5026d1a`, 릴리스 게시 2026-11-05T19:41:34Z
- [보존 ZIP](../../../../assets/track/official/v2026.11.06/README.md): 691,524 bytes, SHA-256 `d1f3fbc1c16c6901130f260f5baa80ebf00040bcbaf3ccdb175e63738ed325fa`, GitHub digest 일치
- `upstream/source_manifest.json`: 취득 시각·main/태그·자산 해시·이전 감사 기준(`17178293`) 이후 변경 파일 목록
- `upstream/api/`: commit·release·tag·compare(패치 본문 제외)·전체 트리 API 응답
- `upstream/main/`: README·MANUAL·track/README·`design_final.json`·`make_design_610.py`·`track_gen.py`·`docs/venue/README.md`·세이브포인트 그림과 생성 코드. Git blob SHA-1로 트리와 대조한 뒤 저장했다.
- `extracted/`: ZIP을 빈 새 경로에 추출한 사본(25파일). 경로 이탈·절대경로·중복·심볼릭 링크·CRC 오류 0건. 이 사본은 수정하지 않는다.

## 도구와 실행

배포 파일은 데이터로만 읽었다. Python은 `-I`로 실행했고 배포 `track_gen.py`(생성기)는 실행하지 않았다.

| 도구 | 실행 | 결과 |
|---|---|---|
| `tools/fetch_sources.py` | Windows `python -I`, exit 0 | 원문·ZIP 취득, digest·크기 일치 |
| `tools/zip_audit.py` | Windows `python -I`, exit 0 | `checks/zip_audit.json`. ZIP 25파일 전부 main `track/`과 같은 blob, 필수 항목 누락 0, 이전 ZIP 대비 동일 6·변경 17·추가 2·삭제 0 |
| `tools/static_audit.py` | WSL `python3 -I`, exit 0 | `checks/static_audit.json`, `checks/plots/*.png`. SDF 구조·재질·물성·URI, 도로/잔디/벽 기하, PDF 표(수기 전사 `tools/pdf_tables_transcribed.json`)·scene·CSV·map 대조, 시설 위치, 차량 정적 기하 경로 |
| `tools/zoom_detail.py` | WSL, exit 0 | P21 사선·접속부 고배율 상자 그림 |
| `tools/p22_entry_feasible.py` | WSL `python3 -I`, exit 0 | `checks/p22_entry_feasible.json`. P22 진입 정적 경로의 가능한 (횡 위치, 반경) 조합. 반경 격자는 0.2795 m(0.45 rad를 자전거 등가각으로 본 값)를 포함한다. 현행 운용 상한(등가 0.3711 rad, 후축 반경 약 0.347 m)과의 구분은 보고서 7절에 있다 |
| `tools/dxf_check.py` | Windows `python -I`, exit 0 | `checks/dxf_check.json`. DXF 레이어·문자·벽선(PDF 끝점 최대 0.5 mm) |
| `tools/team_pipeline_static_check.py` | WSL `python3 -I`, exit 0 | `checks/team_pipeline_static_check.json`. 팀 빌드·평가 함수 읽기 전용 연결 감사(기대 실패 포함) |
| `tools/gz_audit.py load --run-id 01` | WSL, Haiku 실행 보조, exit 0 | `checks/gazebo/load_01/`. `gz sdf -k` "Valid.". 기본 서버로 월드를 1회 로드한 뒤 `--iterations 2000`으로 2,000 physics steps(기본 1 ms, 약 2초 시뮬레이션 시간)를 진행하고 정상 종료, Err/Wrn 0 |
| `tools/gz_audit.py render --run-id 01` | 같은 방식, **exit 1** | 도구 오류 보존: 카메라 이름의 `+`가 ROS 토픽 규칙 위반(`InvalidTopicNameException`). 서버 준비까지 정상, 사진 0장, 잔존 프로세스 0 |
| `tools/gz_audit.py render --run-id 02` | 같은 방식, exit 0 | `checks/gazebo/render_02/`. 원본 world.sdf 그대로 + Sensors(ogre2) 서버 설정 + 실행 중 생성한 정지 카메라 28대. 프레임 전수 수신, 잔존 프로세스 0, 사본 해시 불변 |
| `tools/annotate_overview.py` | WSL, exit 0 | `annotated/overview_top_labeled.png`(표기만 추가), `checks/overview_numbers.json` |

Gazebo는 `~/arena_scratch/official_v20261106_audit_v1/<run>/output_final`에 해시 검증한 복사본에서 고유 `GZ_PARTITION`/`ROS_DOMAIN_ID`로 실행했다. 환경: Ubuntu 24.04.4, ROS 2 Jazzy, Gazebo Sim 8.11.0, SDFormat 14.9.0, dartsim, ogre2, RTX 5070 Laptop.

재현(저장소 루트 기준, 결과 폴더가 있으면 도구가 덮어쓰지 않고 중단한다):

```bash
python -I artifacts/validation/2026-11-06/official_track_audit_v1/tools/zip_audit.py   # Windows
# WSL, 산출물 폴더에서
python3 -I tools/static_audit.py
set +u; source /opt/ros/jazzy/setup.bash; set -u
python3 -I tools/gz_audit.py load --run-id <새 번호>
python3 -I tools/gz_audit.py render --run-id <새 번호> --timeout 300
```

## 사진

- 원본 PNG: `checks/gazebo/render_02/images/`. 판정용 표기 사본: `annotated/`(검출 ID·마커 투영 상자·램프 투영 원). 원본 PNG는 표기하지 않는다.
- 전체: `overview_top.png`(25 m 수직, 6.57 mm/px), `overview_oblique_se.png`, 표기본 `annotated/overview_top_labeled.png`
- 부분: `detail_start_grid`·`detail_P22`·`detail_P21`·`detail_corner_C1`(수직), `oblique_P21_triangle`·`oblique_start_signal`(사선)
- 차량 카메라(현행 B0183 시뮬 구성 1280×720·수평화각 75°·x +0.055·z 0.148 m·수평): `grid0_cam`–`grid5_cam`, `id20_d080_c00` 등(d=카메라–마커판 거리, `r20`/`l20`=진행 방향 오른쪽/왼쪽 0.2 m)
- LiDAR 광축 높이(0.078 m) 광각: `lidar_front_grid`·`lidar_P22_approach`·`lidar_P21_approach`
- 정적 기하 그림: `checks/plots/plan_overview.png`(PDF 벽선 겹침), `zoom_*`, `detail_*`

## 도구 수정 이력

모두 감사 도구의 오류이며 공식 원본의 결함으로 보고하지 않는다.

- `zip_audit.py` 첫 실행은 필수 목록에 `output_final/` 접두를 빠뜨려 16개를 누락으로 잘못 표시했다. 접두를 고쳐 재실행했고, 재실행은 기존 추출본을 덮어쓰지 않고 바이트 일치만 확인했다.
- `static_audit.py` 첫 실행은 생성기의 4자리 반올림으로 생긴 0.1 mm 이하 틈 때문에 도로 83조각·벽 89조각으로 위상을 계산해 섬 내부 18.7 m²를 노출 바닥으로 잘못 셌고, 그림의 구멍 채우기·map 래스터의 구멍 처리도 틀렸다. 위상 계산에만 0.1 mm 닫기를 적용하고(원시 조각 수·최대 틈은 함께 기록) 그림·래스터를 고쳐 재실행했다.
- `annotate_overview.py`의 도로+잔디 경계 총길이는 이음매 수치 띠 때문에 신뢰할 수 없어 기록에서 뺐다.
- 렌더 run01의 토픽 이름 오류는 위 표와 같다.
