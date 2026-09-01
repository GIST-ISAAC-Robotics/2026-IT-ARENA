# 주최 측 이슈 기록과 게시 본문

## 게시 상태

- 사용자의 2026-09-01 명시적 승인에 따라 ArUco 설치 방향 질문은 [이슈 #12](https://github.com/MOSW626/istech-it-arena/issues/12), 노면·잔디 공백 문제는 [이슈 #13](https://github.com/MOSW626/istech-it-arena/issues/13)으로 등록했다. 게시 직후 두 이슈는 `OPEN`이었고 아래 본문과 정확히 일치했다.
- 2026-09-02 재조회에서 #12는 실제 방향이 아직 정해지지 않아 운영 회의 안건으로 이관된 뒤 `CLOSED`, #13은 릴리스 `v2026.09.02`에 반영된 뒤 `CLOSED`였다. #13의 수정 수치는 [새 릴리스 재감사](OFFICIAL_V2026_09_02_REAUDIT.md)에서 독립적으로 확인했다.
- 이슈 #7~#11은 `v2026.09.01`에서 수정됐고 2026-09-01 재조회 시 모두 닫힌 상태다. 같은 내용을 다시 등록하지 않는다.
- 계정 권한상 `question`·`bug` 라벨 추가는 GitHub에서 거부돼 두 이슈의 라벨은 비어 있다. 제목과 본문은 각 공식 템플릿 형식을 따른다.
- 문서 뒤쪽의 `v2026.08.31` 본문은 실제로 등록한 #7의 역사 기록이다.

## 등록된 이슈 #12: 벽 부착 코스 ArUco의 실제 설치 방향

적용 템플릿: `.github/ISSUE_TEMPLATE/question.md` (`❓ 질문`)

### 제목

`[질문] 벽 부착 코스 ArUco의 실제 설치 방향`

### 본문

**질문**

`v2026.09.01`의 코스 ArUco 판은 각 지점의 벽면과 평행하게 배치되어 있습니다. `track/README.md`의 “트랙을 향해 벽에 부착”한다는 안내가 실제 시공에서도 SDF의 pose와 같은 방향을 뜻하는지 궁금합니다.

본선 진행 방향에서 보면 일부 마커가 벽면을 따라 상당히 비스듬하게 보입니다. 정지 표본에서는 1.50 m 전에서 네 마커 모두 검출되지 않았고, 검출이 확인된 가장 먼 거리는 ID에 따라 0.75~1.20 m였습니다. 이번 표본처럼 마커를 검출할 수 있는 거리가 짧으면 차량이 빠르게 주행하거나 다른 차량이 시야를 일부 가리는 상황에서 인식 기회가 부족할 수 있습니다. 따라서 공개 SDF의 벽 평행 방향이 실제 시공에서도 사용할 방향인지 확인하고 싶습니다.

실제 설치는 다음 중 어느 방식으로 계획하고 계신가요?

1. 공개 SDF와 같이 판을 해당 벽면과 평행하게 부착
2. 위치는 벽면에 유지하되, 접근하는 차량을 향하도록 판에 별도 각도를 적용
3. 별도 표지판이나 브래킷을 사용하며 세부 방향은 추후 공유

설치 방향이 아직 정해지지 않았다면 미정 상태인지 알려주셔도 충분합니다.

**해본 것 / 참고한 문서**

- 배포본: `v2026.09.01`, 커밋 `921f3f9a044f1a38ff849cb8e19d00182dd5533b`
- 확인 파일: `track/README.md`, `track/output_final/world.sdf`, `track/output_final/scene.json`, `track/track_gen.py`
- 원본 ArUco pose·10 cm 판·7 cm 코드·PNG/PBR 재질을 변경하지 않았습니다.
- 차량 크기 20×15 cm, 본선 중심선에 정지 배치했습니다.
- 전방 RGB는 848×480, 수평 69.4°·수직 42.5°, 광학 중심 높이 약 7.8 cm이며 OpenCV 4.6.0의 `DICT_4X4_50`으로 확인했습니다.

기본 거리 세 곳의 결과는 다음과 같았습니다.

| ID | 1.50 m 전 | 1.00 m 전 | 0.75 m 전 |
|---:|:---:|:---:|:---:|
| 0 | 미검출 | 검출 | 검출 |
| 20 | 미검출 | 미검출 | 검출 |
| 45 | 미검출 | 검출 | 검출 |
| 30 | 미검출 | 검출 | 화면 밖 |

ID 30, 1.5 m 전:

![ID 30 1.5 m 전](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/validation/2026-09-01/official_update/marker_vehicle_view/id30_150cm_fail.png)

ID 30, 1.0 m 전:

![ID 30 1.0 m 전](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/validation/2026-09-01/official_update/marker_vehicle_view/id30_100cm_pass.png)

약 0.785 m/s의 저속 본선 한 바퀴에서는 ID 0·20·30·45를 모두 한 번 이상 검출했습니다. 따라서 현재 마커가 인식 불가능하다는 주장은 아니며, 주행 속도가 높아졌을 때 짧은 관측 구간이 충분한지와 실제 설치 방향을 미리 확인하려는 질문입니다.

### 게시 후 검증

- 사용자의 명시적 승인 뒤 `leejinh0225` 계정으로 2026-09-01 등록했다.
- GitHub에 게시된 제목·본문이 이 문서의 최종본과 정확히 일치하고 상태가 `OPEN`임을 다시 조회했다.
- 두 GitHub 이미지 URL은 게시 직전 로그인 없는 요청에서 HTTP 200을 확인했다.
- `question` 라벨은 저장소 권한 부족으로 추가되지 않았다.
- 2026-09-02 답변에서는 실제 설치 방향이 미정이며 운영 회의에서 다룰
  예정이라고 확인했다. 현재 공개 생성기의 벽 평행 방향을 최종 시공 사양으로
  승격하지 않는다.

---

## 등록된 이슈 #13: 급커브 노면과 갈림길 주변 잔디의 공백

적용 템플릿: `.github/ISSUE_TEMPLATE/bug.md` (`🐛 파일/트랙 문제`)

### 제목

`[문제] 급커브 노면과 갈림길 주변 잔디에 공백이 생깁니다`

### 본문

**어떤 파일에서**

`v2026.09.01`의 다음 파일에서 확인했습니다.

- `track/track_gen.py`의 `build_geometry_boxes()` 및 `build_ribbon_boxes()`
- `track/output_final/world.sdf`
- `track/output_final/map_with_grass.png`
- 비교용 `track/output_final/preview.png`

**무엇이 이상한가요**

공식 `preview.png`에서는 본선 노면과 본선 양옆 잔디 띠가 매끄러운 곡선으로 표현되지만, 같은 배포본의 `world.sdf`와 `map_with_grass.png`에서는 다음 두 종류의 공백이 나타납니다.

1. 급커브에서 직사각형 노면 조각의 바깥쪽 모서리가 벌어지면서 삼각형 모양으로 지면이 드러납니다. 시각 형상뿐 아니라 같은 크기와 pose의 collision에도 공백이 있습니다.
2. 갈림길에는 전용 잔디 띠가 생성되지 않고, 갈림길 입구에서는 본선 잔디도 잘려 있어 지면이 넓게 드러나는 구간이 있습니다. 생성기 주석에는 갈림길 전용 잔디를 의도적으로 생략한다고 적혀 있으나, 이것이 실제 트랙의 잔디 배치를 뜻하는지 아니면 시뮬레이션 표현만 단순화한 것인지 구분하기 어렵습니다.

공식 생성기의 설계 통로와 실제 노면 상자 합집합을 비교했습니다.

- 설계상 주행 가능 영역: 약 `21.793955 m²`
- 실제 노면 visual/collision 합집합에서 빠진 영역: 약 `0.324238 m²` (`1.4877%`)
- 본선 중심선 자체는 모두 노면 위에 있지만, 가장 심한 급커브 단면에서는 명목 노면 가장자리부터 안쪽으로 약 `0.149 m`까지 노면이 비는 구간이 있었습니다.
- 해당 위치는 중심선 기준 대략 `(x=9.768 m, y=1.244 m)`이며, 공식 지도의 오른쪽 아래 급커브입니다.

공식 `preview.png`:

![연속적인 곡선으로 표현된 공식 preview.png](https://raw.githubusercontent.com/MOSW626/istech-it-arena/921f3f9a044f1a38ff849cb8e19d00182dd5533b/track/output_final/preview.png)

같은 배포본의 `map_with_grass.png`:

![급커브 노면과 갈림길 잔디 공백이 표시된 공식 map_with_grass.png](https://raw.githubusercontent.com/MOSW626/istech-it-arena/921f3f9a044f1a38ff849cb8e19d00182dd5533b/track/output_final/map_with_grass.png)

이 지도에서 흰색은 노면, 중간 회색 띠는 잔디, 바깥 연회색은 일반 지면입니다. 흰색 노면의 바깥쪽 가장자리에 보이는 삼각형 모양의 회색 홈과, 갈림길 주변에서 중간 회색 잔디 띠가 끊기는 부분이 위 현상입니다.

`world.sdf`의 노면 상단은 지면보다 3 mm 높으므로, collision 공백을 바퀴가 지나면 지면으로 내려가는 작은 단차가 됩니다. 현재 20×15 cm 차량의 저속 본선 한 바퀴는 통과했으므로 이 문제 때문에 시뮬레이션을 사용할 수 없다고 판단한 것은 아닙니다. 다만 곡선 외측 주행이나 더 높은 속도의 접촉 거동, 카메라 기반 노면 구분, 동봉 지도를 사용하는 경로 계획에는 차이를 만들 수 있습니다.

또한 현재 `MANUAL.md` §6에는 “잔디 진입”의 상황별 패널티 매핑이 미정 항목으로 남아 있습니다. 이후 잔디 진입 패널티가 확정된다면, 시뮬레이션에서 노면·잔디·일반 지면의 영역이 끊겨 있는 상태는 차량의 잔디 진입 상황을 재현하거나 판정하는 데 혼동을 줄 수 있습니다. 패널티 규칙이 이미 확정됐다는 뜻은 아니며, 현재 생성 형상이 실제 시공 의도와 맞는지 함께 확인을 부탁드리는 내용입니다.

설계한 노면 폭이 급커브에서도 visual/collision으로 연속되도록 생성 방식을 조정할 수 있을지, 그리고 갈림길 전용 잔디 생략과 입구의 잔디 공백이 실제 트랙 의도인지 검토 부탁드립니다.

**환경**

- 확인일: 2026-09-01
- 배포본: `v2026.09.01`, 커밋 `921f3f9a044f1a38ff849cb8e19d00182dd5533b`
- Windows / WSL2 Ubuntu 24.04.4 LTS
- ROS 2 Jazzy / Gazebo Sim 8.11.0 / DART / Ogre2
- 공식 ZIP의 `design_final.json`, `track_gen.py`, `output_final/world.sdf` 및 지도 파일을 변경하지 않고 비교했습니다.

### 게시 후 검증

- 사용자의 명시적 승인 뒤 `leejinh0225` 계정으로 2026-09-01 등록했다.
- GitHub에 게시된 제목·본문이 이 문서의 최종본과 정확히 일치하고 상태가 `OPEN`임을 다시 조회했다.
- 공식 `preview.png`와 `map_with_grass.png`의 raw URL은 게시 직전 로그인 없는 요청에서 HTTP 200을 확인했다.
- 면적·최대 공백 수치는 같은 `v2026.09.01` ZIP으로 두 번 계산해 일치했다. 잔디 진입 패널티는 확정 규칙이 아니라 `MANUAL.md` §6의 미정 항목으로 표현했다.
- `bug` 라벨은 저장소 권한 부족으로 추가되지 않았다.
- 2026-09-02에는 수정 릴리스 `v2026.09.02`가 배포되고 이슈가 닫혔다.
  같은 검사에서 노면 누락 0.0538%, 최대 공백 1.0 mm, 잔디 누락
  0.2311%, 노면 침범 0.1305%로 줄어든 것을 확인했다.

---

## 등록된 이슈 #7 본문: `v2026.08.31` 트랙의 Gazebo 로드 오류와 수정 과정

**어떤 파일에서**

`track/output_final/world.sdf`와 이를 생성하는 `track/track_gen.py`의 `write_sdf()` 부분입니다.

**무엇이 이상한가요**

안녕하세요. [v2026.08.31 배포 파일](https://github.com/MOSW626/istech-it-arena/releases/tag/v2026.08.31)을 압축 해제하고, README에 안내된 대로 `output_final/`에서 다음 명령을 실행했습니다.

```bash
gz sim -v 4 world.sdf
```

월드를 불러오는 단계에서 오류가 발생해 파일을 확인했습니다. 수정 후에도 다음 단계에서 다른 문제가 나타나, 정상적으로 실행하고 마커를 표시하기까지 변경한 내용을 순서대로 공유드립니다.

### 1. ArUco 재질 오류로 월드를 불러오지 못했습니다

다음 오류가 마커 네 개에서 반복됐고, 마지막에 월드 로드 실패가 출력됐습니다.

```text
Error Code 8: Msg: A <script> element is missing a child <name> element, or the <name> element is empty.
Error Code 9: Msg: Failed to load a world.
```

각 마커의 재질에는 아래 블록이 있었는데, 필수 자식인 `name`이 없고 `uri`도 재질 스크립트가 아닌 PNG를 가리키고 있었습니다.

```xml
<script><uri>../aruco/aruco_id0.png</uri></script>
```

같은 재질 안에 PBR의 `albedo_map`이 이미 있으므로, 불완전한 `script` 블록 네 개를 제거하고 PBR 부분은 남겼습니다. 그러자 위 파싱 오류가 사라지고 `gz sdf -k world.sdf`도 통과했습니다.

### 2. 파싱은 통과했지만 첫 물리 갱신에서 진행되지 않았습니다

그다음에는 월드 초기화 로그까지 출력됐지만, 아래처럼 물리 계산을 한 번만 진행하도록 실행해도 **180초 안에 완료되지 않았습니다**.

```bash
gz sim -s -r -v 4 --iterations 1 world.sdf
```

SDF를 살펴보니 정적 형상들이 총 **1,113개 링크**로 나뉘어 있었습니다. 도로·잔디·벽 등 반복 정적 형상의 링크를 합치고, 신호등·마커처럼 별도로 다룰 링크는 남겨 **10개 링크**로 줄여 보았습니다.

병합할 때 각 링크의 위치·회전을 내부 `visual`과 `collision`에 옮겨, 형상의 위치와 크기는 유지했습니다. 충돌체 **1,107개**, 시각 형상 **1,113개**와 접촉 설정도 그대로였습니다.

이 변경 후에는 같은 명령이 **약 2.84초에 첫 갱신을 마치고 정상 종료**했습니다. 시간은 프로그램 시작부터 종료까지입니다. 테스트한 WSL/DART 환경에서 효과를 확인한 것이며, 다른 환경에서도 반드시 같은 지연이 발생하거나 링크 병합만이 해결책이라고 판단한 것은 아닙니다.

### 3. 실행 후 ArUco 이미지 파일을 찾지 못했습니다

물리 계산이 진행된 뒤 마커 영상을 확인하니 판이 검게 보였고, 렌더러에는 다음 오류가 나왔습니다.

```text
Unable to find file [../aruco/aruco_id0.png]
```

나머지 세 마커도 같은 상태였습니다. 실제 파일은 `output_final/aruco/`에 있는데, `world.sdf`의 `albedo_map`은 한 단계 위 폴더를 가리키고 있었습니다.

따라서 네 경로를 `../aruco/aruco_id*.png`에서 `aruco/aruco_id*.png`로 바꾸었습니다. 파일을 찾지 못한다는 오류는 사라졌지만, 마커는 여전히 검게 표시됐습니다.

### 4. 재질 색을 명시하자 마커 패턴이 표시됐습니다

그 상태에서 각 마커의 `material`에 **`<diffuse>1 1 1 1</diffuse>`만 추가**했습니다. ID 0의 최종 재질은 다음과 같습니다.

```xml
<material>
  <pbr>
    <metal>
      <albedo_map>aruco/aruco_id0.png</albedo_map>
    </metal>
  </pbr>
  <diffuse>1 1 1 1</diffuse>
</material>
```

그러자 PNG 패턴이 표시됐고 **ID 0·20·30·45가 모두 검출**됐습니다. PNG, 마커 위치·방향, 조명과 카메라 조건은 바꾸지 않았습니다. `diffuse`는 필수 요소가 아니라 기본값이 검정인 선택 요소로, 1번의 필수 `name` 누락과는 다른 문제였습니다. [SDFormat 1.8 재질 명세](https://sdformat.org/spec/1.8/material/)

아래는 같은 위치의 ID 30을 같은 카메라로 본 비교입니다. 사진의 밝기나 패턴을 편집하지 않았습니다.

경로만 수정한 상태:

![경로 수정 후에도 검게 표시된 ID 30](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/validation/2026-08-31/issue_consolidation/retest/render/03_black_material/marker_30.png)

`diffuse` 흰색을 추가한 상태:

![diffuse 흰색 추가 후 패턴이 표시된 ID 30](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/validation/2026-08-31/issue_consolidation/retest/render/04_ready/marker_30.png)

### 최종 확인

위 수정을 모두 적용한 파일은 SDF 검사와 **물리 계산 2,000회 후 정상 종료**를 통과했습니다. 별도로 물리 엔진과 카메라를 함께 켠 실행에서도 각 카메라의 시뮬레이션 시각이 약 0초에서 4.5초까지 진행했고, 네 마커가 표시·검출된 뒤 서버와 영상 bridge가 정상 종료했습니다.

마커 확인에는 정면 35 cm의 고정 카메라를 사용했습니다. 차량 주행 성능을 시험한 것은 아니며, 코스와 시설의 치수·배치는 변경하지 않았습니다. [단계별 실행 로그·수정 내역](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/tree/main/artifacts/validation/2026-08-31/issue_consolidation)

배포 SDF와 생성기의 재질·경로 부분을 확인해 주실 수 있을까요? 정적 링크 구조도 함께 검토 부탁드립니다. 별도로 권장하시는 Gazebo 버전이나 물리 엔진이 있다면 그 환경에 맞춰 확인해 보겠습니다.

**환경**

- 확인일: 2026-08-31
- 배포본: `v2026.08.31`, 커밋 `d61c5db9252cedfbc163cd044a47671df91e1660`
- Windows / WSL2 Ubuntu 24.04.4 LTS / ROS 2 Jazzy
- Gazebo Sim 8.11.0, libsdformat 14.9.0
- 실제 로드된 물리 엔진: DART (`gz::physics::dartsim::Plugin`)
- 영상 확인: Ogre2, 640×480·2 Hz·수평 화각 0.65 rad, OpenCV 4.6.0 `DICT_4X4_50`
