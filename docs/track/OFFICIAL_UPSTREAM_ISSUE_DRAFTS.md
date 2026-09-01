# 주최 측 이슈 기록과 미승인 초안

## 게시 상태

- 아래 `v2026.09.01` ArUco 접근 시인성 초안은 **아직 게시하지 않았다**. 사용자의 별도 명시적 승인 전에는 이슈·댓글을 등록하지 않는다.
- 이슈 #7~#11은 `v2026.09.01`에서 수정됐고 2026-09-01 재조회 시 모두 닫힌 상태다. 같은 내용을 다시 등록하지 않는다.
- 문서 뒤쪽의 `v2026.08.31` 본문은 실제로 등록한 #7의 역사 기록이다.

## 미승인 초안: 차량 접근 방향에서 벽 부착 ArUco의 검출 구간 확인

### 제목

`[확인 요청] v2026.09.01 벽 부착 ArUco의 차량 접근 시 가시 방향 확인`

### 본문

**어떤 파일에서**

`v2026.09.01`의 `track/output_final/world.sdf`, `scene.json`과 이를 생성하는 `track_gen.py`의 코스 ArUco pose 부분입니다.

**무엇을 확인하고 싶나요**

안녕하세요. 이슈 #7~#11을 반영해 주신 `v2026.09.01`을 다시 확인했습니다. 먼저 이전에 제보한 SDF 로드, 마커 재질·경로, 정적 링크 초기화, 벽 부착/ID 30 정적 간섭, 그리드 메타데이터·높이, 출발/결승 기준은 새 배포본에서 모두 수정된 것을 확인했습니다. 감사합니다.

새 마커 pose를 차량 전방 카메라로 확인하던 중, 네 마커가 벽면 법선을 따라 놓여 있어 본선 중심선을 따라 접근할 때 상당히 비스듬하게 보이거나 곡선 뒤로 빠지는 구간이 있었습니다. 현재 pose가 실제 시공 시에도 사용할 최종 방향인지, 아니면 차량이 접근하면서 보기 쉽도록 상류 쪽으로 각도를 둘 예정인지 확인을 부탁드리고 싶습니다.

### 재현 조건

- 배포본: `v2026.09.01`, 커밋 `921f3f9a044f1a38ff849cb8e19d00182dd5533b`
- 원본 ArUco pose·10 cm 판·7 cm 코드·PNG/PBR 재질을 변경하지 않음
- 차량 크기 20×15 cm, 본선 중심선에 정지 배치
- 전방 RGB: 848×480, 수평 69.4°·수직 42.5°, 광학 중심 높이 약 7.8 cm
- OpenCV 4.6.0, `DICT_4X4_50`

기본 거리 세 곳의 결과는 다음과 같았습니다.

| ID | 1.50 m 전 | 1.00 m 전 | 0.75 m 전 |
|---:|:---:|:---:|:---:|
| 0 | 실패 | 검출 | 검출 |
| 20 | 실패 | 실패 | 검출 |
| 45 | 실패 | 검출 | 검출 |
| 30 | 실패 | 검출 | 실패 |

ID 30을 0.1 m 간격으로 더 확인했을 때 1.2·1.1·1.0 m에서만 검출됐고, 0.9 m 이후에는 곡선과 벽 때문에 영상 밖으로 벗어났습니다.

ID 30, 1.5 m 전(검출 실패):

![ID 30 1.5 m 전](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/validation/2026-09-01/official_update/marker_vehicle_view/id30_150cm_fail.png)

ID 30, 1.0 m 전(검출 성공):

![ID 30 1.0 m 전](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/validation/2026-09-01/official_update/marker_vehicle_view/id30_100cm_pass.png)

ID 30, 0.75 m 전(화면 밖, 검출 실패):

![ID 30 0.75 m 전](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/validation/2026-09-01/official_update/marker_vehicle_view/id30_075cm_fail.png)

다만 약 0.785 m/s의 저속 본선 한 바퀴에서는 ID 0·20·30·45를 모두 한 번 이상 검출했습니다. 따라서 현재 마커가 인식 불가능하다고 주장하는 것은 아닙니다. 정지 표본의 검출 구간이 짧고 비연속적이어서, 목표 최고속도·다른 차량 가림·실물 카메라의 노출과 모션 블러를 고려할 때 pose의 의도를 미리 확인하려는 요청입니다.

가능하시다면 다음 중 어느 쪽이 의도인지 알려주실 수 있을까요?

1. 공개 SDF처럼 마커 판을 벽면과 평행하게 붙이는 것이 최종 시공 방향
2. 벽 근처 위치는 유지하되 차량 접근 방향을 향하도록 별도 각도를 주는 방식
3. 실제 제작 시 표지판·브래킷을 사용하며 세부 방향은 추후 공유 예정

실제 시공 방향이 아직 미정이라면 현재 파일을 즉시 수정해 달라는 요청은 아닙니다. 팀 쪽에서도 실제 D435i와 인쇄 마커로 속도·각도별 검출 시험을 추가하겠습니다.

**환경**

- 확인일: 2026-09-01
- Windows / WSL2 Ubuntu 24.04.4 LTS / ROS 2 Jazzy
- Gazebo Sim 8.11.0, Ogre2
- OpenCV 4.6.0 `DICT_4X4_50`

### 게시 전 추가 확인

- 실제 D435i와 10 cm 인쇄 마커로 거리·각도·속도별 검출률을 측정한다.
- 운영진이 이미 Discord·회의에서 실제 브래킷 방향을 안내했는지 팀 내부에서 확인한다.
- 정지 표본 1회 통과/실패를 확률적 검출률로 표현하지 않는다.
- 위 GitHub 이미지 URL이 로그인 없이 열리고 증상을 직접 보여주는지 확인한다.
- 사용자의 별도 게시 승인을 받는다.

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
