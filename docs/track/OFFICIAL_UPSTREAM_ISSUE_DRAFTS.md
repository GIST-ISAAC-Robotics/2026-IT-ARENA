# [문제] v2026.08.31 트랙의 Gazebo 로드 오류와 수정 과정

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
