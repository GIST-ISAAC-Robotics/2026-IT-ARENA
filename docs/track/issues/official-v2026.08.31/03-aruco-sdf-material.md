**어떤 파일에서**

`track/track_gen.py:write_sdf()`, `track/output_final/world.sdf`의 `aruco_*` material, `track/output_final/aruco/aruco_id*.png`

**무엇이 이상한가요**

네 마커 재질의 `<script>`에 필수 `<name>`이 없어 엄격 SDF 검증이 실패하고, texture 상대경로도 릴리스의 실제 디렉터리 구조와 다릅니다.

**환경**

- 재현 환경: Windows 호스트의 WSL Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.11.0
- repository: `MOSW626/istech-it-arena`
- commit/release: `d61c5db9252cedfbc163cd044a47671df91e1660` / `v2026.08.31`
- 확인 파일: `track/output_final/world.sdf`, `track/output_final/aruco/aruco_id*.png`

## 재현 방법

WSL Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.11.0에서 릴리스 ZIP을 푼 디렉터리로 이동한 뒤 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
gz sdf -k output_final/world.sdf
```

추가로 `world.sdf`의 각 `aruco_*` 재질에서 `<albedo_map>`과 `<script><uri>`를 확인합니다.

## 기대동작

- 배포 디렉터리 구조를 그대로 유지하면 `gz sdf -k output_final/world.sdf`가 오류 없이 통과해야 합니다.
- `output_final/world.sdf`에서 같은 폴더 아래 `output_final/aruco/aruco_id*.png`를 참조해야 합니다.
- `<script>`를 사용할 경우 SDFormat이 요구하는 유효한 `<name>`이 있어야 합니다.

## 실제결과

`gz sdf -k`가 아래 오류를 마커 4개에 대해 출력하고 월드 로드에 실패합니다.

```text
Error Code 8: Msg: A <script> element is missing a child <name> element, or the <name> element is empty.
Error Code 9: Msg: Failed to load a world.
```

또한 SDF는 `../aruco/aruco_id*.png`를 참조하지만 실제 폴더는 `world.sdf`와 같은 `output_final/` 아래의 `aruco/`입니다. 따라서 파일 기준 상대경로는 `aruco/aruco_id*.png`여야 합니다.

로드에 필요한 호환성 변환만 적용하고 좌표·치수는 유지한 실행에서도 ID 30 판이 정상 코드 패턴이 아니라 검게 보이는 증상을 관찰했습니다. 이 시각 증상 자체만으로 UV·노출 등 모든 원인을 단정하지 않으며, 필수 `<name>` 누락과 파일 경로 불일치는 별도로 재현되는 확정 오류입니다.

## 영향

- README에 안내된 `gz sim world.sdf` 사용 경로에서 파서 버전에 따라 월드가 로드되지 않을 수 있습니다.
- 재질 요소를 임의로 제거하거나 파일 구조를 바꾸는 사용자별 workaround가 생겨 재현성이 깨집니다.
- 마커가 검은 판·단색·누락 상태로 렌더링되어 카메라 인식 시험을 오염시킬 수 있습니다.

## 근거자료

- [공식 생성기의 ArUco URI/재질](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L1727-L1744)
- [공식 PNG가 실제로 들어 있는 디렉터리](https://github.com/MOSW626/istech-it-arena/tree/d61c5db9252cedfbc163cd044a47671df91e1660/track/output_final/aruco)
- 위 재현 명령에서 Error Code 8이 4회, 이어서 Error Code 9가 실제 출력됐습니다.

![좌표와 치수를 유지한 호환성 변환 실행에서 검게 보이는 ID 30 판](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/upstream_marker_id30.png)

이 캡처는 원본 좌표에서의 별도 렌더링 증상입니다. 파서 통과용 변환이 적용된 화면을 원본 SDF 자체의 로드 성공 근거로 사용하지 않습니다. 원본 파일의 로드 실패 근거는 위 CLI 오류입니다.

## 수정제안

1. PBR `<albedo_map>`을 유지하면서 URI를 `aruco/aruco_id{id}.png`로 수정합니다.
2. 불완전한 `<script>` 블록이 필요 없다면 제거합니다. 필요하다면 유효한 material script URI와 `<name>`을 함께 제공합니다.
3. 릴리스 생성 단계에 `gz sdf -k output_final/world.sdf` 검사를 추가합니다.
4. 새 ZIP을 실제 압축 해제한 위치에서 texture 파일 존재와 렌더링을 확인합니다.

## 완료 체크리스트

- [ ] `gz sdf -k` 오류 0개
- [ ] `gz sim output_final/world.sdf` 로드 성공
- [ ] 네 ArUco texture가 실제 렌더링됨
- [ ] 상대경로가 릴리스 ZIP 디렉터리 구조와 일치
- [ ] 새 릴리스의 ZIP 해시와 변경 내역 공개
