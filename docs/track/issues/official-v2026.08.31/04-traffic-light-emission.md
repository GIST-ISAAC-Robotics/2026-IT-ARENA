**어떤 파일에서**

`track/track_gen.py:write_sdf()`, `track/output_final/world.sdf`의 `lamp_*`, `track/output_final/traffic_light.py`

**무엇이 이상한가요**

빨강·노랑·초록 visual이 동시에 발광하며 동봉 UDP 제어기는 그 상태를 Gazebo 화면에 반영하지 않습니다. 현재 배포 모델의 상태 문제이며 미공개 실물 신호등의 최종 색 순서를 임의로 확정하려는 요청이 아닙니다.

기존 [#3 신호등 제작](https://github.com/MOSW626/istech-it-arena/issues/3)은 실물 제작·점등 시퀀스 공개, [#5 traffic_light.py 랜덤 홀드](https://github.com/MOSW626/istech-it-arena/issues/5)는 RED 대기 시간의 랜덤화를 다룹니다. 이번 이슈는 두 작업을 중복 요청하지 않고 **동시 emissive와 UDP 상태의 Gazebo 화면 미연동**으로 범위를 한정합니다.

**환경**

- 재현 환경: Windows 호스트의 WSL Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.11.0
- repository: `MOSW626/istech-it-arena`
- commit/release: `d61c5db9252cedfbc163cd044a47671df91e1660` / `v2026.08.31`
- 확인 파일: `track/track_gen.py`, `track/output_final/world.sdf`, `track/output_final/traffic_light.py`, `track/README.md`

## 재현 방법

1. `world.sdf`의 `lamp_red`, `lamp_yellow`, `lamp_green` visual material을 확인합니다.
2. 각 링크의 `<emissive>` 값을 확인하거나 월드를 렌더링합니다.
3. `traffic_light.py`를 실행하고 Gazebo의 세 visual material이 실제로 바뀌는지 확인합니다.

원본 SDF에는 별도의 마커 재질 파서 오류가 있습니다. 아래 실행 화면은 로드용 호환성 변환을 적용했지만, 신호등의 좌표·치수·세 색 emissive 값은 바꾸지 않았습니다. 원본 파일 자체의 상태는 1~2번의 정적 검사로도 확인할 수 있습니다.

## 기대동작

- 기본 시험 상태는 활성 색이 분명해야 합니다. 실제 신호 체계가 확정되기 전에는 단일 색 등 문서화된 임시 상태를 제공하고, 실물에서 복수 색을 함께 켜는 상태가 있다면 이후 별도 명시해야 합니다.
- 시작 상태와 상태 전환 방식이 scene/README에 명시되어야 합니다.
- 실전 규칙처럼 차량은 RGB 영상에서 출발 신호를 판단해야 하며, UDP는 정답 입력이 아니라 시뮬레이션 조작 수단이어야 합니다.

## 실제결과

- 세 visual 모두 자신의 RGB 색이 ambient/diffuse/emissive에 설정되어 빨강·노랑·초록이 동시에 켜진 것처럼 보입니다.
- 동봉된 `traffic_light.py`는 UDP JSON 상태를 송신하지만 Gazebo SDF visual을 직접 변경하지 않습니다.
- README는 UDP가 시뮬레이션 편의용이라고 설명하지만, 배포된 월드에는 그 상태를 화면에 연결하는 수신/제어 구성요소가 없습니다.

## 영향

- 카메라 기반 출발 인식에서 활성 색의 정답이 정의되지 않습니다.
- 세 색 동시 발광을 학습하거나 색 위치만 외우는 잘못된 시뮬레이션 파이프라인이 생길 수 있습니다.
- UDP의 출발 상태와 차량이 보는 영상의 상태가 달라 시험 결과를 해석하기 어렵습니다.

## 근거자료

- [공식 생성기의 emissive 설정](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L1706-L1725)
- [동봉 UDP 스크립트](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/output_final/traffic_light.py)
- [실전 시각 출발과 고정 타이밍 주의](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/README.md#6-시뮬--실전-차이-개발-시-반드시-감안)

![배포 SDF의 빨강 노랑 초록 동시 활성 상태](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/upstream_signal_all_lit.png)

## 수정제안

1. 기본 월드는 한 상태만 emissive로 생성하고 나머지 렌즈는 어두운 재질로 둡니다.
2. Gazebo visual material을 실제로 변경하는 시스템/서비스/토픽 어댑터를 제공합니다.
3. 차량 측에는 RGB만 제공하고 UDP 상태는 경기 제어기 또는 평가기에서만 사용합니다.
4. 기존 #3의 후속 실물 사양 및 #5의 시퀀스 변경과 연동하되, 어떤 제어 상태든 활성 색이 화면에 반영되는지 독립적으로 검사합니다.

## 완료 체크리스트

- [ ] 시작 시 단일 색만 emissive
- [ ] RED/YELLOW/GREEN 각 상태의 실제 렌더링 전환 확인
- [ ] 차량 제어 입력에 UDP 상태가 노출되지 않음
- [ ] 기존 #5의 제어 시퀀스 변경과 무관하게 활성 상태가 화면에 반영됨
- [ ] 실물 신호등 후속 자료 상태 문서화
