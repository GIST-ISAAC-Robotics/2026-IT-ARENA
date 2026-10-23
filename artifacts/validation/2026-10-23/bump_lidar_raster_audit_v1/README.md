# 저장된 바닥 반환의 cubemap 픽셀 대조

`analyze.py`를 WSL에서 1회 실행해 exit 0. Gazebo를 다시 실행하거나 원본 스캔/판정을 수정하지 않았다. `report.json`에 입력·설치 파일 해시와 광선별 사후 대조가 있다.

- 설치 gz-rendering **8.2.3**의 GLSL 1차 pass는 픽셀 중심의 3D 거리, 2차 pass는 cubemap 최근접 값(`filtering none`)을 읽는다. 같은 버전 태그 `Ogre2GpuRays::ConfigureCamera`의 크기 산정은 이 500광선 설정에서 128×128이다. C++ 바이너리 전체가 원문과 동일하다는 인증은 아니다.
- 수평 광선이 놓인 v=0.5와 선택된 아래쪽 픽셀 중심 사이에는 약 **0.447614°**의 차이가 있다. 관측 방향 `(1, quantized tan(yaw), -1/128)`의 평지 교점으로 대조했다.
- 전방 ±10°·3자세·3스캔 총 243광선의 hit/no-hit가 일치했다. 유한 반환 81개 최대 잔차 **0.000003730612 m**. 센서 오차를 맞춘 계수가 아니라 이 저장 영상의 표본 기하 설명이다.
- 원래 geometry 비교 실패와 종료 실패는 그대로다. 다른 장면/반사/가림·GPU 경로·동적 차량·실제 C1의 오류 상한이나 보정 정책으로 확대하지 않는다. 시뮬레이션 센서 출력도 유한 렌더링 근사를 포함하며 “추가 잡음 없음”이 수학적으로 완벽한 광선이라는 뜻은 아니다.
- 원문: [8.2.3 Ogre2GpuRays.cc](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8_8.2.3/ogre2/src/Ogre2GpuRays.cc). 설치 shader/material 경로·해시는 보고서에 수록했다.
