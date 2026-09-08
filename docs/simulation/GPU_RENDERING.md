# WSL GPU 렌더링 복구와 검증

작성: 2026-09-07. 실물 센서 성능이나 차량 설계 변경과 구분합니다.

최종 상태: **GPU 선택 가능, 500 Hz 순차 LiDAR 시험에서 전체 시간 약 33% 감소 확인, 일반 실행 안정성은 미확정**. 기본값은 `system`으로 유지합니다.
9월 8일 [최종 추가 진단](GPU_FINAL_DIAGNOSTIC_2026_09_08.md)에서 센서 단독 이득과 실제 순차 주행·정상 종료 1회를 확인했습니다. 아래 일반 코스의 성능 무이득·간헐적 종료 실패는 보존하며, 새 단일 통과로 해결됐다고 하지 않습니다.
수명 보조 적용 후 첫 시험은 통과했지만 두 번째는 Gazebo 종료 대기 후 강제 종료였습니다.
GPU 경로는 명시적 비교용이며, 일반 실행의 안정된 개선으로 채택하지 않습니다.

## 확인한 문제

- 기본 OpenGL은 `llvmpipe`, `Accelerated: no`였습니다. NVIDIA 장치명 지정만으로는 바뀌지 않았고 `GALLIUM_DRIVER=d3d12`를 함께 지정하면 RTX 5070 Laptop GPU의 가속 렌더링이 확인됐습니다.
- Gazebo 기본 센서 예제에서도 GPU 렌더 스레드 종료 중 SIGSEGV를 재현했습니다. 프로젝트 트랙·차량·ROS 제어가 없어도 발생했습니다. GDB에서 렌더 스레드는 이름이 해석되지 않는 주소에서 충돌하고 주 스레드는 `SensorsPrivate::Stop()`의 join을 기다렸습니다.
- [WSLg #1497](https://github.com/microsoft/wslg/issues/1497)의 동일 Mesa 25.2.8에서 보고된 core 해제·TLS 종료 문제와 증상이 일치합니다. 이 PC의 모든 내부 심볼을 확인한 것은 아니므로 upstream과 동일한 원인이라는 주장은 강한 추론입니다.
- D3D12 core와 로더를 직접 미리 불러오는 시도는 초기화 충돌로 실패했습니다. 원문을 보존하며 최종 코드에서는 제거했습니다.

## 적용 방식

`simulation.launch.py`와 `demo.launch.py`의 `render_backend`:

| 값 | 동작 |
|---|---|
| `auto` | WSL GPU 연결이 있으면 D3D12/NVIDIA 경로. 명시적 기존 드라이버 환경변수는 존중. 다른 Linux에서는 기존 환경 |
| `system` | 이전처럼 실행 환경 그대로 사용 |
| `wsl_nvidia` | D3D12 경로 명시. 어댑터명은 기존 `MESA_D3D12_DEFAULT_ADAPTER_NAME` 또는 NVIDIA |
| `software` | llvmpipe CPU 비교 실행 |

GPU 선택 시 Gazebo 서버·GUI에만 `libarena-wsl-d3d12-lifetime.so`를 적용합니다.
이 보조 기능은 `dlopen`의 대상 파일명이 정확히 `libd3d12core.so`인 경우에만
`RTLD_NODELETE`를 추가합니다. 나머지 라이브러리 로딩은 원래 함수로 전달합니다.
core 초기화 시점은 유지하면서 프로세스 종료 전 코드 언매핑을 방지합니다.
프로세스가 끝나면 운영체제가 메모리를 회수합니다. 시스템 라이브러리·전역 셸 설정은 변경하지 않습니다.

프로젝트 경로에 공백이 있으므로 `LD_PRELOAD`에는 파일명만 넣고
`LD_LIBRARY_PATH`에 패키지 lib 경로를 추가합니다. 기존 값은 뒤에 보존합니다.
누락된 보조 라이브러리를 무시하고 진행하지 않으며 빌드 안내와 함께 중단합니다.

```bash
colcon build --symlink-install --packages-select arena_gazebo
source install/setup.bash
ros2 launch arena_bringup simulation.launch.py headless:=true render_backend:=wsl_nvidia
```

기존 동작 재현은 `render_backend:=system`, CPU 고정은 `render_backend:=software`입니다.
드라이버/Mesa/WSLg 업데이트 후 기본 센서 예제와 프로젝트의 센서·주행·실제 정지·모든 자식 종료를
보조 기능 없이 반복 통과하면 이 우회를 제거할 수 있습니다. GPU 어댑터를 찾지 못한 경우
Mesa가 다른 장치를 선택할 수 있으므로 요청값만 보지 않고 실제 `GL_RENDERER`를 확인합니다.

## 비교 조건과 결과 해석

공식 코스, 같은 차량 설정 해시, RGB·깊이 848×480·30 Hz, ToF6 8×8·15 Hz,
기본 순간 LiDAR, GUI/녹화 없음, 영상 150개(약 4.918 sim초) 계측 후 직진·조향·정지를 사용했습니다.
CPU와 최초 GPU 조건은 각각 0.11843·0.11743 RTF로 속도 개선이 없었습니다.
시뮬레이션 시간당 RGB/깊이 수신은 양쪽 약 30.297 Hz였습니다.
CPU는 22개 자식이 모두 정상 종료했고, 최초 GPU는 Gazebo 강제 종료로 실패했습니다.
첫 비교 소스는 `initial_source/`, 최초 보고서는 원문 그대로 보존합니다.

최종 보조 기능 적용 실행과 검사 결과는 [산출물](../../artifacts/validation/2026-09-07/gpu_audit/)을 확인합니다.
한 쌍의 짧은 시험을 장시간 성능·GPU 우열·모든 과거 실행의 원인으로 확대하지 않습니다.
센서 내용의 픽셀 단위 일치·ArUco 인식·장시간 녹화·순차 LiDAR 500 Hz 부하는 별도 검증입니다.

## 출처

- [Mesa llvmpipe](https://docs.mesa3d.org/drivers/llvmpipe.html)
- [Mesa D3D12 및 어댑터 선택](https://docs.mesa3d.org/drivers/d3d12.html)
- [WSLg 가속 구조](https://github.com/microsoft/wslg/blob/main/README.md)
- [Gazebo 화면 없는 렌더링](https://gazebosim.org/api/sim/8/headless_rendering.html): EGL 사용은 별도이며 `headless:=true`의 GUI 생략과 혼동하지 않습니다.
