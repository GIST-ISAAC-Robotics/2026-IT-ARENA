# 시뮬레이터 GPU 최종 진단 산출물

해석과 적용 판단은 [진단 문서](../../../../docs/simulation/GPU_FINAL_DIAGNOSTIC_2026_09_08.md)를 확인합니다.

- `sensor_profile.cpp`, `CMakeLists.txt`, `build.sh`: Gazebo Rendering 센서만 직접 계측합니다. 단순 상자 장면이며 공식 월드나 로봇 자율주행 코드가 아닙니다.
- `run.py`, `run.sh`: CPU/GPU 각 두 번, GPU EGL 한 번을 직렬 실행했습니다. 시간에 결과 회수가 포함되며 준비·초기화·종료는 별도입니다. `results.json`에 실제 렌더러와 반환 코드를 기록합니다.
- `cpu_*`, `gpu_*`: 실행 출력과 매번 보존한 Ogre 로그입니다. `gpu_egl_*`은 EGL 전용 시도입니다. 센서 단독 EGL 통과를 Gazebo 종료 안정성으로 확대하지 않습니다.
- `run_sequential.sh`: 기존 5 km/h 직선·10 Hz 순차 LiDAR·500 Hz 원시 광선 시험의 CPU/GPU 비교입니다. 원래 차량/물리·제어 설정을 유지합니다. GPU 선택 외에는 같은 조건입니다.
- `sequential_software/`, `sequential_wsl_nvidia/`: 전체 시뮬레이션의 보고서·명령·궤적·센서 취득·원문 로그입니다. 전체 벽시계 시간에는 준비·정지·종료가 포함됩니다.
- `summarize.py`, `summary.json`: 센서 단독 평균과 실제 주행의 판정·종료·입력 해시를 모읍니다.

재실행은 WSL Ubuntu-24.04에서 저장소 루트 기준 `bash artifacts/validation/2026-09-08/gpu_final/build.sh` 후 해당 실행 스크립트를 사용합니다. 기존 결과 덮어쓰기 방지가 있으므로 별도의 새 산출물 위치로 복사/조정한 뒤 실행합니다. `arena_gazebo`의 D3D12 수명 보조 라이브러리 빌드가 선행되어야 합니다.

최초 빌드 스크립트는 상위 폴더를 한 단계 더 올라가는 경로 오류로 구성에 실패했고, 저장소 루트 기준으로 바로잡은 뒤 빌드했습니다. 이 실패에서는 센서 계측을 실행하지 않았습니다.
