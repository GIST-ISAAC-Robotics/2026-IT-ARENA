# GPU 경로 점검·수정·비교

결론: RTX 렌더링 선택은 확인했지만 실행 속도 개선과 종료 안정성은 해결하지 못했습니다.
기본 실행은 `render_backend:=system`으로 보존합니다.

| 실행 | 렌더러 | RGB 계측 RTF | 종료 |
|---|---|---:|---|
| cpu_01 | llvmpipe | 0.11843 | 22/22 정상 |
| gpu_01 | RTX D3D12 | 0.11743 | Gazebo 강제 종료 |
| gpu_final_01 | RTX D3D12 + core 수명 보조 | 0.11231 | 22/22 정상 |
| gpu_final_02 | RTX D3D12 + core 수명 보조 | 0.10980 | Gazebo 강제 종료 |

동일 차량 설정·공식 코스·RGB/깊이 848×480@30·ToF6·영상 150개(약 4.918 sim초)를 비교했습니다.
RGB/깊이 시뮬레이션 수신은 모두 약 30.297 Hz입니다. 녹화/GUI 없음, 기본 순간 LiDAR입니다.
한 쌍/두 번의 짧은 실행이며 순차 LiDAR·장시간·마커/픽셀 동등성 검증이 아닙니다.

- `comparison.json`: 원문 report와 모든 자식 종료를 재감사한 집계. `summarize.py`로 재생성합니다.
- 각 실행 폴더: 원문 보고서와 ROS launch 로그. 실패 판정은 유지합니다.
- `*_ogre2.log`: 실행별 실제 렌더러 로그.
- `initial_source/`: 최초 CPU/GPU 비교의 소스 사본. 최종 수정과 구분합니다.
- `gpu_diagnostic*`: 종료 스택 부착 시도. 부착 전에 SIGSEGV라 스택 수집 실패.
- `gpu_preload_01*`: D3D12 직접 preload 실패. 최종 설정에서 제거했습니다.
- `minimal_debug*`: Gazebo 기본 센서 예제의 종료 SIGSEGV 및 GDB 스택.
- `minimal_nodelete*`: 첫 공백 경로 로딩 실패와 수정 후 최소 예제 정상 종료.
- `tests_final.txt`, `cpp_tests.txt`, `build.txt`: 최종 검사·빌드 결과.

진단용 실행 스크립트·보조 소스/바이너리는 실험 이력입니다. 일반 실행에는 설치된
`arena_gazebo` 보조 라이브러리만 선택적으로 사용합니다. 상세 설명은
[GPU_RENDERING.md](../../../../docs/simulation/GPU_RENDERING.md)를 확인합니다.
