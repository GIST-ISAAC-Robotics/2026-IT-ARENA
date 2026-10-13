# 후처리 검토 이력

최초 `analysis.json`과 정확한 소스 `analysis_source_v1.py`(SHA-256 `ce72c3122bc3cabfb54e6deb22cb0360fd9b76df11f6c912ab835960467c7e86`)를 보존한다. 원시 기록/실행 결과를 바꾸지 않았다.

v1의 `contact_torque_nm`·`approx_net_spin_torque_nm`는 접촉점과 step 이후 차체 자세로 레버암을 계산했다. DART `World::step`은 접촉/충격량 계산 뒤 위치를 적분하므로, 메시지 timestamp가 같아도 이 둘의 내부 평가 위상은 다르다. 해당 값을 바퀴 회전 방정식의 잔차로 해석하면 안 된다. 20 ms 진단만으로 정확한 pre-step 바퀴 자세/각가속도를 복원할 수 없으므로 **v2에서 두 항목을 제거**하고 한계를 표시했다. 다른 힘/속도/운동량 지표를 유리하게 보정하지 않는다. 최종 결과는 `analysis_final.json`이다.

`upstream/World.cpp`와 `upstream_world_source.json`은 추가 확인 원문이다. 기존 10개 소스 목록은 덮어쓰지 않았다.

v2 실행은 새 좌표계 검사 범위가 너무 넓어 분석에 사용하지 않는 `frame base_link/pose relative_to="__model__"`까지 거부했다. `analysis_source_v2_failed.py`와 `analysis_final.log`에 실패를 보존했다. v3는 실제 소비하는 link/inertia/바퀴 joint/axis/collision pose를 검사하고 무관한 visual/named frame은 제외한다. 다른 물리 좌표계를 조용히 기본 좌표로 읽는 허용은 하지 않으며 회귀를 추가했다. 최종 실행 로그는 `analysis_v3.log`다.

설치 라이브러리 수학 시험은 처음 두 WSL inline 호출에서 PowerShell 변수 확장으로 빌드 전 종료됐다. 별도 `run_probe.sh`로 인자를 분리한 v2 실행이 성공했다. 모델/라이브러리 설치·기존 빌드 수정은 없었다. 빈 임시 폴더가 남았을 가능성은 있으나 넓은 `/tmp` 삭제를 하지 않았다.

v3 분석 자체는 최종 JSON을 생성했지만 inline 셸 래퍼의 종료 코드 변수 처리에서 `numeric argument required`로 도구가 1을 반환했다. 별도 회귀는 16개 통과했다. 마감 시 `finalize.py`가 최종 소스로 분석을 다시 계산해 저장 JSON 전체와 비교하고 독립적인 종료 코드로 결과를 확인한다. 래퍼 실패를 성공으로 고쳐 쓰거나 분석 결과를 덮어쓰지 않는다.

최초 마감은 앞서 정상 실행했던 `/tmp` 빌드의 `solver_probe`가 더 이상 존재하지 않아 실행 파일 해시 수집에서 실패했다. 왜 임시 파일이 사라졌는지는 확인하지 않았다. `finalize_source_v1_failed.py`·`finalize.log`에 실패를 보존한다. 이미 저장한 소스/configure/build/ldd/시험 결과는 유지하고, v2 마감에서 **해당 실행 파일 미보존·해시 없음**을 명시한다. 새로 빌드한 파일을 과거 실행 파일이라고 대신 넣거나 설치 라이브러리 시험을 반복하지 않는다. 최종 마감 로그는 `finalize_v2.log`다.
