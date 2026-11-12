# 실제 peripheral 결합 첫 시험 r298

**통신 실패다.** 기존 r289 출력 비활성 앱을20초 실행했으나 IA/IMU 프레임0, 비프레임25,388바이트만 받았다. 이 원문은 부팅/오류 출력의 baud 차이 가능성이 있으며 원인은 아직 미확정이다. 관련 PC mux 검사 통과와 구분한다.

시험 직전 factory app1MiB를 백업하고 동일 영역만 시험 앱으로 바꿨다. 시험 후 **전체1MiB SHA `eaf5c549…df6ae` 일치**로 원 IMU 앱을 복구했다. 비활성 상태 응답·포트 소유자0, 모터 구동 명령/호스트 UART송신0을 확인했다. 시험 앱은 output_ready=false·PWM0 고정이며 읽기 전용 서보 요청만 포함한다. ROM/리셋 동안의 물리 핀 전압은 측정하지 않았다.

[원 결과](result.json)·[회수](recovery.json)·[실행 소스](bench_executed.py)·[후속 부팅 진단](../runtime_boot_r300_v1/README.md). 최초 전달본을 한국어 docstring으로만 수정한 재전송은 목적지 보존 정책으로 거절됐으나 뒤 실행 명령은 계속되어 최초본으로 수행됐다. 두 소스의 실행 본문은 같고 실제 사용본을 따로 보존했다. raw UART·Flash백업은 Git 제외 actual 폴더와 Jetson에 보존한다.
