# r462 최신 GUARD 정책의 estimated 계약 이관 대표 준비

**후속 주 담당 자원 배정으로 승인13회 계산을 완료하고 정상 회수했습니다.** 같은 ARM current5쌍/국소 checkpoint1쌍은 전체 출력 exact이고 verified 위장은 거절했습니다. 원 x86 비교1건의 region count 차이는 실패로 보존했습니다. [최종 결과](RESULT.md), [시각별 처리시간](timing.json), [회수](remote_result/resource_return.json)를 참조합니다. ROS/장치 출력·빌드·WSL 사용은0입니다. 아래는 당시 선정과 고정 recipe 기록이며 새 실행 지시가 아닙니다. 최초 준비본은 `README.preparation.md`에 보존했습니다.

목적은 r459의 제한 자기 가림 접근 분류가 기존 명시 `host_estimated_crop_v1` 계약에서도 `timing_verified=false`, `timing_policy=host_estimated_v1`, 원시 `time_basis=unverified_estimate`를 보존하는지 확인하는 것입니다. 같은 저장 시뮬 수치의 계약 사본이며 실제 센서나 실차 주행이 아닙니다. r459 공통6파일 이관은 실물 profile 선택/구동 완료를 뜻하지 않습니다.

## 선정 입력

| ID | 실제 저장 상태 | 목적 |
|---|---|---|
| r457:1360 | plan(361,514),32.52→32.523s, 측정0, plan/current .15, 양쪽 `observed_with_mask_budget` | 새 r459 분기의 실제 plan→current 연결 |
| r457:1986 | plan(361,843),48.12→48.122s, 측정0, plan .75/current0 | 새 right-mask 정지 보존. 원인 분석은 r463 담당 |
| r452:519 | plan(360,76),11.52→11.522s, 측정 .91837563, current .35 | 기존 양수 GUARD를 일괄 .1로 낮추지 않는지 |
| r452:539 | plan(360,89),12.02→12.022s, 측정 −.061225042, current .209617158 | 음수 축속도 보존·기존 양수 유지 |
| r452:531 | plan(360,85),11.82→11.822s, current0/new side | 실제 유한 측방 반환에 대한 기존 거절 보존 |

마지막 항목은 기록된 새 측방 관측 반환의 거절 대표이며 명확한 물리 접촉의 증거라는 뜻은 아닙니다. 전방/측방 장애물을 새로 주입하거나 원 ray/epoch/속도를 바꾸지 않습니다. 기존 r447의99개 반복도 하지 않습니다.

r456에서 실제 호출 시 복제한 owner/scene/baseline/selected checkpoint(360,573;35.22s;측정0)를 추가1개 선택했습니다. 원 r452에는 이 새 양수 계획의 후속 current가 없으므로 만들지 않습니다. 이 checkpoint는 최신 `_guard_curve`의 국소 계약 동등성만 확인합니다. 전체 상태형 plan을 처음부터 재생하거나 국소 owner/baseline을 새로 맞춰 만들지 않습니다. 원7.64MB pickle에는106개가 있으나 실행 시 지정한1개만 계산하며, 원 SHA를 대조했습니다.

current5개×두 계약10회, checkpoint1개×두 계약2회, estimated 입력의 verified flag만 true로 바꾼 거절1회로 **최대13회**입니다. 추가 warmup/시각민감도/센서 재시험/전체plan prefix 재생은 없습니다.

## 변환과 정책 경계

`contract_reference.py`는 r444 `verify_saved.py` byte exact 사본입니다. `contract_tools.py`는 그 `estimated_header/estimated_record` 함수 AST를 재사용합니다. checkpoint plan 한 개에는 원 변환기의 snapshot for-body만 재사용하여 가짜 current를 생성하지 않습니다. Windows 준비에서11개 raw snapshot을 변환한 뒤 허용 metadata3개를 원래대로 되돌리면 전체 값이 원본과 일치하는 것을 확인했습니다. scan/ranges/점별 시각/history/기하/부호/epoch 수치는 불변입니다. 이 Windows 준비 자체는 runtime 실행이 아니며 후속13회 결과는 별도로 기록했습니다.

현재 공통 코드는 `Evaluator`의 명시 scope/환경 검사와 `timing_matches`를 거쳐 false/host-estimated/basis 조합을 확인합니다. r459 helper 자체는 공간 관측 분류를 다루고 시간을 true로 승격하지 않습니다. 공간 증거의 `basis=observed_with_mask_budget`와 취득시각 `time_basis=unverified_estimate`는 서로 다른 값이며 혼동하지 않습니다. 후속 선택 대표의 실제 수용/출력 동등성은 RESULT에서 확인했으며, 실물 profile 자동 활성화나 모든 입력의 이관 증명은 아닙니다.

**GUARD의 기존 .15 단계와 estimated crop/corner 현재 보완의 단일 .1 목표는 별도 분기입니다.** .1은 원 현재 판단0인 관측 보완 후보에 대한 제약이며 정상 양수 전체의 상한이 아닙니다. 이번 검증은 독립 guard와 runtime_authorized=false 경계를 보존합니다. 노드/실물 선택·MCU/host/배선/서보/전력은 변경하지 않습니다.

## 패키지·검사·실행 recipe

[prepared.json](prepared.json), [source_manifest.json](source_manifest.json), [준비 로그](prepare_v1.log)와 [읽기 전용 환경](jetson_readonly_probe.log)을 참조합니다. 239개 파일과 원 trace/정책6파일의 SHA를 기록했고 AST와 metadata 변환 준비는 exit0입니다. 소스 수정0입니다. zip SHA256은 `b3c14a2be227724fb709f8b8debed33fe3b5ba335f208caed5225202ed996405`, manifest SHA256은 `39b40550153e1405f28938fb342b64fb280fb94b3c2a6619d0c587d05e8a7a9b`입니다.

Jetson 읽기 전용 조회는 ARM64/Python3.10.12, load .03/.03/.00, 관련 작업0을 확인했습니다. 현재 native 소스3개+setup.py4개의 SHA가 r447과 같으며 기존 binary `/home/q/it-arena-r447-a/src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-310-aarch64-linux-gnu.so` SHA는 `a2afb70525be1e86f63ae072abc4fbaeec10d65f0596e5f642f135ba6a1de394`입니다. 기존 폴더는 읽기만 하며 후속 독립 `/home/q/it-arena-r462-a`에 이 파일을 복사해 재사용합니다. 새 native 빌드/최적화는 필요하지 않습니다. 후속 전송·계산 후 ZIP 회수까지 완료했으며 원 r447 binary는 수정하지 않았습니다.

주 담당이 자원을 배정한 뒤 `bundle_sources.zip`을 신규 r462 경로에 해시 확인·압축 해제하고, 같은 경로에 `execution_review.json`을 기록합니다. 필수 필드는 `root_reviewed=true`, `jetson_exclusive_resource_assigned=true`, 위 `manifest_sha256`, `planned_total_computations=13`, `repeat_on_failure=false`입니다. 이는 주 담당 자원 조율을 위한 기록이며 사용자에게 구현별 재승인을 요구하는 절차가 아닙니다.

```bash
cd /home/q/it-arena-r462-a
PYTHONDONTWRITEBYTECODE=1 python3 -B execute_package.py
```

이 recipe는 원 r447의 환경/다른 작업·포트 사용 조회를 재사용합니다. 다른 작업이나 열린 센서 포트가 있으면 시작하지 않습니다. native 재사용 해시를 확인한 뒤 simulation→estimated→summarize를 분리된 프로세스에서 실행하며 각 단계 timeout60초입니다. 실패/잔존은 기록·소유 process group 회수하고 자동 반복하지 않습니다. 센서 포트를 열거나 ROS/host/ARM/flash/전력을 조작하지 않습니다. NumPy pickle 모듈 경로 `_core`→`core` 호환만 준비했으며 실제 ARM unpickle은 계산 전에 성공했고 원 owner 값은 보존했습니다. 실패하면 원 로그를 보존하고 먼저 원인을 보고합니다.

산출물은 `results/{simulation,estimated}`의 실제 입력 contract/명령·이유·proof·원 x86 차이, `results/summary.json`, `execution_receipt.json`입니다. 같은 ARM의 두 계약 비교와 원 x86 저장 결과 비교는 분리합니다. tinyfloat/region count 차이는 원 차이로 남기고 허가 기준을 바꾸지 않습니다. 국소 checkpoint/소수 저장 비교를 전체 ROS 실시간·시각 정확도·실차 출력/안전으로 확대하지 않습니다.
